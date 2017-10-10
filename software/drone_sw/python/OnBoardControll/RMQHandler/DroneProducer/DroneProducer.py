import json
import kombu
import datetime
import threading
import Queue
import logging
import socket
class DroneProducer:
    def __init__(self, _id, connection):
        self._id = _id
        self.connection = connection
        self.dronesensor = kombu.Exchange(name="dronesensor", type="topic", durable = False)
        self.producer = connection.Producer()
        self.queue = Queue.Queue(maxsize = 100)
        self.run_thread = None

    def put(self, *args, **kwargs):
        try:
            self.queue.put_nowait( kwargs )
        except Queue.Full:
            logging.exception("Exception in droneproducer put")

    def publish_onboard_gps(self, info, location):
        #Publish the info about the delution, sattelites etc, as well as the location
        time_str = datetime.datetime.now().strftime("%Y/%m/%d_%H:%M:%S")
        data = {"time" : time_str}
        data["info"] = info
        data["location"] = location
        body = json.dumps(data)
        self.put(body = body, routing_key = "drone.onboard_gps.{}".format(self._id), exchange = self.dronesensor,
            declare =  [self.dronesensor], retry =  False)

    def publish_external_gps(self, info, location):
        #Publish the info about the delution, sattelites etc, as well as the location
        time_str = datetime.datetime.now().strftime("%Y/%m/%d_%H:%M:%S")
        data = {"time" : time_str}
        data["info"] = info
        data["location"] = location
        body = json.dumps(data)
        self.put(body = body, routing_key = "drone.external_gps.{}".format(self._id), exchange = self.dronesensor,
            declare =  [self.dronesensor], retry =  False)

    def start_publishing(self):
        self.run_thread = threading.Thread(target = self.publisher_thread)
        self.run_thread.daemon = True
        self.run_thread.start()
        logging.info("Started publisher")

    def publisher_thread(self):
        self.heartbeat() #Start sending a heartbeat
        while True:
            try:
                kwargs = self.queue.get()
                self.producer.publish(**kwargs)
            except socket.error:
                logging.warning("Producer connection lost, trying to reestablish")
                self.producer.revive(self.connection)
            except:
                logging.exception("Exception in publisher thread")

    def heartbeat(self):
        threading.Timer(1.0, self.heartbeat).start() #Call ourself in 1 second
        logging.debug("Publishing heartbeat")
        self.put(body = "", routing_key = "drone.heartbeat.{}".format(self._id), exchange = self.dronesensor, declare = [self.dronesensor], retry = False)
