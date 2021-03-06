import json
import kombu
import datetime
import threading
import Queue
import logging
import socket
from GeoFenceChecker.GeoFenceChecker import fencechecker
class DroneProducer:
    def __init__(self, _id, connection):
        self._id = _id
        self.connection = connection
        self.dronesensor = kombu.Exchange(name="dronesensor", type="topic", durable = False)
        self.droneExchange = kombu.Exchange(name="drone", type="topic", durable = False)
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
        time_str = datetime.datetime.utcnow().strftime("%Y/%m/%d_%H:%M:%S")
        data = {"time" : time_str}
        data["info"] = info
        data["location"] = location
        body = json.dumps(data)
        self.put(body = body, routing_key = "drone.onboard_gps.{}".format(self._id), exchange = self.dronesensor,
            declare =  [self.dronesensor], retry =  False)

    def publish_external_gps(self, info, location):
        #Publish the info about the delution, sattelites etc, as well as the location
        time_str = datetime.datetime.utcnow().strftime("%Y/%m/%d_%H:%M:%S")
        data = {"time" : time_str}
        data["info"] = info
        data["location"] = location
        body = json.dumps(data)
        self.put(body = body, routing_key = "drone.external_gps.{}".format(self._id), exchange = self.dronesensor,
            declare =  [self.dronesensor], retry =  False)

    def request_geofence(self):
        self.put(body = "", routing_key = "drone.geofencerequest.{}".form(self._id), exchange = self.droneExchange)


    def start_publishing(self):
        self.run_thread = threading.Thread(target = self.publisher_thread)
        self.run_thread.daemon = True
        self.run_thread.start()
        logging.info("Started publisher")

    def publisher_thread(self):
        self.heartbeat() #Start sending a heartbeat
        self.geofenceRequester() #Start requesting geofencing
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
        timer = threading.Timer(2.0, self.heartbeat)
        timer.daemon = True
        timer.start() #Call ourself in 1 second
        time_str = datetime.datetime.utcnow().strftime("%Y/%m/%d_%H:%M:%S")
        data = {"time" : time_str}
        logging.debug("Publishing heartbeat")

        self.put(body = json.dumps(data), routing_key = "drone.heartbeat.{}".format(self._id), exchange = self.dronesensor, declare = [self.dronesensor], retry = False)

    def geofenceRequester(self):
        if not fencechecker.initialised:
            timer = threading.Timer(10.0, self.geofenceRequester)
            timer.daemon = True
            timer.start()
            logging.debug("Requesting geofence from GCS")
            self.put(body = "{}", routing_key = "drone.geofencerequest.{}".format(self._id), exchange = self.droneExchange, declare = [self.droneExchange], retry = False)
