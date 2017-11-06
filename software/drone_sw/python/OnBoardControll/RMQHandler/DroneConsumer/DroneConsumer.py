import kombu
from kombu.mixins import ConsumerMixin, ConsumerProducerMixin
import logging
import json
import datetime
import threading
from GeoFenceChecker.GeoFenceChecker import fencechecker

class DroneAbortWorker(ConsumerProducerMixin):
    def __init__(self, connection, drone_id, drone, gps_monitor, connection_monitor):
        self.drone_id = drone_id
        self.drone = drone
        self.connection = connection
        self.gps_monitor = gps_monitor
        self.connection_monitor = connection_monitor
        self.heartbeat_lock = threading.RLock()
        self.last_gcs_heartbeat = datetime.datetime.now()
        self.droneAbortExchange = kombu.Exchange(name="droneabort", type="topic", durable = False)
        self.droneActionExchange = kombu.Exchange(name="droneaction", type="topic", durable = False)
        self.droneExchange = kombu.Exchange(name="drone", type="topic", durable = False)
        self.droneSoftAbort = kombu.Queue(exchange=self.droneAbortExchange, routing_key="drone.softabort.{}".format(drone_id), exclusive = True)
        self.droneHardAbort = kombu.Queue(exchange=self.droneAbortExchange, routing_key="drone.hardabort.{}".format(drone_id), exclusive = True)
        self.droneLand = kombu.Queue(exchange=self.droneActionExchange, routing_key="drone.land.{}".format(drone_id), exclusive = True)
        self.droneTakeoff = kombu.Queue(exchange=self.droneActionExchange, routing_key="drone.takeoff.{}".format(drone_id), exclusive = True)
        self.gcsheartbeat =  kombu.Queue(exchange=self.droneExchange, routing_key="gcs.heartbeat.{}".format(drone_id), exclusive = True)
        self.droneFence = kombu.Queue(exchange = self.droneExchange, routing_key = "drone.geofence.{}".format(drone_id), exclusive = True)
        self.is_ready_qeueue = kombu.Queue(exchange = self.droneExchange, routing_key = "drone.ready_rpc.{}".format(drone_id), exclusive = True)

    def get_consumers(self, Consumer, channel):
        return [
                Consumer(queues = [self.droneSoftAbort], callbacks = [self.DroneSoftAbortCallback], prefetch_count = 1),
                Consumer(queues = [self.droneHardAbort], callbacks = [self.DroneHardAbortCallback], prefetch_count = 1),
                Consumer(queues = [self.droneLand], callbacks = [self.DroneLandCallback], prefetch_count = 1),
                Consumer(queues = [self.droneTakeoff], callbacks = [self.DroneTakeoffCallback], prefetch_count = 1),
                Consumer(queues = [self.gcsheartbeat], callbacks = [self.GCSHeartbeatCallback], prefetch_count = 1),
                Consumer(queues = [self.droneFence], callbacks = [self.fencecallback], prefetch_count = 1),
                Consumer(queues = [self.is_ready_qeueue], callbacks = [self.rpc_is_ready], prefetch_count = 1),
                ]
    def on_connection_error(self, exc, interval):
        super(DroneAbortWorker, self).on_connection_error(exc, interval)
        logging.error("Failed to reconnect. Connection interval is {}".format(interval))

    def on_connection_revived(self):
        logging.error("Connection was established")

    def extract_common_data(self, body, message):
        drone_id = message.delivery_info["routing_key"].split('.')[-1]
        data = json.loads(body)
        time = datetime.datetime.strptime(data["time"], "%Y/%m/%d_%H:%M:%S")
        return (drone_id, data, time)

    def DroneHardAbortCallback(self, body, message):
        drone_id, data, time = self.extract_common_data(body, message)
        logging.debug("Performing Hard Abort!")
        self.drone.hardabort()
        message.ack()

    def DroneSoftAbortCallback(self, body, message):
        drone_id, data, time = self.extract_common_data(body, message)
        logging.debug("Performing Soft Abort!")
        self.drone.softabort()
        message.ack()

    def fencecallback(self, body, message):
        try:
            fence_data = json.loads(body)["data"]
            fencechecker.intialize(self.drone, fence_data)
            message.ack()
        except:
            logging.exception("Error while initialising geofence checker")

    def DroneLandCallback(self, body, message):
        drone_id, data, time = self.extract_common_data(body, message)
        logging.debug("Performing Soft Abort!")
        self.drone.land()
        message.ack()

    def DroneTakeoffCallback(self, body, message):
        drone_id, data, time = self.extract_common_data(body, message)
        logging.debug("Performing Soft Abort!")
        self.drone.takeoff(data["altitude"])
        message.ack()

    def GCSHeartbeatCallback(self, body, message):
        drone_id, data, time = self.extract_common_data(body, message)
        logging.debug("Recieved GCS heartbeat")
        with self.heartbeat_lock:
            self.last_gcs_heartbeat = datetime.datetime.now()
        message.ack()

    def rpc_is_ready(self, body, message):
        #This function is an rpc call which can be used to get information about whether the onboard computer thinks the drone is ready for flight
        ready_dict = {}
        ready_dict["gpsLock"] = self.gps_monitor.isOperational()
        ready_dict["connectivity"] = self.connection_monitor.getConnectionStatus()
        ready_dict["withinFence"] = fencechecker.getStatus()
        ready_dict["ready"] = not False in ready_dict.values() and not False in ready_dict["connectivity"].values()
        self.producer.publish(
            json.dumps(ready_dict),
            exchange ='', routing_key=message.properties['reply_to'],
            correlation_id=message.properties['correlation_id'],
            retry=True
        )
        message.ack()



    def get_last_gcs_heartbeat(self):
        with self.heartbeat_lock:
            return self.last_gcs_heartbeat
