import kombu
from kombu.mixins import ConsumerMixin
import logging
import json
import datetime

class DroneAbortWorker(ConsumerMixin):
    def __init__(self, connection, drone_id, drone):
        self.drone_id = drone_id
        self.drone = drone
        self.connection = connection
        self.droneAbortExchange = kombu.Exchange(name="droneabort", type="topic")
        self.droneActionExchange = kombu.Exchange(name="droneaction", type="topic")
        self.droneSoftAbort = kombu.Queue("dronesoftabort{}".format(drone_id), exchange=self.droneAbortExchange, routing_key="drone.softabort.{}".format(drone_id))
        self.droneHardAbort = kombu.Queue("dronehardabort{}".format(drone_id), exchange=self.droneAbortExchange, routing_key="drone.hardabort.{}".format(drone_id))
        self.droneLand = kombu.Queue("droneland{}".format(drone_id), exchange=self.droneActionExchange, routing_key="drone.land.{}".format(drone_id))
        self.droneTakeoff = kombu.Queue("dronetakeoff{}".format(drone_id), exchange=self.droneActionExchange, routing_key="drone.takeoff.{}".format(drone_id))

    def get_consumers(self, Consumer, channel):
        return [
                Consumer(queues = [self.droneSoftAbort], callbacks = [self.DroneSoftAbortCallback], prefetch_count = 1),
                Consumer(queues = [self.droneHardAbort], callbacks = [self.DroneHardAbortCallback], prefetch_count = 1),
                Consumer(queues = [self.droneLand], callbacks = [self.DroneLandCallback], prefetch_count = 1),
                Consumer(queues = [self.droneTakeoff], callbacks = [self.DroneTakeoffCallback], prefetch_count = 1),
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
