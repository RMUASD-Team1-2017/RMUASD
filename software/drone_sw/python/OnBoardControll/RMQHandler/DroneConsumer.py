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
        self.droneSoftAbort = kombu.Queue("dronesoftabort{}".format(drone_id), exchange=self.droneAbortExchange, routing_key="drone.softabort.{}".format(drone_id))
        self.droneHardAbort = kombu.Queue("dronehardabort{}".format(drone_id), exchange=self.droneAbortExchange, routing_key="drone.hardabort.{}".format(drone_id))
    def get_consumers(self, Consumer, channel):
        return [
                Consumer(queues = [self.droneSoftAbort], callbacks = [self.DroneSoftAbortCallback], prefetch_count = 1),
                Consumer(queues = [self.droneHardAbort], callbacks = [self.DroneHardAbortCallback], prefetch_count = 1),
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
        message.ack()

    def DroneSoftAbortCallback(self, body, message):
        drone_id, data, time = self.extract_common_data(body, message)
        logging.debug("Performing Soft Abort!")
        message.ack()
