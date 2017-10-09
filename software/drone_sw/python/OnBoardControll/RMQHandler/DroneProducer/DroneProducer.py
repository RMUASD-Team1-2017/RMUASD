import json
import kombu
import datetime


class DroneProducer:
    def __init__(self, _id, connection):
        self._id = _id
        self.dronesensor = kombu.Exchange(name="dronesensor", type="topic", durable = False)
        self.producer = connection.Producer()

    def publish_onboard_gps(self, info, location):
        #Publish the info about the delution, sattelites etc, as well as the location
        time_str = datetime.datetime.now().strftime("%Y/%m/%d_%H:%M:%S")
        data = {"time" : time_str}
        data["info"] = {"eph" : info.eph, "epv" : info.epv, "fix_type" : info.fix_type, "satellites_visible" : info.satellites_visible}
        data["location"] = {"lat" : location.lat, "lon" : location.lon, "alt" : location.alt}
        body = json.dumps(data)
        self.producer.publish(body, routing_key = "drone.onboard_gps.{}".format(self._id), exchange = self.dronesensor,
                declare = [self.dronesensor], retry = True)

    def publish_external_gps(self, info, location):
        #Publish the info about the delution, sattelites etc, as well as the location
        time_str = datetime.datetime.now().strftime("%Y/%m/%d_%H:%M:%S")
        data = {"time" : time_str}
        data["info"] = {"eph" : info.eph, "epv" : info.epv, "fix_type" : info.fix_type, "satellites_visible" : info.satellites_visible}
        data["location"] = {"lat" : location.lat, "lon" : location.lon, "alt" : location.alt}
        body = json.dumps(data)
        self.producer.publish(body, routing_key = "drone.external_gps.{}".format(self._id), exchange = self.dronesensor,
                declare = [self.dronesensor], retry = True)
