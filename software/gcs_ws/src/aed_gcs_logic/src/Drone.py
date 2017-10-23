#!/usr/bin/env python
#This class contains a simple representation of a drone, with status, id, etc.
import datetime
from sensor_msgs.msg import NavSatFix
import rospy
import threading
from ros_rabbitmq_bridge.msg import userinfo
from aed_gcs_logic.srv import mission_request

class Mission:
    def __init__(self, mission_id, destination):
        self.mission_id = mission_id
        self.destination = destination
        self.plan_path = rospy.ServiceProxy('plan_path', mission_request)
        self.goal = None

    def plan(self, start_pos):
        self.goal = self.plan_path(start_pos, self.destination)
        print("Response was", response)

class Drone:
    def __init__(self, drone_id):
        self.location = {"location" : None, "update_time" : None}
        self.drone_id = drone_id
        self.state = "Idle"
        self.mission = None
        self.position_sub = rospy.Subscriber("mavros/global_position/global", NavSatFix, self.position_callback, queue_size=10)
        self.status_publish = rospy.Publisher("drone/status", userinfo, queue_size=10)
        self.publish_sem = threading.Semaphore(0)
        self.lock = threading.RLock()
        self.status_publisher_thread = threading.Thread(target = self.status_publisher)
        self.status_publisher_thread.daemon = True
        self.status_publisher_thread.start()

    def set_mission(self, mission):
        with self.lock:
            self.mission = mission
            self.state = "On Mission"
            self.publish_sem.release()
            return True

    def unset_mission(self, mission):
        with self.lock:
            self.mission = None
            self.state = "Idle"
            self.publish_sem.release()

    def status_publisher(self):
        while True:
            self.publish_sem.acquire()
            with self.lock:
                info = userinfo()
                info.position = self.location["location"]
                info.state = self.state
                info.serial = self.drone_id
                info.current_time = self.location["update_time"]
                if self.mission:
                    info.mission_id = self.mission.mission_id
                    info.destination = self.mission.destination
                self.status_publish.publish(info)


    def position_callback(self, data):
        now = rospy.get_rostime()
        diff = now - data.header.stamp
        with self.lock:
            self.location["location"] = data
            time = datetime.datetime.now() + datetime.timedelta(seconds = diff.to_sec())
            self.location["update_time"] = time.strftime("%Y/%m/%d_%H:%M:%S")
            self.publish_sem.release()

   
