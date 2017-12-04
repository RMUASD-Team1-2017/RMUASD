#!/usr/bin/env python
#This class contains a simple representation of a drone, with status, id, etc.
import datetime

from std_msgs.msg import String
from std_msgs.msg import Float32

from sensor_msgs.msg import NavSatFix
import rospy
import threading
from ros_rabbitmq_bridge.msg import userinfo
from aed_gcs_logic.srv import mission_request, OnboardStatus, OnboardStatusRequest
POSITION_INTERVAL = datetime.timedelta(milliseconds = 500) #We only update the position if it is this time since we did it last
import traceback
import json
class Mission:
    def __init__(self, mission_id, destination):
        self.mission_id = mission_id
        self.destination = destination
        self.plan_path = rospy.ServiceProxy('plan_path', mission_request)
        self.goal = None

    def plan(self, start_pos):
        self.waypoints = self.plan_path(start_pos, self.destination).path
        if len(self.waypoints):
            self.goal = self.waypoints[-1]
        else:
            print("The pathplanner returned an empty path! It may be retarded...")


class Drone:
    def __init__(self, drone_id):
        self.location = {"location" : None, "update_time" : None, "update_time" : None}
        self.drone_id = drone_id
        self.state = "Idle"
        self.mission = None
        self.position_sub = rospy.Subscriber("mavros/global_position/global", NavSatFix, self.position_callback, queue_size=10)
        self.status_publish = rospy.Publisher("drone/status", userinfo, queue_size=10)
        self.drone_ready_service = rospy.ServiceProxy("drone/get_readyness", OnboardStatus)

        self.risk_metric_sub = rospy.Subscriber("/risk_assessment/risk_metric", Float32, self.risk_metric_callback)

        self.publish_sem = threading.Semaphore(0)
        self.lock = threading.RLock()

        self.status_publisher_thread = threading.Thread(target = self.status_publisher)
        self.status_publisher_thread.daemon = True
        self.status_publisher_thread.start()
        self.last_pos_update = datetime.datetime.min

    def set_mission(self, mission):
        with self.lock:
            self.mission = mission
            self.state = "On Mission"
            self.publish_sem.release()
            return True


    def risk_metric_callback(self, data):
        #print "Recieved risk metric"

        self.current_risk_metric = data.data

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
                if self.mission and self.mission.goal and self.location["update_time"]:
                    info.mission_id = self.mission.mission_id
                    info.destination = self.mission.goal
                    info.path = self.mission.waypoints
                self.status_publish.publish(info)

    def position_callback(self, data):
        now = rospy.get_rostime()
        diff = now - data.header.stamp
        with self.lock:
            self.location["location"] = data
            time = datetime.datetime.utcnow() + datetime.timedelta(seconds = diff.to_sec())
            self.location["update_time"] = time.strftime("%Y/%m/%d_%H:%M:%S")
            if self.last_pos_update < datetime.datetime.utcnow() - POSITION_INTERVAL:
                self.last_pos_update = datetime.datetime.utcnow()
                self.publish_sem.release()

    def rpcIsDroneReady(self):


        if rospy.get_param('/ignore_onboard', False) is True:   # remember to remove "or True" both in risk assesment node, ans link monitoring node.
            return True
        try:
            request = OnboardStatusRequest()
            response = self.drone_ready_service(request)
            print("Drone response: {}".format(response))
            return json.loads(response.response.data)["ready"]
        except Exception as e:
            try:
                print("Drone not ready, response was {}".format(response))
            except:
                traceback.print_exc()

    def RiskAssesment(self):
        print "Risk Assesment"
        #print self.risk_metric_sub
        try:
            return self.risk_metric_sub
        except rospy.ServiceException as e:
            print "Service call failed :"
            traceback.print_exc()
