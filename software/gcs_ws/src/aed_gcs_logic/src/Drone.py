#!/usr/bin/env python
#This class contains a simple representation of a drone, with status, id, etc.
import datetime
from std_msgs.msg import String
from sensor_msgs.msg import NavSatFix
import rospy
import threading
from ros_rabbitmq_bridge.msg import userinfo
from aed_gcs_logic.srv import mission_request, OnboardStatus, OnboardStatusRequest,HealthCheckService, HealthCheckServiceRequest
# HealthCheckServiceRequest is declered by ros.
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
        self.goal = self.plan_path(start_pos, self.destination).goal


class Drone:
    def __init__(self, drone_id):
        self.location = {"location" : None, "update_time" : None, "update_time" : None}
        self.drone_id = drone_id
        self.state = "Idle"
        self.mission = None
        print " wait "
        rospy.wait_for_service('drone/get_readyness')
        self.drone_ready_service = rospy.ServiceProxy("drone/get_readyness", OnboardStatus)


        #self.health_check_service = rospy.ServiceProxy("drone/Health_check_service", HealthCheckService)
        #rospy.wait_for_service('drone/Health_check_service')
        print " drone.py "

        self.publish_sem = threading.Semaphore(0)
        self.lock = threading.RLock()
        #self.risk_metric_sub = rospy.Subscriber("/risk_assessment/risk_metric", String, self.risk_metric_callback)
        self.position_sub = rospy.Subscriber("mavros/global_position/global", NavSatFix, self.position_callback, queue_size=10)
        self.status_publish = rospy.Publisher("drone/status", userinfo, queue_size=10)
        self.last_pos_update = datetime.datetime.min

        self.status_publisher_thread = threading.Thread(target = self.status_publisher)
        self.status_publisher_thread.daemon = True
        self.status_publisher_thread.start()


    def set_mission(self, mission):
        with self.lock:
            self.mission = mission
            self.state = "On Mission"
            self.publish_sem.release()
            return True


    def risk_metric_callback(self, data):
        print "Recieved risk metric"

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
                self.status_publish.publish(info)

    def position_callback(self, data):
        now = rospy.get_rostime()
        diff = now - data.header.stamp
        with self.lock:
            self.location["location"] = data
            time = datetime.datetime.now() + datetime.timedelta(seconds = diff.to_sec())
            self.location["update_time"] = time.strftime("%Y/%m/%d_%H:%M:%S")
            if self.last_pos_update < datetime.datetime.now() - POSITION_INTERVAL:
                self.last_pos_update = datetime.datetime.now()
                self.publish_sem.release()

    def rpcIsDroneReady(self):
        if rospy.get_param('/ignore_onboard', False) is True:# or True:   # remember to remove or True
            return True
        try:
            request = OnboardStatusRequest()
            response = self.drone_ready_service(request)
            return json.loads(response.response.data)["ready"]
        except Exception as e:
            try:
                print("Drone not ready, response was {}".format(response))
            except:
                traceback.print_exc()
    def BatteryAndGPStatus(self):
        print "Battery and GPS status "

        if rospy.get_param('/ignore_weather_and_GPS', False) is True:# or True:
            return True

        try:
            request1 = HealthCheckServiceRequest()
            response1 = self.health_check_service(request1)
            print response1.flight
            #add_two_ints = rospy.ServiceProxy('add_two_ints', AddTwoInts)
            return response1.flight
        except rospy.ServiceException as e:
            print "Service call failed :"
            traceback.print_exc()


#    def RiskAssesment(self):
#        print "Risk Assesment"
#        print self.risk_metric_sub
#        try:
#            return self.risk_metric_sub
#        except rospy.ServiceException as e:
#            print "Service call failed :"
#            traceback.print_exc()
