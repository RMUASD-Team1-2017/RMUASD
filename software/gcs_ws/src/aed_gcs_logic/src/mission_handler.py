#!/usr/bin/env python
import rospy

from std_msgs.msg import String
from aed_gcs_logic.msg import *
from Drone import Drone, Mission
from ros_rabbitmq_bridge.msg import mission_request
from geometry_msgs.msg import TwistStamped
from sensor_msgs.msg import NavSatFix
import math

from  geopy.distance import vincenty    # pip install geopy

#gps_velocity = 1

class mission_handler:
    """Path Planner class"""

    def __init__(self):
        #self.risk_metric_sub = rospy.Subscriber("/risk_assessment/risk_metric", String, self.risk_metric_callback)
        self.drone = Drone(drone_id = int(rospy.get_param('/drone_id', 1)) )
        self.current_risk_metric = 0
        self.mission_request_sub = rospy.Subscriber("drone/missionrequest", mission_request, self.mission_request_callback, queue_size=10)
        self.velocity_gps = rospy.Subscriber("/mavros/local_position/velocity", TwistStamped, self.twist_request_callback, queue_size=10)
        self.position_sub = rospy.Subscriber("mavros/global_position/global", NavSatFix, self.position_callback, queue_size=10)

        print " hej "
        self.gps_velocity = 0
        self.velocityx = 0
        self.velocityy = 0
        self.velocityz = 0
        self.latitude = 0
        self.longitude = 0
        self.altitude = 0
        self.distance = 0

    def risk_metric_callback(self, data):
        print "Recieved risk metric"

        self.current_risk_metric = data.data

    def mission_request_callback(self, data):
        mission = Mission(data.mission_id, data.destination)
        print " mission request callback "
        if not self.drone.mission:
            #print("Drone not in mission")
            if not self.drone.rpcIsDroneReady():
                print("Drone OES is not ready for mission")
                return
            #bat_and_gps_status = self.drone.RiskAssesment()
            #print bat_and_gps_status
            print " bat and gps status"
        #    print("Drone is not in mission")
        #    if  not bat_and_gps_status:
        #        print("Drone Battery and GPS is not working, or the weather is bad")
        #        return

            print("Drone Battery and GPS is working, and the weather is god")
            print  self.drone.set_mission(mission)
            if self.drone.set_mission(mission):
                #mission.plan(self.drone.location["location"])
                print("Mission was set!")

            else:
                print("Drone rejected mission")
        else:
            print("Drone already have a mission")

    def twist_request_callback(self, data):
        self.velocityx = data.twist.linear.x
        self.velocityy = data.twist.linear.y
        self.velocityz = data.twist.linear.z


    def position_callback(self,data):
        self.latitude = data.latitude
        self.longitude = data.longitude
        self.altitude = data.altitude

    #    velocity_gps = rospy.Subscriber("global_position/raw/gps_vel", Twist, self.Twist_request_callback, queue_size=1)

    def calc_velocity(self):
        self.velocity = math.sqrt((self.velocityx * self.velocityx) + (self.velocityy * self.velocityy) + (self.velocityz * self.velocityz))


        return self.velocity
        #return self.velocity


    def calc_distance(self):
        end_position = (55.564339, 10.121369)
        current_position = ( self.latitude, self.longitude)
        self.distance = vincenty(current_position, end_position).meters
        return self.distance

    def calc_time_to_end_pos(self):
        time = 0
        time_min = 0
        if self.velocity > 0:
            time = self.distance/self.velocity
            time_min = time/60

        return time_min


if __name__ == "__main__":

    print "Starting Mission Handler Node"
    rospy.init_node('mission_handler_node', anonymous=True)

    m_handler = mission_handler()
    rospy.spin()
