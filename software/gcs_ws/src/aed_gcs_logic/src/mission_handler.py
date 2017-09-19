#!/usr/bin/env python
import rospy

from std_msgs.msg import String
from aed_gcs_logic.msg import *
from Drone import Drone, Mission
from ros_rabbitmq_bridge.msg import mission_request



class mission_handler:
    """Path Planner class"""

    def __init__(self):
        self.risk_metric_sub = rospy.Subscriber("/risk_assessment/risk_metric", String, self.risk_metric_callback)
        self.drone = Drone(drone_id = int(rospy.get_param('/drone_id', 1)) )
        self.current_risk_metric = 0
        self.mission_request_sub = rospy.Subscriber("drone/missionrequest", mission_request, self.mission_request_callback, queue_size=10)

    def risk_metric_callback(self, data):
        print "Recieved risk metric"

        self.current_risk_metric = data.data

    def mission_request_callback(self, data):
        mission = Mission(data.mission_id, data.destination)
        if not self.drone.mission:
            if self.drone.set_mission(mission):
                mission.plan(self.drone.location["location"])
                print("Mission was set!")
            else:
                print("Drone rejected mission")
        else:
            print("Drone already have a mission")




if __name__ == "__main__":

    print "Starting Mission Handler Node"

    rospy.init_node('mission_handler_node', anonymous=True)

    m_handler = mission_handler()
    print("Startup of mission handler complete")
    rospy.spin()
