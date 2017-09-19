#!/usr/bin/env python
import pika
import sys
import rospy
import numpy as np
import threading
import json
from rabbitmq_classes import ros_to_rabbitmq_bridge, rabbitmq_to_ros_bridge

from Queue import Queue
from std_msgs.msg import Int32
from std_msgs.msg import Empty
from std_msgs.msg import String
from ros_rabbitmq_bridge.msg import userinfo, mission_request
from rospy_message_converter import json_message_converter
import threading


# test case for json to ros conversion
#datain = '{"current_time": {"data": {"secs" : 22, "nsecs": 2200}}, "state": "flying", "destination" : {"longitude": 10000, "latitude": 20000, "position_covariance_type" : 1}, "eta" : 100}'
#newmsgin = json_message_converter.convert_json_to_ros_message('ros_rabbitmq_bridge/userinfo', datain)


if __name__ == '__main__':

    RABBIT_BROKER = "amqp://guest:guest@localhost:5672/%2F?connection_attempts=3&heartbeat_interval=3600"


    rospy.init_node('rabbitmq_bridge', anonymous=True)


    ros_to_rabbit_settings = [
    {"ros_topic" : "drone/status", "message_type" : userinfo, "routing_key" : "drone.status", "exchange" : "drone"}
    ]
    rabbit_to_ros_settings = [
    {"ros_topic" : "drone/missionrequest", "message_type_str" : "ros_rabbitmq_bridge/mission_request", "message_type" : mission_request, "routing_key" : "drone.mission_request", "exchange" : "drone"}

    ]
    print("Starting rabbitmq bridge")
    ros_to_rabbit = ros_to_rabbitmq_bridge(RABBIT_BROKER, ros_to_rabbit_settings)
    rabbit_to_ros = rabbitmq_to_ros_bridge(RABBIT_BROKER, rabbit_to_ros_settings)
    rabbit_to_ros_thread = threading.Thread(target = rabbit_to_ros.channel.start_consuming)
    rabbit_to_ros_thread.daemon = True
    rabbit_to_ros_thread.start()
    rospy.spin()


    print "Exiting Main thread"
    sys.exit()
