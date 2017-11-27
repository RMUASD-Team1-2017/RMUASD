#!/usr/bin/env python
import pika
import sys
import rospy
import numpy as np
import threading
import json
from rabbitmq_classes import ros_to_rabbitmq_bridge, rabbitmq_to_ros_bridge, ServiceHandler

from Queue import Queue
from std_msgs.msg import Int32
from std_msgs.msg import Empty
from std_msgs.msg import String
from sensor_msgs.msg import NavSatFix
from ros_rabbitmq_bridge.msg import userinfo, mission_request
from rospy_message_converter import json_message_converter
import threading
import os
from aed_gcs_logic.srv import OnboardStatus

# test case for json to ros conversion
#datain = '{"current_time": {"data": {"secs" : 22, "nsecs": 2200}}, "state": "flying", "destination" : {"longitude": 10000, "latitude": 20000, "position_covariance_type" : 1}, "eta" : 100}'
#newmsgin = json_message_converter.convert_json_to_ros_message('ros_rabbitmq_bridge/userinfo', datain)
DRONE_ID = 1


def heartbeat_thread(publisher, timeout):
    publisher.publish(Empty())
    timer = threading.Timer(timeout, heartbeat_thread, args = (publisher, timeout))
    timer.daemon = True
    timer.start()


if __name__ == '__main__':
    user = os.environ.get("rmq_user", "drone")
    password = os.environ.get("rmq_pass", "drone")
    host = os.environ.get("rmq_host", "drone.stefanrvo.dk")
    port = os.environ.get("rmq_port", "5672")
    RABBIT_BROKER = "amqp://{}:{}@{}:{}/%2F?connection_attempts=3&heartbeat_interval=3600".format(user, password, host, port)
    print("Connecting to {}".format(RABBIT_BROKER))

    rospy.init_node('rabbitmq_bridge', anonymous=True)
    heartbeat_pulbisher = rospy.Publisher("gcs/heartbeat", Empty, queue_size=10)
    heartbeat_thread(heartbeat_pulbisher, 1.0)
    readyness_servicehandler = ServiceHandler()

    ros_to_rabbit_settings = [
    {"ros_topic" : "drone/status", "message_type" : userinfo, "routing_key" : "drone.status", "exchange" : "drone"}, \
    {"ros_topic" : "drone/softabort", "message_type" : Empty, "routing_key" : "drone.softabort.{}".format(DRONE_ID), "exchange" : "droneabort", "callback" : ros_to_rabbitmq_bridge.time_callback}, \
    {"ros_topic" : "drone/land", "message_type" : Empty, "routing_key" : "drone.land.{}".format(DRONE_ID), "exchange" : "droneabort", "callback" : ros_to_rabbitmq_bridge.time_callback}, \
    {"ros_topic" : "drone/hardabort", "message_type" : Empty, "routing_key" : "drone.hardabort.{}".format(DRONE_ID), "exchange" : "droneabort", "callback" : ros_to_rabbitmq_bridge.time_callback}, \
    {"ros_topic" : "gcs/heartbeat", "message_type" : Empty, "routing_key" : "gcs.heartbeat.{}".format(DRONE_ID), "exchange" : "drone", "callback" : ros_to_rabbitmq_bridge.time_callback}, \
    {"ros_topic" : "drone/setgeofence", "message_type" : String, "routing_key" : "drone.geofence.{}".format(DRONE_ID), "exchange" : "drone"}, \
    {"ros_service" : "drone/get_readyness" , "routing_key" : "drone.ready_rpc.{}".format(DRONE_ID), "exchange" : "drone", "callback" : readyness_servicehandler.ros_callback, "response_callback" : readyness_servicehandler.service_response_callback, "service_type" : OnboardStatus}, \
    ]

    rabbit_to_ros_settings = [
    {"ros_topic" : "drone/missionrequest", "message_type_str" : "ros_rabbitmq_bridge/mission_request", "message_type" : mission_request, "routing_key" : "drone.mission_request", "exchange" : "drone"},
    {"ros_topic" : "drone/requestgeofence", "message_type_str" : "std_msgs/Empty", "message_type" : Empty, "routing_key" : "drone.geofencerequest.{}".format(DRONE_ID), "exchange" : "drone"},
    {"ros_topic" : "drone/heartbeat", "message_type_str" : "std_msgs/Empty", "message_type" : Empty, "routing_key" : "drone.heartbeat.{}".format(DRONE_ID), "exchange" : "dronesensor", "callback" : rabbitmq_to_ros_bridge.heartbeat_callback},
    {"ros_topic" : "drone/drone_external_gps", "message_type_str" : "sensor_msgs/NavSatFix", "message_type" : NavSatFix, "routing_key" : "drone.external_gps.{}".format(DRONE_ID), "exchange" : "dronesensor", "callback" : rabbitmq_to_ros_bridge.navsatfix_callback},
    {"ros_topic" : "drone/drone_onboard_gps", "message_type_str" : "sensor_msgs/NavSatFix", "message_type" : NavSatFix, "routing_key" : "drone.onboard_gps.{}".format(DRONE_ID), "exchange" : "dronesensor", "callback" : rabbitmq_to_ros_bridge.navsatfix_callback},




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
