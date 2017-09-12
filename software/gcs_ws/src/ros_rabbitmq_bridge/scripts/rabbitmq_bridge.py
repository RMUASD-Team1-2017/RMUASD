#!/usr/bin/env python
import pika
import sys
import rospy
import numpy as np
from std_msgs.msg import Int32
from std_msgs.msg import Empty
from std_msgs.msg import String
from ros_rabbitmq_bridge.msg import userinfo
from rospy_message_converter import json_message_converter


# datain is supposed to be input from rabbitmq consumer, for now, simple test case
datain = '{"state": "flying", "dronepos" : {"longitude": 10000, "latitude": 20000, "position_covariance_type" : 1}, "eta" : 100}'


newmsgin = json_message_converter.convert_json_to_ros_message('ros_rabbitmq_bridge/userinfo', datain)

# dataout is supposed to be incoming ros messages that gets converted to json and the emitted using rabbitmq
#dataout = userinfo(state = 'idle')

#newmsgout = json_message_converter.convert_ros_message_to_json(dataout)

def talker():
    pub = rospy.Publisher('ros_rabbitmq_bridge/userinfo', userinfo, queue_size=100)
    rospy.init_node('custom_talker', anonymous=True)
    r = rospy.Rate(10) #10hz
    msg = userinfo()
    msg = newmsgin

    while not rospy.is_shutdown():

        pub.publish(msg)
        r.sleep()

if __name__ == '__main__':



    talker()
