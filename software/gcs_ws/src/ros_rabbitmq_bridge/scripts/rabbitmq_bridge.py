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

message = String(data = 'Hello')

json_str = json_message_converter.convert_ros_message_to_json(message)


pub = rospy.Publisher('/status', String, queue_size=1000)

def callback(data):
    print "Message received"

def listener():

    rospy.init_node('control', anonymous=True)
    print "Recieving"
    rospy.Subscriber('control_c', Empty, callback)


    while not rospy.is_shutdown():
    # do whatever you want here

        print "sending"
        pub.publish(String)
        rospy.sleep(1)  # sleep for one second


def rabbitmq(arg):
    pass



if __name__ == '__main__':
    print "Running"
    listener()
