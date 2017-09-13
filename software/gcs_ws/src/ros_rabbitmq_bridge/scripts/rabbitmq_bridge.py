#!/usr/bin/env python
import pika
import sys
import rospy
import numpy as np
import threading
import json
from rabbitmq_classes import status_consumer, status_emitter, missionreq_consumer, missionreq_emitter, listener, talker

from Queue import Queue
from std_msgs.msg import Int32
from std_msgs.msg import Empty
from std_msgs.msg import String
from ros_rabbitmq_bridge.msg import userinfo
from rospy_message_converter import json_message_converter



# test case for json to ros conversion
#datain = '{"current_time": {"data": {"secs" : 22, "nsecs": 2200}}, "state": "flying", "dronepos" : {"longitude": 10000, "latitude": 20000, "position_covariance_type" : 1}, "eta" : 100}'
#newmsgin = json_message_converter.convert_json_to_ros_message('ros_rabbitmq_bridge/userinfo', datain)


if __name__ == '__main__':

    listen = listener( "listener" )

    status_consume = status_consumer( "status_consumer" )
    status_emit = status_emitter( "status_emitter", status_consume, listen)

    missionreq_consume = missionreq_consumer( "missionreq_consumer" )
    missionreq_emit = missionreq_emitter( "missionreq_emitter", missionreq_consume, listen)

    talk = talker( "talker", status_consume)


    status_consume.run()
    status_emit.run()
    missionreq_consume.run()
    missionreq_emit.run()
    talk.run()

    print "Exiting Main thread"
