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
#datain = '{"current_time": {"data": {"secs" : 22, "nsecs": 2200}}, "state": "flying", "destination" : {"longitude": 10000, "latitude": 20000, "position_covariance_type" : 1}, "eta" : 100}'
#newmsgin = json_message_converter.convert_json_to_ros_message('ros_rabbitmq_bridge/userinfo', datain)


if __name__ == '__main__':



    rospy.init_node('rabbitmq_bridge', anonymous=True)

    status = listener( "ROS: status listener", "status")

    missionrequest = listener ("ROS: missionreq listener", "missionrequest")

    status_consume = status_consumer( "rabbitMQ: status_consumer" )
    status_emit = status_emitter( "rabbitMQ: status_emitter", status_consume, status)

    missionreq_consume = missionreq_consumer( "rabbitMQ: missionreq_consumer" )
    missionreq_emit = missionreq_emitter( "rabbitMQ: missionreq_emitter", missionreq_consume, listen)

    talk = talker( "ROS: status_mission_req_talker", status_consume, missionreq_consume)




    status_consume.run()
    missionreq_consume.run()

    status_emit.run()
    missionreq_emit.run()

    listen.run()
    talk.run()

    status_consume.consuming_thread.join()
    status_emit.emitting_thread.join()
    missionreq_consume.consuming_thread.join()
    missionreq_emit.emitting_thread.join()

    listen.listen_thread.join()
    talk.talker_thread.join()



    print "Exiting Main thread"
