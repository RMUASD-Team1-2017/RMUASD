import pika
import sys
import rospy
import numpy as np
import threading
import json


from Queue import Queue
from std_msgs.msg import Int32
from std_msgs.msg import Empty
from std_msgs.msg import String
from ros_rabbitmq_bridge.msg import userinfo
from rospy_message_converter import json_message_converter
from ros_rabbitmq_bridge import *
import functools

class ros_to_rabbitmq_bridge:
    def __init__(self, rabbitmq_host, bridge_settings):
        #Bridge settings is a list of dicts with the following content:
        #You can add further variables for use in the callback if you want
        #{"ros_topic" : .., "routing_key" : .., "message_type" : .., "callback" : .., "exchange" : .., }
        self.settings = bridge_settings
        self.host = rabbitmq_host
        self.setup_rabbitmq()
        self.setup_subscribe_publish()

    def setup_subscribe_publish(self):
        for setting in self.settings:
            if not "callback" in setting.keys():
                setting["callback"] = self.simple_callback
            self.channel.exchange_declare(exchange=setting["exchange"], type='topic' )
            rospy.Subscriber(setting["ros_topic"], setting["message_type"], setting["callback"], (setting, self.channel,), queue_size=10)

    def setup_rabbitmq(self):
        self.connection = pika.BlockingConnection(pika.URLParameters(self.host))
        self.channel = self.connection.channel()

    def simple_callback(self, data, args):
        #Demo callback. Simply publishes the converted json on the same topic as recieved on.
        settings = args[0]
        channel = args[1]
        json_str = json_message_converter.convert_ros_message_to_json(data)
        self.channel.basic_publish(exchange=settings["exchange"], routing_key=settings["routing_key"], body=json_str)

class rabbitmq_to_ros_bridge(ros_to_rabbitmq_bridge):
    #Mostly the same as before, but the settings are a bit different:
    #{"ros_topic" : .., "message_type_str" : .., "message_type" : .., "callback" : .., "exchange" : .., "routing_key" : ..}

    def setup_subscribe_publish(self):
        for setting in self.settings:
            if not "callback" in setting.keys():
                setting["callback"] = self.simple_callback
            self.channel.exchange_declare(exchange=setting["exchange"], type='topic' )
            setting["queue"]  = self.channel.queue_declare(exclusive=True).method.queue
            rospy.Subscriber(setting["ros_topic"], setting["message_type"], setting["callback"], (setting, ), queue_size=10)
            self.channel.queue_bind(exchange= setting["exchange"], queue= setting["queue"], routing_key = setting["routing_key"])
            self.channel.basic_consume(functools.partial(setting["callback"], settings = setting) ,queue=setting["queue"], no_ack=False)
            setting["publisher"] = rospy.Publisher(setting["ros_topic"], setting["message_type"], queue_size=10)

    def simple_callback(self, *args, **kwargs):
        if len(args) != 4:
            return
        ch = args[0]
        method = args[1]
        body = args[3]
        settings = kwargs["settings"]
        message = json_message_converter.convert_json_to_ros_message(settings["message_type_str"], body)
        settings["publisher"].publish(message)
        ch.basic_ack(delivery_tag = method.delivery_tag)
