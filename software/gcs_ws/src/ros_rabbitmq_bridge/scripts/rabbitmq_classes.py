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
import datetime
import genpy
import uuid
import threading
import traceback


MAX_HEARTBEAT_AGE = datetime.timedelta(seconds = 5)

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
            if "ros_topic" in setting.keys():
                self.channel.exchange_declare(exchange=setting["exchange"], type='topic', durable = False)
                rospy.Subscriber(setting["ros_topic"], setting["message_type"], setting["callback"], (setting, self.channel,), queue_size=10)
            elif "ros_service" in setting.keys():
                setting["callback_queue"] = self.channel.queue_declare(exclusive = True).method.queue
                self.channel.basic_consume(setting["response_callback"], queue=setting["callback_queue"])
                rospy.Service(setting["ros_service"], setting["service_type"], functools.partial(setting["callback"], args = (setting, self.channel, self.connection)) )
            else:
                print("Not a valid subscription entry: {}".format(setting))


    def setup_rabbitmq(self):
        self.connection = pika.BlockingConnection(pika.URLParameters(self.host))
        self.channel = self.connection.channel()

    @classmethod
    def simple_callback(self, data, args):
        #Demo callback. Simply publishes the converted json on the same topic as recieved on.
        settings = args[0]
        channel = args[1]
        try:
            json_str = json_message_converter.convert_ros_message_to_json(data)
            channel.basic_publish(exchange=settings["exchange"], routing_key=settings["routing_key"], body=json_str)
        except:
            traceback.print_exc()

    @classmethod
    def time_callback(cls, data, args):
        #Simple sends a json message containing the current time
        settings = args[0]
        channel = args[1]
        time_str = datetime.datetime.utcnow().strftime("%Y/%m/%d_%H:%M:%S")
        data = {"time" : time_str}
        channel.basic_publish(exchange=settings["exchange"], routing_key=settings["routing_key"], body=json.dumps(data))

class ServiceHandler:
    def __init__(self):
        self.lock = threading.RLock()
        self.responses = {}
    def ros_callback(self, req, args):
        #Example callback for services
        settings = args[0]
        channel = args[1]
        connection = args[2]
        try:
            json_str = json_message_converter.convert_ros_message_to_json(req.request)
            corr_id = str(uuid.uuid4())
            with self.lock:
                self.responses[corr_id] = None
            channel.basic_publish(exchange=settings["exchange"], \
                                  routing_key=settings["routing_key"],  \
                                  body=json_str, \
                                  properties=pika.BasicProperties(
                                    reply_to = settings["callback_queue"],
                                    correlation_id = corr_id,
                                    ),
            )
            start_wait = datetime.datetime.utcnow()
            while datetime.datetime.utcnow() < start_wait + datetime.timedelta(seconds=10):
                with self.lock:
                    response = self.responses[corr_id]
                if response is not None:
                    del self.responses[corr_id]
                    resp =  settings["service_type"]._response_class()
                    resp.response.data = response
                    return resp
                connection.process_data_events()
        except:
            traceback.print_exc()
        resp =  settings["service_type"]._response_class()
        resp.response.data = ""
        return resp

    def service_response_callback(self, ch, method, props, body):
        with self.lock:
            try:
                if props.correlation_id in self.responses.keys():
                    self.responses[props.correlation_id] = body
            except:
                traceback.print_exc()
        ch.basic_ack(delivery_tag = method.delivery_tag)


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

    @classmethod
    def heartbeat_callback(self, *args, **kwargs):
        if len(args) != 4:
            return
        ch = args[0]
        method = args[1]
        body = args[3]
        data = json.loads(body)
        time = datetime.datetime.strptime(data["time"], "%Y/%m/%d_%H:%M:%S")
        heartbeat_age = abs(datetime.datetime.utcnow() - time)
        if heartbeat_age > MAX_HEARTBEAT_AGE:
            print("Heartbeat from OES was too old. Age was: {}".format(heartbeat_age))
        else:
            settings = kwargs["settings"]
            message = json_message_converter.convert_json_to_ros_message(settings["message_type_str"], "{}")
            settings["publisher"].publish(message)
        ch.basic_ack(delivery_tag = method.delivery_tag)


    @classmethod
    def navsatfix_callback(self, *args, **kwargs):
        if len(args) != 4:
            return
        ch = args[0]
        method = args[1]
        body = args[3]
        settings = kwargs["settings"]
        data = json.loads(body)
        message = settings["message_type"]()
        message.status.status = message.status.STATUS_NO_FIX if data["info"]["fix_type"] <= 1 else message.status.STATUS_FIX
        message.status.service = message.status.SERVICE_GPS

        message.latitude = data["location"]["lat"]
        message.longitude = data["location"]["lon"]
        message.altitude = data["location"]["alt"]
        if not None in data["location"].values():
            settings["publisher"].publish(message)
        ch.basic_ack(delivery_tag = method.delivery_tag)
