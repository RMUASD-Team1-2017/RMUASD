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



class talker:
    def __init__(self,name, status_consume, missionreq_consume):
      self.name = name
      self.status_consume = status_consume
      self.missionreq_consume = missionreq_consume


    def talk(self):
        pub = rospy.Publisher('ros_rabbitmq_bridge/userinfo', userinfo, queue_size=10)
        r = rospy.Rate(10) #10hz
        msg = userinfo()
        while not rospy.is_shutdown():

            if not self.status_consume.q.empty():
                status_message = self.status_consume.q.get()
                status_message_body = status_message["body"]
                status = json_message_converter.convert_json_to_ros_message('ros_rabbitmq_bridge/userinfo', status_message_body)
                print status
                pub.publish(status)


            if not self.missionreq_consume.t.empty():
                missionreq_message = self.missionreq_consume.t.get()
                missionreq_message_body = missionreq_message["body"]
                missionreq = json_message_converter.convert_json_to_ros_message('ros_rabbitmq_bridge/userinfo', missionreq_message_body)
                print missionreq
                pub.publish(missionreq)

            r.sleep()


    def run(self):
        print "Starting " + self.name
        self.talker_thread = threading.Thread(target=self.talk)
        self.talker_thread.daemon = True
        self.talker_thread.start()

    def __del__(self):
        print("talk_joining")
        self.talker_thread.join()
        print("talk_joined")

class listener:
    def __init__(self,name):
        self.name = name
        self.dat = Queue()

    def listen_callback(self,data):
        print(data)
        self.dat.put({"body" : data})

    def listen(self):
        rospy.Subscriber("ros_rabbitmq_bridge/userinfo", userinfo, self.listen_callback)
        rospy.spin()

    def run(self):
        print "Starting " + self.name
        self.listen_thread = threading.Thread(target=self.listen)
        self.listen_thread.daemon = True
        self.listen_thread.start()

    def __del__(self):
        print("listen_joining")
        self.listen_thread.join()
        print("listen_joined")


class status_consumer:
    def __init__(self, name):
        self.connection = pika.BlockingConnection(pika.ConnectionParameters(host='localhost'))
        self.channel = self.connection.channel()
        self.channel.exchange_declare(exchange='drone', type='topic')
        self.result = self.channel.queue_declare(exclusive=True)
        self.queue_name = self.result.method.queue
        self.channel.queue_bind(exchange='drone', queue=self.queue_name, routing_key='drone.*.status')
        self.name = name
        self.q = Queue()

    def callback_rabbit(self,ch,method,properties,body):
        ID = body.split(".")[1]
        self.q.put({"body" :body, "id" : ID})
        #print body
    def consume(self):
        self.channel.basic_consume(self.callback_rabbit,queue=self.queue_name,no_ack=True)
        self.channel.start_consuming()

    def run(self):
        print "Starting " + self.name + " thread"
        self.consuming_thread = threading.Thread(target=self.consume)
        self.consuming_thread.daemon = True
        self.consuming_thread.start()

    def __del__(self):
        print("statusconsume joining")
        self.consuming_thread.join()
        print("statusconsume joined")

class status_emitter:
    def __init__(self,name,status_consume,listen):
        self.connection = pika.BlockingConnection(pika.ConnectionParameters(host='localhost'))
        self.channel = self.connection.channel()
        self.channel.exchange_declare(exchange='drone', type='topic')
        self.name = name
        self.status_consume = status_consume
        self.listen = listen

    def emit(self):
        #message = self.status_consume.q.get()
        #message_id = message["id"]

        while 1:
            temp = self.listen.dat.get(block=True)["body"]
            json_str = json_message_converter.convert_ros_message_to_json(temp)
            self.channel.basic_publish(exchange='drone', routing_key='drone.40.status', body=json_str)

    def run(self):
        print "Starting " + self.name + " thread"
        self.emitting_thread = threading.Thread(target=self.emit)
        self.emitting_thread.daemon = True
        self.emitting_thread.start()

    def __del__(self):
        print("status emitter joining")
        self.emitting_thread.join()
        self.connection.close()
        print("status emitter joined")


class missionreq_consumer:
    def __init__(self, name):
        self.connection = pika.BlockingConnection(pika.ConnectionParameters(host='localhost'))
        self.channel = self.connection.channel()
        self.channel.exchange_declare(exchange='drone', type='topic')
        self.result = self.channel.queue_declare(exclusive=True)
        self.queue_name = self.result.method.queue
        self.channel.queue_bind(exchange='drone', queue=self.queue_name, routing_key='drone.*.mission_request')
        self.name = name
        self.t = Queue()

    def callback_rabbit(self,ch,method,properties,body):
        ID = body.split(".")[1]
        self.t.put({"body" :body, "id" : ID})
        print body
    def consume(self):
        self.channel.basic_consume(self.callback_rabbit,queue=self.queue_name,no_ack=True)
        self.channel.start_consuming()

    def run(self):
        print "Starting " + self.name + " thread"
        self.consuming_thread = threading.Thread(target=self.consume)
        self.consuming_thread.daemon = True
        self.consuming_thread.start()

    def __del__(self):
        print("mission consumer joining")
        self.consuming_thread.join()
        print("mission consumer joined")


class missionreq_emitter:
    def __init__(self,name,missionreq_consume,listen):
        self.connection = pika.BlockingConnection(pika.ConnectionParameters(host='localhost'))
        self.channel = self.connection.channel()
        self.channel.exchange_declare(exchange='drone', type='topic')
        self.name = name
        self.missionreq_consume = missionreq_consume
        self.listen = listen

    def emit(self):
        #message = self.missionreq_consume.t.get()
        #message_id = message["id"]
        while 1:
            temp = self.listen.dat.get(block=True)["body"]
            json_str = json_message_converter.convert_ros_message_to_json(temp)
            self.channel.basic_publish(exchange='drone', routing_key='drone.40.mission_request', body=json_str)


    def run(self):
        print "Starting " + self.name + " thread"
        self.emitting_thread = threading.Thread(target=self.emit)
        self.emitting_thread.daemon = True
        self.emitting_thread.start()

    def __del__(self):
        print("mission emitter joining")
        self.emitting_thread.join()
        self.connection.close()
        print("mission emitter joined")
