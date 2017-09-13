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
    def __init__(self,name, status_consume):
      self.name = name
      self.status_consume = status_consume

    def run(self):
      print "Starting " + self.name
      pub = rospy.Publisher('ros_rabbitmq_bridge/userinfo', userinfo, queue_size=100)
      rospy.init_node('talker', anonymous=True)
      r = rospy.Rate(10) #10hz
      msg = userinfo()
      if not self.status_consume.q.empty():
        message = self.status_consume.q.get()
        message_body = message["body"]
        msg = json_message_converter.convert_json_to_ros_message('ros_rabbitmq_bridge/userinfo', message_body)
      while not rospy.is_shutdown():
          pub.publish(msg)
          r.sleep()

      print "Exiting " + self.name
class listener:
    def __init__(self,name):
        self.name = name
        self.dat = Queue()

    def callback(data):
        dat.put({"info" : data})

    def run(self):
        print "Starting " + self.name
        rospy.init_node('listener', anonymous=True)
        rospy.Subscriber("ros_rabbitmq_bridge/userinfo", userinfo, callback)
        # spin() simply keeps python from exiting until this node is stopped
        rospy.spin()


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
        q.put({"body" :body, "id" : ID})
        print body
    def consume(self):
        self.channel.basic_consume(self.callback_rabbit,queue=self.queue_name,no_ack=True)
        self.channel.start_consuming()

    def run(self):
        print "Starting " + self.name + " thread"
        self.consuming_thread = threading.Thread(target=self.consume)
        self.consuming_thread.daemon = True
        self.consuming_thread.start()

class status_emitter:
    def __init__(self,name,status_consume,listen):
        self.connection = pika.BlockingConnection(pika.ConnectionParameters(host='localhost'))
        self.channel = self.connection.channel()
        self.channel.exchange_declare(exchange='drone', type='topic')
        self.name = name
        self.status_consume = status_consume
        self.listion = listen

    def emit(self):
        message = self.status_consume.q.get()
        message_id = message["id"]
        temp = self.listen.dat.get()
        data = temp["info"]
        self.channel.basic_publish(exchange='topic_logs', routing_key='drone.' + str(message_id) + '.status', body=data)

        self.connection.close()

    def run(self):
        print "Starting " + self.name + " thread"
        self.consuming_thread = threading.Thread(target=self.emit)
        self.consuming_thread.daemon = True
        self.consuming_thread.start()



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
        t.put({"body" :body, "id" : ID})
        print body
    def consume(self):
        self.channel.basic_consume(self.callback_rabbit,queue=self.queue_name,no_ack=True)
        self.channel.start_consuming()

    def run(self):
        print "Starting " + self.name + " thread"
        self.consuming_thread = threading.Thread(target=self.consume)
        self.consuming_thread.daemon = True
        self.consuming_thread.start()

class missionreq_emitter:
    def __init__(self,name,missionreq_consume,listen):
        self.connection = pika.BlockingConnection(pika.ConnectionParameters(host='localhost'))
        self.channel = self.connection.channel()
        self.channel.exchange_declare(exchange='drone', type='topic')
        self.name = name
        self.missionreq_consume = missionreq_consume
        self.listen = listen

    def emit(self):
        message = self.missionreq_consume.t.get()
        message_id = message["id"]
        temp = self.listen.dat.get()
        data = temp["info"]
        self.channel.basic_publish(exchange='topic_logs', routing_key='drone.' + str(message_id) + '.mission_request', body=data)

        self.connection.close()

    def run(self):
        print "Starting " + self.name + " thread"
        self.consuming_thread = threading.Thread(target=self.emit)
        self.consuming_thread.daemon = True
        self.consuming_thread.start()
