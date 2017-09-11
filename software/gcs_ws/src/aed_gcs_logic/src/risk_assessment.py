#!/usr/bin/env python
import rospy

from std_msgs.msg import String 
from std_msgs.msg import Int8
from std_msgs.msg import Float32

class risk_analyzer:
    """Risk Analyzer class"""

    def __init__(self):
        #self.risk_metric_pub = rospy.Publisher('/risk_assessment/risk_metric', Int8, queue_size=1)		# using int metric
        self.risk_metric_pub = rospy.Publisher('/risk_assessment/risk_metric', Float32, queue_size=1)		# using float metric

    def setup_analyzer(self):
        print "Settting up analyzer"

    def analyze(self):
        print "Analyzing..."

        # Risk analysis goes in here and should depend on several factors:
        # 	- Obstacles, weather, etc.
        #	- Should be something like 'weather condition + other factors - ...'
        #	- risk_metric should have some upper bound, such as 100
        #	- Need to get weather conditions and obstacle conditions from somewhere
        weather_conditions = 5
        obstacle_conditions = 5

        # compute final risk metric:
        risk_metric = weather_conditions + obstacle_conditions
        self.risk_metric_pub.publish(risk_metric)

if __name__ == "__main__":

    print "Starting Risk Assessment Node"

    rospy.init_node('risk_assessment_node', anonymous=True)

    risk_object = risk_analyzer()

    risk_object.setup_analyzer()

    rate = rospy.Rate(0.2) # Once every 5 seconds
    while not rospy.is_shutdown():
        risk_object.analyze()
        rate.sleep()
