#!/usr/bin/env python
import rospy

from std_msgs.msg import String

class risk_analyzer:
    """Risk Analyzer class"""

    def __init__(self):
        self.risk_metric_pub = rospy.Publisher('/risk_assessment/risk_metric', String, queue_size=1)

    def setup_analyzer(self):
        print "Settting up analyzer"

    def analyze(self):
        print "Analyzing..."

        # Do analyze here
        # ...

        risk_metric = "100 percent clear"
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
