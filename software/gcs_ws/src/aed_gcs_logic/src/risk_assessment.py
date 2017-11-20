#!/usr/bin/env python
import rospy

from std_msgs.msg import String
from std_msgs.msg import Int8
from std_msgs.msg import Float32
import pyowm

owm = pyowm.OWM('ca3bb59d639ee6edca97b28794d28c7e')

class risk_analyzer:
    """Risk Analyzer class"""

    def __init__(self):
        #self.risk_metric_pub = rospy.Publisher('/risk_assessment/risk_metric', Int8, queue_size=1)	    	# using int metric
        self.risk_metric_pub = rospy.Publisher('/risk_assessment/risk_metric', Float32, queue_size=1)		# using float metric
        
        #s = rospy.Service('risk_assessment/risk_metric', RiskAssesmentService, analyze)
        rospy.wait_for_service('drone/Health_check_service')
        self.health_check_service = rospy.ServiceProxy("drone/Health_check_service", HealthCheckService)



    def BatteryAndGPStatus(self):
        #print "Battery and GPS status "
        if rospy.get_param('/ignore_weather_and_GPS', False) is True or True:
                return True

        try:
            request1 = HealthCheckServiceRequest()
            response1 = self.health_check_service(request1)
            print response1.flight
            #add_two_ints = rospy.ServiceProxy('add_two_ints', AddTwoInts)
            return response1.flight
        except rospy.ServiceException as e:
            print "Service call failed :"
            traceback.print_exc()



    def analyze(self):
        #print "Analyzing..."

        # get current weather conditions at some lat/long coordinate:
        try:
            latitude = 55.480935                                                        # currently set to HCA airport
            longitude = 10.333937
            obs = owm.weather_at_coords(latitude,longitude)
            w = obs.get_weather()
            current_wind_vector = w.get_wind()                                      # Get wind degree and speed format: {'deg': 59, 'speed': 2.660}
            current_wind_speed = current_wind_vector['speed']						# Units are metres per second
            current_rain_intensity = w.get_rain()
            if not current_rain_intensity:											# if it's not raining, current_rain_intensity will return empty - set to 0.
    			current_rain_intensity = 0
        except Exception, e:
        	current_wind_speed = 0
        	print 'Weather fetch error: ' + str(e)

        # set weather paramaters
        weather_conditions = 0
        wind_conditions = 0
        rain_conditions = 0
        max_operating_wind_speed = 20 												# the maximum wind speed that the drone can fly in
        max_operating_rain_intensity = 13   										# the maximum amount it can be raining and fly the drone. Units are ml?

        wind_conditions = max_operating_wind_speed - current_wind_speed
        rain_conditions = max_operating_rain_intensity - current_rain_intensity

        weather_conditions = 100 - 100*((wind_conditions/max_operating_wind_speed) + (rain_conditions/max_operating_rain_intensity))/2    # should be in range 0-100

        if current_wind_speed > max_operating_wind_speed:
        	print "*** WARNING: TOO WINDY TO FLY ***"
        	weather_conditions = 1000000 											# set weather conditions really high if it's too windy for drone to fly!
        if current_rain_intensity > max_operating_rain_intensity:
        	print "*** WARNING: TOO RAINY TO FLY ***"
        	weather_conditions = 1000000

        #include misc. factors
        number_of_golfers = 0
        number_of_other_obstacles = 0
        obstacle_conditions = number_of_golfers + number_of_other_obstacles

        # compute final risk metric:

        print 'For location ' + str(latitude) + ',' + str(longitude) + ':'
        #print 'Current wind speed is: ' + str(current_wind_speed)
        #print 'Current rain amount is: ' + str(current_rain_intensity)
        #print 'Current weather conditions are ' + str(weather_conditions) + '%' + ' ideal'
        risk_metric = weather_conditions + obstacle_conditions						# this should be normalised
        print 'Risk metric is ' + str(risk_metric) + '%' + ' ideal'
        
        if  not self.BatteryAndGPStatus():
            risk_metric = 1000000
            print "Battery and GPS condition is bad"

  
        self.risk_metric_pub.publish(risk_metric)


if __name__ == "__main__":

    print "Starting Risk Assessment Node"

    rospy.init_node('risk_assessment_node', anonymous=True)

    risk_object = risk_analyzer()

    rate = rospy.Rate(0.2) # Once every 5 seconds
    while not rospy.is_shutdown():
        risk_object.analyze()
        rate.sleep()
