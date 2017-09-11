

// %Tag(FULLTEXT)%
#include "ros/ros.h"
#include "std_msgs/String.h"

#include <std_msgs/Int8.h>
#include "mavros_msgs/Waypoint.h"
#include "mavros_msgs/CommandCode.h"

// %Tag(CALLBACK)%
void Callback(const std_msgs::String::ConstPtr& msg)
{
  ROS_INFO("I heard: [%s]", msg->data.c_str());
}

void MavrosCallback(const mavros_msgs::Waypoint::ConstPtr& msgs)
{
  ROS_INFO("I heard: [%s]", msgs->x_lat);
}
// %EndTag(CALLBACK)%

int main(int argc, char **argv)
{
 
  ros::init(argc, argv, "listener");

 
  ros::NodeHandle n;

 
// %Tag(SUBSCRIBER)%

  ros::Subscriber sub1 = n.subscribe("Feedback", 1000, Callback);
  ros::Subscriber sub2 = n.subscribe("Take_off", 1000, Callback);
  ros::Subscriber sub3 = n.subscribe("/mavros/mission/waypoints", 1000, MavrosCallback);
  ros::Subscriber sub4 = n.subscribe("Land", 1000, Callback);


  ros::Publisher Abs = n.advertise<std_msgs::String>("AbortCmd", 1000);
  ros::Publisher WaypointNa = n.advertise<std_msgs::String>("WaypointNav", 1000);
  ros::Publisher Mot = n.advertise<std_msgs::String>("MotorCheck", 1000);

 

  

// %Tag(LOOP_RATE)%
  ros::Rate loop_rate(10);
// %EndTag(LOOP_RATE)%

  
// %Tag(ROS_OK)%
  int count = 0;
  while (ros::ok())
  {
// %EndTag(ROS_OK)%
   
// %Tag(FILL_MESSAGE)%
    std_msgs::String msg;
    std_msgs::String msg1;
    std_msgs::String msg2;
    //int messageint= 5; 

    std::stringstream ss;
    std::stringstream ss1;
    std::stringstream ss2;
    ss << "Abort command " << count;
    ss1 << "Waypoint navigator " << count;
    ss2 << "Motor check " << count;


    msg.data = ss.str();
    msg1.data = ss1.str();
    msg2.data = ss2.str();


std_msgs::Int8 msgint;
msgint.data = 17;
ROS_INFO("%d", msgint.data);



// %EndTag(FILL_MESSAGE)%

// %Tag(ROSCONSOLE)%
    ROS_INFO("%s", msg.data.c_str());
// %EndTag(ROSCONSOLE)%

// %Tag(PUBLISH)%
    Abs.publish(msg);
    //WaypointNa.publish(msgint);
    Mot.publish(msg2);
    
    

// %EndTag(PUBLISH)%

// %Tag(SPINONCE)%
    ros::spinOnce();
// %EndTag(SPINONCE)%

// %Tag(RATE_SLEEP)%
    loop_rate.sleep();
// %EndTag(RATE_SLEEP)%
    ++count;
  }

 


// %EndTag(SUBSCRIBER)%

 
// %Tag(SPIN)%
  ros::spin();
// %EndTag(SPIN)%

  return 0;
}
// %EndTag(FULLTEXT)%
