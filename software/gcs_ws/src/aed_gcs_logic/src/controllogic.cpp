// %Tag(FULLTEXT)%
// %Tag(ROS_HEADER)%
#include "ros/ros.h"
// %EndTag(ROS_HEADER)%
// %Tag(MSG_HEADER)%
#include "std_msgs/String.h"
#include <std_msgs/Int8.h>
#include "mavros_msgs/Waypoint.h"
#include "mavros_msgs/CommandCode.h"
// %EndTag(MSG_HEADER)%

#include <sstream>




  void Callback(const std_msgs::String::ConstPtr& msg)
{
  ROS_INFO("I heard: [%s]", msg->data.c_str());
}

 void intCallback(const std_msgs::Int8::ConstPtr& msgint)
{
  ROS_INFO("I heard: [%d]", msgint->data);
//ROS_INFO("%d", msgint.data);
}


int main(int argc, char **argv)
{
  
// %Tag(INIT)%
  ros::init(argc, argv, "controllogic");
// %EndTag(INIT)%

// %Tag(NODEHANDLE)%
  ros::NodeHandle n;
// %EndTag(NODEHANDLE)%

  ros::Publisher Feed = n.advertise<std_msgs::String>("Feedback", 1000);
  ros::Publisher TakeOff = n.advertise<std_msgs::String>("Take_off", 1000);
  ros::Publisher GoWaypoint = n.advertise<mavros_msgs::Waypoint>("/mavros/mission/waypoints", 1000);
  ros::Publisher Land_Now = n.advertise<std_msgs::String>("Land", 1000);
 


  ros::Subscriber AbortMission = n.subscribe("AbortCmd", 1000, Callback);
 // ros::Subscriber WaypointNavigation = n.subscribe("WaypointNav", 1000, intCallback); // gets an int
  ros::Subscriber Motor_check = n.subscribe("MotorCheck", 1000, Callback);
  

  

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
    std_msgs::String msg3;
    std_msgs::String msg4;
    mavros_msgs::Waypoint way;


  
    std::stringstream ss1;
    std::stringstream ss2;
    std::stringstream ss3;
    std::stringstream ss4;

  
    ss1 << "Feedback " << count;
    ss2 << "Take Off " << count;
    ss3 << "Go to waypoint " << count;
    ss4 << "Land Now " << count;

   // msg.data = ss.str();
    msg1.data = ss1.str();
    msg2.data = ss2.str();
    msg3.data = ss3.str();
    msg4.data = ss4.str();

    way.frame = mavros_msgs::Waypoint::FRAME_GLOBAL;
    way.command = mavros_msgs::CommandCode::NAV_TAKEOFF;  
    way.is_current = true;
    way.autocontinue = true; 
    way.x_lat = 10;
    way.y_long = 10;
    way.z_alt = 20; 
    

// %EndTag(FILL_MESSAGE)%

// %Tag(ROSCONSOLE)%
    ROS_INFO("%s", msg.data.c_str());
// %EndTag(ROSCONSOLE)%

   
// %Tag(PUBLISH)%
   
    Feed.publish(msg1);
    TakeOff.publish(msg2);
    GoWaypoint.publish(way);
    Land_Now.publish(msg4);
    

// %EndTag(PUBLISH)%

// %Tag(SPINONCE)%
    ros::spinOnce();
// %EndTag(SPINONCE)%

// %Tag(RATE_SLEEP)%
    loop_rate.sleep();
// %EndTag(RATE_SLEEP)%
    ++count;
  }


  return 0;
}
// %EndTag(FULLTEXT)%
