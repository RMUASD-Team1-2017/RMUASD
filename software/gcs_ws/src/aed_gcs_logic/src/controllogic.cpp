// %Tag(FULLTEXT)%
// %Tag(ROS_HEADER)%
#include "ros/ros.h"
#include <vector>
// %EndTag(ROS_HEADER)%
// %Tag(MSG_HEADER)%
#include "std_msgs/String.h"
#include <std_msgs/Int8.h>
#include "mavros_msgs/Waypoint.h"
#include "mavros_msgs/WaypointList.h"
#include "mavros_msgs/WaypointPush.h"
#include "mavros_msgs/CommandCode.h"


#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
//#include <mavros_msgs/CommandTOL>


mavros_msgs::State current_state;


void state_cb(const mavros_msgs::State::ConstPtr& msg){
    current_state = *msg;
}

int StateOfFlight = 0;
int CounterTime = 0;
// %EndTag(MSG_HEADER)%

#include <sstream>

// HEJ HEJ




  void Callback(const std_msgs::String::ConstPtr& msg)
{
  ROS_INFO("I heard: [%s]", msg->data.c_str());
}

 void ListCallback(const mavros_msgs::WaypointList::ConstPtr& msgList)
{
 // ROS_INFO("I heard: [%d]", msgList->data);
//ROS_INFO("%d", msgint.data);
}

void updateWaypoint(mavros_msgs::WaypointList::ConstPtr& PointList){

  ros::NodeHandle ny;
  ros::Subscriber WaypointNavigation = ny.subscribe("WaypointNav", 1000, ListCallback); // gets an waypointList


}


int main(int argc, char **argv)
{

// %Tag(INIT)%
  ros::init(argc, argv, "controllogic");
// %EndTag(INIT)%

// %Tag(NODEHANDLE)%
  ros::NodeHandle n;
// %EndTag(NODEHANDLE)%
      ros::Rate rate(20.0);

  ros::Publisher Feed = n.advertise<std_msgs::String>("Feedback", 1000);
  ros::Publisher TakeOff = n.advertise<std_msgs::String>("Take_off", 1000);
  ros::Publisher GoWaypoint = n.advertise<mavros_msgs::Waypoint>("/mavros/mission/waypoints", 1000);
 	ros::Publisher Land_Now = n.advertise<std_msgs::String>("Land", 1000);
 	//ros::Publisher GoTo = n.advertise<geometry_msgs::PoseStamped>("/mavros/setpoint_position/local",1000); 
	



	ros::Subscriber state_sub = n.subscribe<mavros_msgs::State>("mavros/state", 1000, state_cb);
	ros::Publisher local_pos_pub = n.advertise<geometry_msgs::PoseStamped>("mavros/setpoint_position/local", 1000);
	//ros::Publisher global_pos_pub = n.advertise<geometry_msgs::PoseStamped>("mavros/setpoint");
	ros::ServiceClient arming_client = n.serviceClient<mavros_msgs::CommandBool>("mavros/cmd/arming");
	ros::ServiceClient set_mode_client = n.serviceClient<mavros_msgs::SetMode>("mavros/set_mode");
	ros::ServiceClient land = n.serviceClient<mavros_msgs::SetMode>("/mavros/cmd/set_mode");
	ros::ServiceClient client = n.serviceClient<mavros_msgs::WaypointPush>("mavros/mission/push");
	//ros::ServiceClient Wayli = n.serviceClient<mavros_msgs::WayPoint>("/mavros/mission/set_current");

  while(ros::ok() && current_state.connected){  // waiting for heartbeat
        ros::spinOnce();
        rate.sleep();
    }


  ros::Subscriber AbortMission = n.subscribe("AbortCmd", 1000, Callback);
  
  ros::Subscriber Motor_check = n.subscribe("MotorCheck", 1000, Callback);
  //ros::Subscriber ListOfWaypoints = n.subscribe("mavros_msgs/WaypointList  ",1000,ListCallback);
	int count = 1; 
  geometry_msgs::PoseStamped pose;
  geometry_msgs::PoseStamped position1;
  //position1.pose.position.x = ListOfWaypoints[1];
 // mavros_msgs::WaypointList Way;
	int ArrayListWaypoint [9] = {0.0,0.0,1.0,2.0,5.0,1.0,0.0,0.0,1.0};
	pose.header.stamp = ros::Time::now();
	pose.header.seq = count; 
	pose.header.frame_id = 1;
	
	pose.pose.position.x = 0.0;
	pose.pose.position.y = 0.0;
	pose.pose.position.z = 1.0;
 // mavros_msgs.Way[pose];


// %Tag(LOOP_RATE)%
  ros::Rate loop_rate(10);
// %EndTag(LOOP_RATE)%


//for(int i = 100; ros::ok() && i > 0; --i){
       // GoTo.publish(pose);
        //ros::spinOnce();
        //rate.sleep();
  //  }

mavros_msgs::SetMode offb_set_mode;
mavros_msgs::SetMode land_set_mode;
mavros_msgs::SetMode auto_set_mode;
mavros_msgs::Waypoint wp;
mavros_msgs::WaypointPush service2;
mavros_msgs::SetMode takeoff_set_mode;

mavros_msgs::WaypointList WPList;


    offb_set_mode.request.custom_mode = "OFFBOARD";
	takeoff_set_mode.request.custom_mode = "AUTO.TAKEOFF";
    land_set_mode.request.custom_mode = "AUTO.LAND";
    auto_set_mode.request.custom_mode = "AUTO.MISSION";   // should load waypoints automatically when flying

	wp.frame = mavros_msgs::Waypoint::FRAME_MISSION; 
	wp.command = mavros_msgs::CommandCode::NAV_WAYPOINT;
	wp.is_current = false ;
	wp.autocontinue = true;
	wp.x_lat = 7;
	wp.y_long = 2;
	wp.z_alt = 1;

	service2.request.waypoints.push_back(wp);
	WPList.waypoints.push_back(wp);
	std::cout<<" waypoint List1		"<<wp<<std::endl;
	wp.frame = mavros_msgs::Waypoint::FRAME_MISSION; 
	wp.command = mavros_msgs::CommandCode::NAV_WAYPOINT;
	wp.is_current = false ;
	wp.autocontinue = true;
	wp.x_lat = 2;
	wp.y_long = 5;
	wp.z_alt = 1;

	service2.request.waypoints.push_back(wp);
	WPList.waypoints.push_back(wp);
	std::cout<<" waypoint List2		"<<wp<<std::endl;
	wp.frame = mavros_msgs::Waypoint::FRAME_MISSION; 
	wp.command = mavros_msgs::CommandCode::NAV_WAYPOINT;
	wp.is_current = false ;
	wp.autocontinue = true;
	wp.x_lat = 0;
	wp.y_long = 0;
	wp.z_alt = 1;
    
	service2.request.waypoints.push_back(wp);
	WPList.waypoints.push_back(wp);
	std::cout<<" waypoint List3		"<<wp<<std::endl;
	//std::cout<<" service "<<service2<<std::endl;
mavros_msgs::CommandBool arm_cmd;
    arm_cmd.request.value = true;

ros::Time last_request = ros::Time::now();

	std::cout<<" Start While loop "<<current_state<<std::endl;
 while(ros::ok()){
       // std::cout<<" While "<<current_state<<std::endl;
        if( current_state.mode != "OFFBOARD" && (ros::Time::now() - last_request > ros::Duration(5.0))){
		std::cout<<" hej "<<std::endl;
            if( set_mode_client.call(offb_set_mode) && StateOfFlight == 0 ){//&& offb_set_mode.response.mode_sent){
                ROS_INFO("Offboard enabled");
		StateOfFlight=1;
            }
	    std::cout<<" hej1 "<<StateOfFlight<<std::endl;
	    CounterTime++;
            last_request = ros::Time::now();
        }   else {
	   // std::cout<<" hej2 "<<std::endl;
            if( !current_state.armed && StateOfFlight == 1 ){//&& (ros::Time::now() - last_request > ros::Duration(5.0))){
		std::cout<<" hej3 "<<current_state<<std::endl;
                if( arming_client.call(arm_cmd) && arm_cmd.response.success){
                    ROS_INFO("Vehicle armed");
	 	    StateOfFlight = 2;
                }
                last_request = ros::Time::now();
            }
        }

	if(current_state.mode != "TAKEOFF" && StateOfFlight == 2){
	if( set_mode_client.call(takeoff_set_mode) && StateOfFlight == 2 && ros::Duration(2).sleep()){//&& offb_set_mode.response.mode_sent){
        ROS_INFO("Takeoff"); // auto takeoff
		StateOfFlight=3;
		std::cout<<" Takeoff2 "<<current_state<<std::endl;
		ros::Duration(10).sleep();
            }
	else if(StateOfFlight == 2) {
		ROS_INFO("Failed to enable Takeoff");
	}
	}


	
	

	//if(ros::Time::now() - last_request > ros::Duration(5.0)){
    local_pos_pub.publish(pose);
	//}
    ros::spinOnce();
    rate.sleep();
	 
	if(StateOfFlight == 3 ){//&& current_state.mode != "AUTO.LOITER"){
			ros::Duration(10).sleep(); 
        	//set_mode_client.call(land_set_mode);   // See link : http://wiki.ros.org/mavros/CustomModes#PX4_native_flight_stack
 			//ArrayListWaypoint
		//for(int i = 0; i<9;i+=3){
			//std::cout<<"position x "<<ArrayListWaypoint[i]<<" y "<<ArrayListWaypoint[i+1]<<" z "<<ArrayListWaypoint[i+2]<<std::endl;
			pose.pose.position.x = 2;// ArrayListWaypoint[i];
 	 		pose.pose.position.y = 5; //ArrayListWaypoint[i+1];
 			 pose.pose.position.z = 1; //ArrayListWaypoint[i+2];
			//GoTo.publish(pose);
			local_pos_pub.publish(pose);
			client.call(service2);
			//Wayli.call(service2);
			std::cout<<" all waypoints are sent "<<std::endl;
			//std::cout<<" number "<<i<<std::endl;
			std::cout<<" Num "<<current_state<<std::endl;
        	ros::Duration(5).sleep(); 
		//}
	//std::cout<<" hej7 "<<std::endl;
	StateOfFlight = 4; 
	//ros::Duration(12).sleep();
	}

if(current_state.mode != "AUTO.MISSION" && StateOfFlight == 4){
	if( StateOfFlight == 4 && ros::Duration(2).sleep()){//&& offb_set_mode.response.mode_sent){
        set_mode_client.call(auto_set_mode);
		ROS_INFO("AUTO.MISSION enabled");
		StateOfFlight=5;
		std::cout<<" AUTO "<<current_state<<std::endl;
            }
	else if(StateOfFlight == 4) {
		ROS_INFO("Failed to enable AUTO.MISSION");
	}
	}

   	if(StateOfFlight == 5 && current_state.mode != "AUTO.LAND"  && current_state.mode != "AUTO.RTL" ){
        set_mode_client.call(land_set_mode);   // See link : http://wiki.ros.org/mavros/CustomModes#PX4_native_flight_stack
 		std::cout<<" AUTO.LAND "<<current_state<<std::endl;
		//pose.pose.position.x = 0;
 		// pose.pose.position.y = 2;
		// pose.pose.position.z = 0;
		//local_pos_pub.publish(pose);
		//std::cout<<" hej7 "<<std::endl;
		//StateOfFlight = 3; 
	}

	rate.sleep();
    }


// %Tag(ROS_OK)%
/*  int count = 0;
  while (ros::ok())
  {

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
    way.command = mavros_msgs::CommandCode::NAV_WAYPOINT;
    way.is_current = true;
    way.autocontinue = true;
    way.x_lat = 0.5;
    way.y_long = 0.7;
    way.z_alt = 20;


// %EndTag(FILL_MESSAGE)%

// %Tag(ROSCONSOLE)%
    ROS_INFO("%s", msg.data.c_str());
    ROS_INFO("%i", pose.pose.position);
    
// %EndTag(ROSCONSOLE)%


// %Tag(PUBLISH)%

    Feed.publish(msg1);
    TakeOff.publish(msg2);
    GoWaypoint.publish(way);
    Land_Now.publish(msg4);
    GoTo.publish(pose);


// %EndTag(PUBLISH)%

// %Tag(SPINONCE)%
    ros::spinOnce();
// %EndTag(SPINONCE)%

// %Tag(RATE_SLEEP)%
    loop_rate.sleep();
// %EndTag(RATE_SLEEP)%
    ++count;
  }

*/
  return 0;
}
// %EndTag(FULLTEXT)%
