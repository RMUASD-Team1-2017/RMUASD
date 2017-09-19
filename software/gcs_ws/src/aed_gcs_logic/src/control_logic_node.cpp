#include "ros/ros.h"
#include <vector>

#include "std_msgs/String.h"
#include <std_msgs/Int8.h>
#include "mavros_msgs/Waypoint.h"
#include "mavros_msgs/WaypointList.h"
#include "mavros_msgs/WaypointPush.h"
#include "mavros_msgs/CommandCode.h"

#include <sstream>

#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>

#define STATE_IDLE                  0
#define STATE_OFF_BOARD_ENABLED     1
#define STATE_ARMED                 2
#define STATE_TAKE_OFF              3
#define STATE_IN_AIR                4
#define STATE_LANDING               5
#define STATE_MISSION_READY         6

bool armed;
bool connected;

void state_cb(const mavros_msgs::State::ConstPtr& msg){
    armed = msg->armed;
    connected = msg->connected;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "controllogic");
    ros::NodeHandle n;
    ros::Rate rate(20.0);

    ros::Publisher Feed = n.advertise<std_msgs::String>("Feedback", 1000);
    ros::Publisher TakeOff = n.advertise<std_msgs::String>("Take_off", 1000);
    ros::Publisher GoWaypoint = n.advertise<mavros_msgs::Waypoint>("/mavros/mission/waypoints", 1000);
    ros::Publisher Land_Now = n.advertise<std_msgs::String>("Land", 1000);
    ros::Publisher GoTo = n.advertise<geometry_msgs::PoseStamped>("/mavros/setpoint_position/local",1000);

    ros::Subscriber state_sub = n.subscribe<mavros_msgs::State>("mavros/state", 1, state_cb);
    ros::Publisher local_pos_pub = n.advertise<geometry_msgs::PoseStamped>("mavros/setpoint_position/local", 1000);
    ros::ServiceClient arming_client = n.serviceClient<mavros_msgs::CommandBool>("mavros/cmd/arming");
    ros::ServiceClient set_mode_client = n.serviceClient<mavros_msgs::SetMode>("mavros/set_mode");
    ros::ServiceClient land = n.serviceClient<mavros_msgs::SetMode>("/mavros/cmd/set_mode");
    ros::ServiceClient client = n.serviceClient<mavros_msgs::WaypointPush>("mavros/mission/push");
    // ros::Subscriber AbortMission = n.subscribe("AbortCmd", 1000, Callback);
    // ros::Subscriber Motor_check = n.subscribe("MotorCheck", 1000, Callback);

    while(ros::ok() && connected){  // waiting for heartbeat
        ros::spinOnce();
        rate.sleep();
    }

    std::cout << "Received heartbeat" << std::endl;

    geometry_msgs::PoseStamped pose;
    pose.pose.position.x = 0;
    pose.pose.position.y = 0;
    pose.pose.position.z = 2;

    //send a few setpoints before starting
    for(int i = 100; ros::ok() && i > 0; --i){
        local_pos_pub.publish(pose);
        ros::spinOnce();
        rate.sleep();
    }

    ros::Time last_request = ros::Time::now();

    bool off_board_requested = false;
    int StateOfFlight = STATE_IDLE;

    while(ros::ok()){
        // MAIN SWITCH CASE
        switch(StateOfFlight){
            case STATE_IDLE:
                /* IDLE STATE */
                std::cout << "Time: " << ros::Time::now() - last_request << std::endl;
                if( !off_board_requested && (ros::Time::now() - last_request > ros::Duration(1.0))){
                    mavros_msgs::SetMode offboard_msg;
                    offboard_msg.request.custom_mode = "OFFBOARD";
                    set_mode_client.call(offboard_msg);
                    if(offboard_msg.response.mode_sent){
                        StateOfFlight = STATE_OFF_BOARD_ENABLED;
                    }
                    else{
                        last_request = ros::Time::now();
                    }
                }
                break;
            case STATE_OFF_BOARD_ENABLED:
                /* OFF BOARD ENABLED */
                if(!armed){
                    mavros_msgs::CommandBool arm_msg;
                    arm_msg.request.value = true;
                    arming_client.call(arm_msg);
                    if(arm_msg.response.success){
                        StateOfFlight = STATE_ARMED;
                    }
                }
                else{
                    StateOfFlight = STATE_ARMED;
                }
                break;
            case STATE_ARMED:
                /* ARMED STATE */
                if(armed){
                    mavros_msgs::WaypointPush mission_srv;
                    mavros_msgs::Waypoint temp_wp;

                    temp_wp.frame = 3;
                    temp_wp.command = 22;
                    temp_wp.is_current = true;
                    temp_wp.autocontinue = true;
                    temp_wp.x_lat = 47.3977419;
                    temp_wp.y_long = 8.5455944;
                    temp_wp.z_alt = 5;

                    mission_srv.request.waypoints.push_back(temp_wp);

                    // 47.3977419, 8.5455944

                    temp_wp.command = 16;
                    temp_wp.is_current = false;
                    temp_wp.x_lat = 47.3980419;
                    temp_wp.y_long = 8.5450944;
                    temp_wp.z_alt = 10;

                    mission_srv.request.waypoints.push_back(temp_wp);

                    temp_wp.x_lat = 47.3975419;
                    temp_wp.y_long = 8.5452944;
                    temp_wp.z_alt = 10;

                    mission_srv.request.waypoints.push_back(temp_wp);

                    temp_wp.z_alt = 0;
                    temp_wp.command = 21;

                    mission_srv.request.waypoints.push_back(temp_wp);
                    mission_srv.request.start_index = 0;

                    client.call(mission_srv);
                    std::cout << "Mission ";
                    if(mission_srv.response.success){
                        StateOfFlight = STATE_MISSION_READY;
                        std::cout << "Success!" << std::endl;
                    }
                    else{
                        std::cout << "Failed!" << std::endl;
                    }
                    std::cout << "Mission wp sent: " << mission_srv.response.wp_transfered << std::endl;
                }
                break;
            case STATE_MISSION_READY:
                /* TAKING OFF */

                break;
            case STATE_IN_AIR:
                /* IDLE STATE */
                break;
            case STATE_LANDING:
                /* IDLE STATE */
                break;
            default:
                std::cout << "Error: Default switch case, value of StateOfFlight: " << StateOfFlight << std::endl;
                break;
        }
        // Feedback
        std::cout << "State: " << StateOfFlight << std::endl;
        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}
