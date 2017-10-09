#pragma once

// Includes
#include "ros/ros.h"
#include "mavros_msgs/Waypoint.h"
#include "mavros_msgs/WaypointPush.h"
#include "mavros_msgs/WaypointPull.h"
#include "mavros_msgs/CommandCode.h"
#include "mavros_msgs/CommandBool.h"
#include "mavros_msgs/SetMode.h"
#include "mavros_msgs/State.h"
#include "mavros_msgs/ParamSet.h"
#include "mavros_msgs/ParamGet.h"
#include "geometry_msgs/TwistStamped.h"
#include "geometry_msgs/PoseStamped.h"
#include "sensor_msgs/NavSatFix.h"
#include <std_msgs/Float64.h>
#include <GeographicLib/Geodesic.hpp>

#include "aed_gcs_logic/waypoints.h"

#include <vector>

#include <string>
#include <mutex>
#include <condition_variable>
#include <math.h> 
using namespace GeographicLib;
// namespace drone_logic{
// NAMESPACE START

// #define IDLE                1
// #define OFF_BOARD_ENABLED   2
// #define ARMED               3
// #define MISSION_READY       4
// #define ON_MISSION          5
// #define MISSION_DONE        6
#define max_dist_between_waypoints  500

enum droneState {
    IDLE = 1,
    FAILED = 2,
	SEND_MISSION = 3,
    ARM = 4,
	WAITING_STATE = 5,
    AUTO_MISSION = 6,
	START_MISSION = 7,
    ON_MISSION = 8,
    MISSION_DONE = 9
};

struct path{

};

class drone_handler
{
    public:
        drone_handler();
        bool wait_for_connection();
        bool setup();
        void run_state_machine();

        ~drone_handler();
    private:
        ros::NodeHandle n;

        ros::ServiceClient param_set_client; // = n.serviceClient<mavros_msgs::WaypointPush>("mavros/mission/push");
        ros::ServiceClient mission_push_client; // = n.serviceClient<mavros_msgs::WaypointPush>("mavros/mission/push");
        ros::ServiceClient arming_client; // n.serviceClient<mavros_msgs::CommandBool>("mavros/cmd/arming");
        ros::ServiceClient set_mode_client; // n.serviceClient<mavros_msgs::SetMode>("mavros/set_mode");
        ros::Subscriber state_sub; // n.subscribe<mavros_msgs::State>("mavros/state", 1, state_cb);
        ros::Subscriber mission_sub; // n.subscribe<mavros_msgs::State>("mavros/state", 1, state_cb);
        ros::Publisher velocity_pub; // n.subscribe<mavros_msgs::State>("mavros/state", 1, state_cb);
		ros::ServiceClient pull_client; // = n.serviceClient<mavros::WaypointPull>("/mission/WaypointPull");
		ros::Subscriber nav_sat_fix_gps; // = n.subscribe<sensor_msgs::NavSatFix>("mavros/global_position/global", 1, &drone_handler::mission_callback, this);
        // Callbacks
        void current_state_callback(const mavros_msgs::State::ConstPtr& data);
		void gps_callback(const sensor_msgs::NavSatFix::ConstPtr& data);
        void mission_callback(const aed_gcs_logic::waypoints::ConstPtr& data);
		double calc_dist_between_waypoints(double lat1, double lat2, double long1, double long2);

        droneState state;
        bool received_mission;
        bool off_board_requested;
        bool connected;
        bool armed;
		double latitude;
		double longitude;
		double altitude;
        std::string mode;
		double current_height = 0;
		double start_height = 0;
		
		
		
		ros::Time start_time;
		ros::Time arm_time;


		


        mavros_msgs::WaypointPush mission_srv;
        std::mutex path_m;
        std::condition_variable path_cv;
};

// NAMESPACE END
// }
