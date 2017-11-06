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
#include "std_msgs/Float64.h"
#include "std_srvs/Trigger.h"

#include "aed_gcs_logic/waypoints.h"
#include "aed_gcs_logic/AbortRequest.h"

#include <vector>

#include <string>
#include <mutex>
#include <condition_variable>
#include <math.h>
#include <GeographicLib/Geodesic.hpp>

using namespace GeographicLib;
// namespace drone_logic{
// NAMESPACE START

// #define IDLE                1
// #define OFF_BOARD_ENABLED   2
// #define ARMED               3
// #define MISSION_READY       4
// #define ON_MISSION          5
// #define MISSION_DONE        6
#define MAX_DIST_BETWEEN_WAYPOINTS  500

// Make sure this is the same as in the SRV file.
#define NOTHING                  0
#define TYPE_SOFT_ABORT_RTL      1
#define TYPE_SOFT_ABORT_LAND     2
#define TYPE_HARD_ABORT          3
#define CONTINUE                 4

enum droneState {
    IDLE = 1,
    FAILED = 2,
	SEND_MISSION = 3,
    ARM = 4,
    START_MISSION = 5,
    ON_MISSION = 6,
    MISSION_DONE = 7,
    SOFT_ABORT_RTL = 8,
    SOFT_ABORT_LAND = 9,
    HARD_ABORT = 10,
    WAIT_FOR_CONTINUE = 11,
	WAITING_STATE = 12,
    AUTO_MISSION = 13,
    WAIT_FOR_MAVROS = 14
};


class drone_handler
{
    public:
        drone_handler();
        bool wait_for_connection();
        bool setup();
        bool run_state_machine();

        ~drone_handler();
    private:
        ros::NodeHandle n;

        ros::ServiceServer abort_server;
        ros::ServiceServer restart_server;
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
        bool abort_callback(aed_gcs_logic::AbortRequest::Request& req, aed_gcs_logic::AbortRequest::Response& res);
        bool restart_callback(std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& res);
        void current_state_callback(const mavros_msgs::State::ConstPtr& data);
		void gps_callback(const sensor_msgs::NavSatFix::ConstPtr& data);
        void mission_callback(const aed_gcs_logic::waypoints::ConstPtr& data);

        // Functionality
		double calc_dist_between_waypoints(double lat1, double lat2, double long1, double long2);
        bool set_mode(const std::string& mode);
        bool set_arm(const bool& command);
        void check_mavros_connection();

        droneState state;
        droneState lastState;
        bool received_mission;
        bool connected;
        bool armed;
        bool mavrosConnection;
		double latitude;
		double longitude;
		double altitude;

        std::string mode;
        double current_height = 0;
        double start_height = 0;

		std::mutex abortM;
        int abortType;

		ros::Time start_time;
		ros::Time arm_time;

        mavros_msgs::WaypointPush mission_srv;
        std::mutex mission_m;

        std::condition_variable path_cv;
        std::mutex path_m;

        std::condition_variable reset_cv;
        std::mutex reset_m;
};

// NAMESPACE END
// }
