#pragma once

// Includes
#include "ros/ros.h"
#include "mavros_msgs/Waypoint.h"
#include "mavros_msgs/WaypointPush.h"
#include "mavros_msgs/CommandCode.h"
#include "mavros_msgs/CommandBool.h"
#include "mavros_msgs/SetMode.h"
#include "mavros_msgs/State.h"
#include "mavros_msgs/ParamSet.h"

#include "geometry_msgs/PoseStamped.h"

#include <vector>
#include <string>

// namespace drone_logic{
// NAMESPACE START

// #define IDLE                1
// #define OFF_BOARD_ENABLED   2
// #define ARMED               3
// #define MISSION_READY       4
// #define ON_MISSION          5
// #define MISSION_DONE        6

enum droneState {
    IDLE = 1,
    OFF_BOARD_ENABLED = 2,
    ARMED = 3,
    MISSION_READY = 4,
    ON_MISSION = 5,
    MISSION_DONE = 6
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

        // Callbacks
        void current_state_callback(const mavros_msgs::State::ConstPtr& data);

        droneState state;
        bool off_board_requested;
        bool connected;
        bool armed;
        std::string mode;

};

// NAMESPACE END
// }
