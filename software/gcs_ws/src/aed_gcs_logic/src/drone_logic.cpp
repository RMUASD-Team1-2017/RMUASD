#include "aed_gcs_logic/drone_logic.h"

drone_handler::drone_handler()
{
    this->state = IDLE;
    this->off_board_requested = false;
    this->connected = false;
    this->armed = false;
    this->mode = "None";

    this->param_set_client = n.serviceClient<mavros_msgs::ParamSet>("mavros/param/set");
    this->mission_push_client = n.serviceClient<mavros_msgs::WaypointPush>("mavros/mission/push");
    this->arming_client = n.serviceClient<mavros_msgs::CommandBool>("mavros/cmd/arming");
    this->set_mode_client = n.serviceClient<mavros_msgs::SetMode>("mavros/set_mode");
    this->state_sub = n.subscribe<mavros_msgs::State>("mavros/state", 1, &drone_handler::current_state_callback, this);
}

drone_handler::~drone_handler(){
}

bool drone_handler::wait_for_connection()
{
    bool result = false;
    for(auto i = 0; i < 5; i++){
        if(this->connected){
            result = true;
            break;
        }
        ros::spinOnce();
        ros::Duration(1).sleep();
    }
    return result;
}

bool drone_handler::setup()
{
    bool result = false;
    mavros_msgs::ParamSet param_srv;
    param_srv.request.param_id = "NAV_DLL_ACT";
    this->param_set_client.call(param_srv);
    if(param_srv.response.success){
        result = true;
    }
    return result;
}

void drone_handler::current_state_callback(const mavros_msgs::State::ConstPtr& data)
{
    this->connected = data->connected;
    this->armed = data->armed;
    this->mode = data->mode;
}

void drone_handler::run_state_machine()
{
    switch(this->state){
        case IDLE:{
            /* IDLE STATE */
            mavros_msgs::SetMode offboard_msg;
            offboard_msg.request.custom_mode = "OFFBOARD";
            this->set_mode_client.call(offboard_msg);
            if(offboard_msg.response.mode_sent){
                this->state = OFF_BOARD_ENABLED;
            }
            break;}

        case OFF_BOARD_ENABLED:{
            /* OFF BOARD ENABLED */
            if(!this->armed){
                mavros_msgs::CommandBool arm_msg;
                arm_msg.request.value = true;
                this->arming_client.call(arm_msg);
                if(arm_msg.response.success){
                    this->state = ARMED;
                }
            }
            else{
                this->state = ARMED;
            }
            break;}
        case ARMED:{
            /* ARMED STATE */
            if(this->armed){
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

                this->mission_push_client.call(mission_srv);
                std::cout << "Mission ";
                if(mission_srv.response.success){
                    this->state = MISSION_READY;
                    std::cout << "Sent!" << std::endl;
                }
                else{
                    std::cout << "Failed!" << std::endl;
                }
                std::cout << "Mission wp sent: " << mission_srv.response.wp_transfered << std::endl;
            }
            break;}
        case MISSION_READY:{
            /* MISSION READY */
            mavros_msgs::SetMode mission_msg;
            mission_msg.request.custom_mode = "AUTO.MISSION";
            this->set_mode_client.call(mission_msg);
            if(mission_msg.response.mode_sent){
                this->state = ON_MISSION;
            }
            break;}
        case ON_MISSION:

            break;
        case MISSION_DONE:
            /* IDLE STATE */
            break;
    }
}
