#include "aed_gcs_logic/drone_logic.h"

drone_handler::drone_handler()
{
    this->state = IDLE;
    this->received_mission = false;
    this->off_board_requested = false;
    this->connected = false;
    this->armed = false;
    this->mode = "None";

    this->param_set_client = n.serviceClient<mavros_msgs::ParamSet>("mavros/param/set");
    this->param_get_client = n.serviceClient<mavros_msgs::ParamGet>("mavros/param/get");
    this->mission_push_client = n.serviceClient<mavros_msgs::WaypointPush>("mavros/mission/push");
    this->arming_client = n.serviceClient<mavros_msgs::CommandBool>("mavros/cmd/arming");
    this->set_mode_client = n.serviceClient<mavros_msgs::SetMode>("mavros/set_mode");
    this->state_sub = n.subscribe<mavros_msgs::State>("mavros/state", 1, &drone_handler::current_state_callback, this);
    this->mission_sub = n.subscribe<aed_gcs_logic::waypoints>("path", 1, &drone_handler::mission_callback, this);
    this->velocity_pub = n.advertise<geometry_msgs::PoseStamped>("mavros/setpoint_position/local", 1);
}

drone_handler::~drone_handler(){
}

bool drone_handler::wait_for_connection()
{
    for(auto i = 0; i < 10; i++){
        if(this->connected){
            std::cout << "Drone connected..." << std::endl;
            return true;
        }
        ros::spinOnce();
        ros::Duration(1).sleep();
    }
    return false;
}

bool drone_handler::setup()
{
    // Try to get the parameter NAV_DLL_ACT
    for(auto i = 0; i < 10; i++){
        mavros_msgs::ParamSet param_set_srv;
        param_set_srv.request.param_id = "NAV_DLL_ACT";
        this->param_set_client.call(param_set_srv);
        if(param_set_srv.response.success){
            return true;
        }
        ros::spinOnce();
        ros::Duration(1).sleep();
    }
    return false;
}

void drone_handler::current_state_callback(const mavros_msgs::State::ConstPtr& data)
{
    this->connected = data->connected;
    this->armed = data->armed;
    this->mode = data->mode;  // indicates that is a variable from this class
}

void drone_handler::mission_callback(const aed_gcs_logic::waypoints::ConstPtr& data)
{
    if(!this->received_mission){
        if(data->path.size() >= 2){
            mavros_msgs::WaypointPush mission_srv_temp;
            mavros_msgs::Waypoint temp_wp;

            temp_wp.frame = 3;
            temp_wp.command = 22;
            temp_wp.is_current = true;
            temp_wp.autocontinue = true;
            temp_wp.x_lat = data->path[0].latitude;
            temp_wp.y_long = data->path[0].longitude;
            temp_wp.z_alt = 50;

            mission_srv_temp.request.waypoints.push_back(temp_wp);
            std::cout << "Point 0: " << temp_wp.x_lat << "," << temp_wp.y_long << std::endl;

            temp_wp.command = 16;
            temp_wp.is_current = false;

            for(int i = 1; i < data->path.size() - 1; i++){
                temp_wp.x_lat = data->path[i].latitude;
                temp_wp.y_long = data->path[i].longitude;
                temp_wp.z_alt = 50;

                mission_srv_temp.request.waypoints.push_back(temp_wp);
                std::cout << "Point " << i << ": " << temp_wp.x_lat << "," << temp_wp.y_long << std::endl;
            }

            temp_wp.x_lat = data->path[data->path.size() - 1].latitude;
            temp_wp.y_long = data->path[data->path.size() - 1].longitude;
            temp_wp.z_alt = 50;
            temp_wp.command = 21;

            mission_srv_temp.request.waypoints.push_back(temp_wp);
            std::cout << "Point " << data->path.size() - 1 << ": " << temp_wp.x_lat << "," << temp_wp.y_long << std::endl;

            this->mission_srv = mission_srv_temp;

            this->path_cv.notify_one();

        }
        else{
            std::cout << "Not enough nodes in path" << std::endl;
        }

    }
    else{
        std::cout << "Drone already received mission..." << std::endl;
        std::cout << "Ignoring..." << std::endl;
    }
}

void drone_handler::run_state_machine()
{
    switch(this->state){
        case IDLE:{
            /* IDLE STATE */
            std::unique_lock<std::mutex> lock(this->path_m);
            this->path_cv.wait(lock);

            mavros_msgs::SetMode offboard_msg;

            offboard_msg.request.custom_mode = "OFFBOARD";
            this->set_mode_client.call(offboard_msg);
            if(offboard_msg.response.mode_sent){
                this->state = SEND_MISSION;
                this->received_mission = true;
            }
            else{
                this->state = FAILED;
            }
            break;}

        case FAILED:{
            std::cout << "Drone entered failed mode" << std::endl;
            break;}

		case SEND_MISSION:{
            /* SEND MISSION TO DRONE */
            this->mission_push_client.call(this->mission_srv);
            std::cout << "Mission ";
            if(this->mission_srv.response.success){
                std::cout << "Sent!" << std::endl;
                this->state = ARM;
            }
            else{
                std::cout << "Failed!" << std::endl;
                this->state = FAILED;
            }
            std::cout << "Mission wp sent: " << mission_srv.response.wp_transfered << std::endl;
            break;}


        case ARM:{
            /* ARMED STATE */
            if(!this->armed){
                mavros_msgs::CommandBool arm_msg;
                arm_msg.request.value = true;
                this->arming_client.call(arm_msg);
                if(arm_msg.response.success){
                    std::cout << "Armed sent" << std::endl;
                }
            }
            else{
                this->state = START_MISSION;
            }
            break;}
        case START_MISSION:{
            /* START MISSION */
            mavros_msgs::SetMode mission_msg;
            mission_msg.request.custom_mode = "AUTO.MISSION";
            this->set_mode_client.call(mission_msg);
            if(mission_msg.response.mode_sent){
				std::cout << "Mission Started!" << std::endl;
                this->state = ON_MISSION;
            }
            else{
                this->state = FAILED;
            }
            break;}
        case ON_MISSION:{
            if(!this->armed){
                std::cout << "Drone has finished mission" << std::endl;
                this->state = MISSION_DONE;
            }
            break;}
        case MISSION_DONE:
            /* DO NOTHING */
            break;
    }
}
