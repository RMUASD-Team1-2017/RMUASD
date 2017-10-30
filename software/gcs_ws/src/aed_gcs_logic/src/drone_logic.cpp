#include "aed_gcs_logic/drone_logic.h"

drone_handler::drone_handler()
{
    this->state = IDLE;
    this->received_mission = false;
    this->connected = false;
    this->armed = false;
    this->mode = "None";
	this->latitude = 0;
	this->longitude = 0;
	this->altitude = 0;
    this->abortType = 0;

    this->abort_server = n.advertiseService("drone_logic/abort", &drone_handler::abort_callback, this);
    this->restart_server = n.advertiseService("drone_logic/restart", &drone_handler::restart_callback, this);
    this->param_set_client = n.serviceClient<mavros_msgs::ParamSet>("mavros/param/set");
    this->mission_push_client = n.serviceClient<mavros_msgs::WaypointPush>("mavros/mission/push");
    this->arming_client = n.serviceClient<mavros_msgs::CommandBool>("mavros/cmd/arming");
    this->set_mode_client = n.serviceClient<mavros_msgs::SetMode>("mavros/set_mode");
    this->state_sub = n.subscribe<mavros_msgs::State>("mavros/state", 1, &drone_handler::current_state_callback, this);
    this->mission_sub = n.subscribe<aed_gcs_logic::waypoints>("path", 1, &drone_handler::mission_callback, this);
    this->velocity_pub = n.advertise<geometry_msgs::PoseStamped>("mavros/setpoint_position/local", 1);
	this->pull_client = n.serviceClient<mavros_msgs::WaypointPull>("mavros/mission/pull");
	this->nav_sat_fix_gps = n.subscribe<sensor_msgs::NavSatFix>("mavros/global_position/global", 1, &drone_handler::gps_callback, this);
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

bool drone_handler::set_mode(const std::string& mode){
    mavros_msgs::SetMode mode_msg;
    mode_msg.request.custom_mode = mode;
    this->set_mode_client.call(mode_msg);
    return mode_msg.response.mode_sent;
}

bool drone_handler::set_arm(const bool& mode){
    mavros_msgs::CommandBool arm_msg;
    arm_msg.request.value = mode;
    this->arming_client.call(arm_msg);
    return arm_msg.response.success;
}

bool drone_handler::abort_callback(aed_gcs_logic::AbortRequest::Request &req, aed_gcs_logic::AbortRequest::Response &res){
    int type = req.abort_type;
    if(type > 0 && type <= 4){
        if(type == CONTINUE){
            if(this->state == WAIT_FOR_CONTINUE){
                abortType = type;
                res.success = true;
                res.message = "Continuing";
            }
            else{
                res.success = false;
                res.message = "Not currently aborting";
            }
        }
        else{
            if(this->state == ON_MISSION){
                abortType = type;
                res.success = true;
                res.message = "Aborting";
            }
            else{
                res.success = false;
                res.message = "Not on mission";
            }
        }
    }
    else{
        res.success = false;
        res.message = "Please Specify a abort type within 1 - 4";
    }
    return true;
}

bool drone_handler::restart_callback(std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& res){
    if(this->state == MISSION_DONE){
        this->reset_cv.notify_all();
        res.success = true;
        res.message = "Restart accepted";
    }
    else{
        res.success = false;
        res.message = "Can't restart, drone is either idle or on mission";
    }

    return true;
}

double drone_handler::calc_dist_between_waypoints(double lat1, double lat2, double long1, double long2)
{
	double dist_between_waypoints_m;
	const Geodesic& geod = Geodesic::WGS84();
  	geod.Inverse(lat1, long1, lat2, long2, dist_between_waypoints_m);
	return dist_between_waypoints_m;
}


void drone_handler::current_state_callback(const mavros_msgs::State::ConstPtr& data)
{
    this->connected = data->connected;
    this->armed = data->armed;
    this->mode = data->mode;
}

void drone_handler::gps_callback(const sensor_msgs::NavSatFix::ConstPtr& data)
{
    this->latitude = data->latitude;
	this->longitude = data->longitude;
	this->altitude = data->altitude;
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
            temp_wp.z_alt = 20;

            mission_srv_temp.request.waypoints.push_back(temp_wp);

            temp_wp.command = 16;
            temp_wp.is_current = false;

            for(int i = 1; i < data->path.size() - 1; i++){
				double iterator = 0;
				int iterator_ceil = 0;

				if(calc_dist_between_waypoints(data->path[i-1].latitude,data->path[i].latitude,data->path[i-1].longitude,data->path[i].longitude) > MAX_DIST_BETWEEN_WAYPOINTS){
					iterator = (calc_dist_between_waypoints(data->path[i-1].latitude,data->path[i].latitude,data->path[i-1].longitude,data->path[i].longitude) / MAX_DIST_BETWEEN_WAYPOINTS);
					iterator_ceil = ceil( iterator );
					for(int counter = 1 ; counter<iterator_ceil; counter++){
						temp_wp.x_lat =  data->path[i-1].latitude + (((data->path[i].latitude - data->path[i-1].latitude)/iterator_ceil)*counter);
               		 	temp_wp.y_long = data->path[i-1].longitude + (((data->path[i].longitude - data->path[i-1].longitude)/iterator_ceil)*counter);
                		temp_wp.z_alt = 20;

						mission_srv_temp.request.waypoints.push_back(temp_wp);
					}
				}

                temp_wp.x_lat = data->path[i].latitude;
                temp_wp.y_long = data->path[i].longitude;
                temp_wp.z_alt = 20;

                mission_srv_temp.request.waypoints.push_back(temp_wp);
            }
			if(calc_dist_between_waypoints(data->path[data->path.size()-2].latitude,data->path[data->path.size()-1].latitude,data->path[data->path.size()-2].longitude,data->path[data->path.size()-1].longitude)>500){
				temp_wp.x_lat =  data->path[data->path.size()-2].latitude + ((data->path[data->path.size()-1].latitude - data->path[data->path.size()-2].latitude)/2);
       		 	temp_wp.y_long = data->path[data->path.size()-2].longitude + ((data->path[data->path.size()-1].longitude - data->path[data->path.size()-2].longitude)/2);
        		temp_wp.z_alt = 20;
				mission_srv_temp.request.waypoints.push_back(temp_wp);
			}


            temp_wp.x_lat = data->path[data->path.size() - 1].latitude;
            temp_wp.y_long = data->path[data->path.size() - 1].longitude;
            temp_wp.z_alt = 20;
            temp_wp.command = 21;

            mission_srv_temp.request.waypoints.push_back(temp_wp);

            std::unique_lock<std::mutex> lock(this->mission_m);
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

bool drone_handler::run_state_machine()
{
    switch(this->state){
        case IDLE:{
            /* IDLE STATE */
            std::unique_lock<std::mutex> lock(this->path_m);
            this->path_cv.wait(lock);
            this->state = SEND_MISSION;
            break;}

        case FAILED:
            std::cout << "Drone Encountered an Erorr" << std::endl;
            return false;
            break;

		case SEND_MISSION:{
            /* SEND MISSION TO DRONE */
            std::cout << "Sending Mission" << std::endl;
            std::unique_lock<std::mutex> lock(this->mission_m);
            this->received_mission = true;
            this->mission_push_client.call(this->mission_srv);
            this->state = this->mission_srv.response.success ? ARM : FAILED;
            ros::Duration(3).sleep();
            break;}

        case ARM:
            /* ARMED STATE */
            std::cout << "Trying to Arm" << std::endl;

            set_arm(true);
            this->arm_time = ros::Time::now();
            this->state = WAITING_STATE;
            ros::Duration(3).sleep();
            break;

		case WAITING_STATE:
            /* WATING STATE */
			// wait until the arm is registered
			if(this->armed){   // if armed go to next state.
                this->state = AUTO_MISSION;
            }
			else if(ros::Time::now() - arm_time > ros::Duration(5.0)){
                std::cout << "Armed" << std::endl;
				this->state = ARM;
            }
            break;


        case AUTO_MISSION:
            /* AUTO MISSION */
            std::cout << "Setting mode to AUTO.MISSION" << std::endl;
            ros::Duration(2).sleep();
            if(set_mode("AUTO.MISSION")){
				this->start_height = this->altitude;
				this->start_time = ros::Time::now();
                this->state = START_MISSION;
                std::cout << "Mission Started" << std::endl;
            }
            else{
                this->state = FAILED;
            }
            break;

		case START_MISSION:
            /* START MISSION */
            this->current_height = this->altitude;
            if(start_height + 5 < current_height){  // when the drone is 5 meters over its start position, it is assumed, that the mission is started succesfully.
                this->state = ON_MISSION; // the drone is in the air
                std::cout << "On Mission" << std::endl;
            }
			else if(ros::Time::now() - start_time > ros::Duration(10.0)){
				this->state = FAILED; // if not the drone is in the air after 10 seconds, it failed.
			}
            break;

        case ON_MISSION:
			// be sure that the drone has been in air
       		if(!this->armed){
                std::cout << "Mission Done" << std::endl;
                this->state = MISSION_DONE;
        	}
            else if(abortType != NOTHING){
                switch(abortType){
                    case TYPE_SOFT_ABORT_RTL:
                        this->state = SOFT_ABORT_RTL;
                        break;
                    case TYPE_SOFT_ABORT_LAND:
                        this->state = SOFT_ABORT_LAND;
                        break;
                    case TYPE_HARD_ABORT:
                        this->state = HARD_ABORT;
                        break;
                }
            }
            break;
        case MISSION_DONE:{
            /* WAIT FOR RESTART */
            std::unique_lock<std::mutex> lock(this->reset_m);
            this->reset_cv.wait(lock);
            this->received_mission = false;
            this->state = IDLE;
            break;}

        case SOFT_ABORT_RTL:
            this->state = set_mode("AUTO.RTL") ? WAIT_FOR_CONTINUE : FAILED;
            break;

        case SOFT_ABORT_LAND:
            this->state = set_mode("AUTO.LAND") ? WAIT_FOR_CONTINUE : FAILED;
            break;

        case HARD_ABORT:
            this->state = set_arm(false) ? MISSION_DONE : FAILED;
            break;

        case WAIT_FOR_CONTINUE:
            if(abortType == CONTINUE){
                this->abortType = NOTHING;
                this->state = set_mode("AUTO.MISSION") ? ON_MISSION : FAILED;
            }
            else if(!armed){
                this->abortType = NOTHING;
                this->state = MISSION_DONE;
            }
            break;
    }

    return true;
}
