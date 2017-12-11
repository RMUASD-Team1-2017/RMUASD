#include "aed_gcs_logic/drone_logic.h"

drone_handler::drone_handler()
{
    this->state = IDLE;
    this->lastState = IDLE;
    this->received_mission = false;
    this->connected = false;
    this->mavrosConnection = false;
    this->armed = false;
    this->mode = "None";
	this->latitude = 0;
	this->longitude = 0;
	this->altitude = 0;
    this->abortType = 0;
    this->timeFromLastPosition = 0;

    this->abort_server = n.advertiseService("drone_logic/abort", &drone_handler::abort_callback, this);
    this->restart_server = n.advertiseService("drone_logic/restart", &drone_handler::restart_callback, this);
    this->param_set_client = n.serviceClient<mavros_msgs::ParamSet>("mavros/param/set");
    this->mission_push_client = n.serviceClient<mavros_msgs::WaypointPush>("mavros/mission/push");
    this->mission_pull_client = n.serviceClient<mavros_msgs::WaypointPull>("mavros/mission/pull");
    this->mission_clear_client = n.serviceClient<mavros_msgs::WaypointClear>("mavros/mission/clear");
    this->arming_client = n.serviceClient<mavros_msgs::CommandBool>("mavros/cmd/arming");
    this->set_mode_client = n.serviceClient<mavros_msgs::SetMode>("mavros/set_mode");
    this->state_sub = n.subscribe<mavros_msgs::State>("mavros/state", 1, &drone_handler::current_state_callback, this);
    this->mission_sub = n.subscribe<aed_gcs_logic::waypoints>("path", 1, &drone_handler::mission_callback, this);
    this->velocity_pub = n.advertise<geometry_msgs::PoseStamped>("mavros/setpoint_position/local", 1);
	this->pull_client = n.serviceClient<mavros_msgs::WaypointPull>("mavros/mission/pull");
	this->nav_sat_fix_gps = n.subscribe<sensor_msgs::NavSatFix>("mavros/global_position/global", 1, &drone_handler::gps_callback, this);
    this->send_mission_client = n.serviceClient<aed_gcs_logic::SendMission>("drone/set_mission");
}

drone_handler::~drone_handler(){
}

bool drone_handler::wait_for_connection()
{
    while(ros::ok()){
        if(this->connected){
            std::cout << "Drone connected..." << std::endl;
            return true;
        }
        std::cout << "Failed connecting to the drone! " << std::endl;
        ros::spinOnce();
        ros::Duration(1).sleep();
    }
    return false;
}

bool drone_handler::setup()
{
    // Try to get the parameter NAV_DLL_ACT
    while (ros::ok()){
        mavros_msgs::ParamSet param_set_srv;
        param_set_srv.request.param_id = "NAV_DLL_ACT";
        this->param_set_client.call(param_set_srv);
        if(param_set_srv.response.success){
            return true;
        }
        std::cout << "Failed setting up the drone! " << std::endl;
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
        this->reset = true;
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
    this->timeFromLastPosition = ros::Time::now().toSec();
}

void drone_handler::mission_callback(const aed_gcs_logic::waypoints::ConstPtr& data)
{
    double timeSinceLastGPS = ros::Time::now().toSec() - this->timeFromLastPosition;
    std::cout << "Time since last GPS: " << timeSinceLastGPS << std::endl;

    if(!this->received_mission && timeSinceLastGPS < 10){
        if(data->path.size() >= 1){
            mavros_msgs::WaypointPush mission_srv_temp;
            mavros_msgs::Waypoint temp_wp;

            aed_gcs_logic::waypoints localCopy = *data;

            temp_wp.frame = 3;
            temp_wp.command = 22;
            temp_wp.is_current = true;
            temp_wp.autocontinue = true;
            temp_wp.x_lat = this->latitude;
            temp_wp.y_long = this->longitude;
            temp_wp.z_alt = this->missionHeight;

            std::cout << "Pushing Back: " << temp_wp << std::endl;
            mission_srv_temp.request.waypoints.push_back(temp_wp);

            // Add starting point to local Copy
            sensor_msgs::NavSatFix tempCoord;
            tempCoord.latitude = this->latitude;
            tempCoord.longitude = this->longitude;
            localCopy.path.insert(localCopy.path.begin(), tempCoord);

            temp_wp.command = 16;
            temp_wp.is_current = false;

            for(int i = 1; i < localCopy.path.size() - 1; i++){
                double distanceBetweenPoints = calc_dist_between_waypoints(localCopy.path[i-1].latitude,
                                                                           localCopy.path[i].latitude,
                                                                           localCopy.path[i-1].longitude,
                                                                           localCopy.path[i].longitude);

				if(distanceBetweenPoints > MAX_DIST_BETWEEN_WAYPOINTS){
                    int numberOfWaypoints = ceil(distanceBetweenPoints / MAX_DIST_BETWEEN_WAYPOINTS);
					for(int counter = 1 ; counter < numberOfWaypoints; counter++){
						temp_wp.x_lat =  localCopy.path[i-1].latitude + (localCopy.path[i].latitude - localCopy.path[i-1].latitude) / numberOfWaypoints * counter ;
               		 	temp_wp.y_long = localCopy.path[i-1].longitude + (localCopy.path[i].longitude - localCopy.path[i-1].longitude) / numberOfWaypoints * counter;
                		temp_wp.z_alt = this->missionHeight;

                        std::cout << "Pushing Back: " << temp_wp << std::endl;
						mission_srv_temp.request.waypoints.push_back(temp_wp);
					}
				}

                temp_wp.x_lat = localCopy.path[i].latitude;
                temp_wp.y_long = localCopy.path[i].longitude;
                temp_wp.z_alt = this->missionHeight;

                std::cout << "Pushing Back: " << temp_wp << std::endl;
                mission_srv_temp.request.waypoints.push_back(temp_wp);
            }

            temp_wp.x_lat = data->path[data->path.size() - 1].latitude;
            temp_wp.y_long = data->path[data->path.size() - 1].longitude;
            temp_wp.z_alt = this->missionHeight;
            temp_wp.command = 21;

            std::cout << "Pushing Back: " << temp_wp << std::endl;
            mission_srv_temp.request.waypoints.push_back(temp_wp);

            std::unique_lock<std::mutex> lock(this->mission_m);
            this->mission_srv = mission_srv_temp;

            this->timeMissionReceived = ros::Time::now().toSec();
            this->received_mission = true;

        }
        else{
            std::cout << "Not enough nodes in path" << std::endl;
        }

    }
    else{
        std::cout << "Ignoring..." << std::endl;
    }
}


bool drone_handler::run_state_machine()
{
    if(!this->connected && this->state != WAIT_FOR_MAVROS){
        std::cout << "MAVROS Lost, going to waiting state" << std::endl;
        this->lastState = this->state;
        this->state = WAIT_FOR_MAVROS;
    }

    switch(this->state){
        case IDLE:{
            /* IDLE STATE */
            std::unique_lock<std::mutex> lock(this->mission_m);
            if(this->received_mission){
                std::cout << "Sending mission" << std::endl;
                this->state = SEND_MISSION;
            }
            break;}

        case FAILED:
            std::cout << "Drone Encountered an Error" << std::endl;
            this->state = MISSION_DONE;
            break;

		case SEND_MISSION:{
            /* SEND MISSION TO DRONE */
            aed_gcs_logic::SendMission srv;

            for(auto& node : this->mission_srv.request.waypoints){
                sensor_msgs::NavSatFix navsatfix;
                navsatfix.latitude = node.x_lat;
                navsatfix.longitude = node.y_long;
                navsatfix.altitude = node.z_alt;

                srv.request.path.push_back(navsatfix);
            }

            send_mission_client.call(srv);

            if(srv.response.response.data == "SUCCESS"){
                std::cout << "Setting mode to AUTO.MISSION" << std::endl;
                this->state = AUTO_MISSION;
            }
            // Check timer
            else if(ros::Time::now().toSec() - this->timeMissionReceived > ALLOWED_START_TIME){
                std::cout << "Timeout at Sending Mission" << std::endl;
                this->state = FAILED;
            }
            else{
                std::cout << "Send mission failed with response: ";
                std::cout << srv.response.response.data << std::endl;
            }
            break;}


        case ARM:
            /* ARM */
            if(this->armed){
                this->start_height = this->altitude;
                this->start_time = ros::Time::now();
                this->state = START_MISSION;
            }
            else{
                set_arm(true);
            }

            // Check timer
            if(ros::Time::now().toSec() - this->timeMissionReceived > ALLOWED_START_TIME){
                std::cout << "Timeout at Arming mode" << std::endl;
                this->state = FAILED;
            }
            break;

        case AUTO_MISSION:
            /* AUTO MISSION */

            if(this->mode == "AUTO.MISSION"){
                std::cout << "Trying to Arm" << std::endl;
                this->state = ARM;
            }
            else{
                set_mode("AUTO.MISSION");
            }

            // Check timer
            if(ros::Time::now().toSec() - this->timeMissionReceived > ALLOWED_START_TIME){
                std::cout << "Timeout at Setting to mission mode" << std::endl;
                this->state = FAILED;
            }
            break;

		case START_MISSION:
            /* START MISSION */
            this->current_height = this->altitude;
            //std::cout << "I am here " << std::endl;
            if(this->start_height + 2 < this->current_height){  // when the drone is 2 meters above its start position, it is assumed, that the mission is started succesfully.
                this->state = ON_MISSION; // the drone is in the air
                std::cout << "On Mission" << std::endl;
            }

            // Check timer
            else if(ros::Time::now().toSec() - this->timeMissionReceived > ALLOWED_START_TIME){
                std::cout << "Timeout at Setting to mission mode" << std::endl;
                this->state = FAILED;
            }
            break;

        case ON_MISSION:
			// be sure that the drone has been in air
       		if(!this->armed && this->connected){
                std::cout << "Mission Done" << std::endl;
                this->state = MISSION_DONE;
        	}
            else if(this->abortType != NOTHING){
                switch(abortType){
                    case TYPE_SOFT_ABORT_RTL:
                        std::cout << "Abort RTL called" << std::endl;
                        this->state = SOFT_ABORT_RTL;
                        break;
                    case TYPE_SOFT_ABORT_LAND:
                        std::cout << "Abort LAND called" << std::endl;
                        this->state = SOFT_ABORT_LAND;
                        break;
                    case TYPE_HARD_ABORT:
                        std::cout << "Hard Abort called" << std::endl;
                        this->state = HARD_ABORT;
                        break;
                }
            }
            break;
        case MISSION_DONE:{
            /* WAIT FOR RESTART */
            if(this->reset){
                std::cout << "Restarting..." << std::endl;
                this->received_mission = false;
                this->state = IDLE;
                this->reset = false;
            }
            break;}

        case SOFT_ABORT_RTL:
            if(this->mode == "AUTO.RTL"){
                this->state = WAIT_FOR_CONTINUE;
            }
            else{
                set_mode("AUTO.RTL");
            }
            break;

        case SOFT_ABORT_LAND:
            if(this->mode == "AUTO.LAND"){
                this->state = WAIT_FOR_CONTINUE;
            }
            else{
                set_mode("AUTO.LAND");
            }
            break;

        case HARD_ABORT:
            // NOT IMPLEMENTED NOW, BUT SHOULD DISARM MIDAIR.
            // WE WILL NOT IMPLEMENT THIS HERE FOR SAFETY REASONS.
            this->state = WAIT_FOR_CONTINUE;
            break;

        case WAIT_FOR_CONTINUE:
            if(abortType == CONTINUE){
                if(this->mode == "AUTO.MISSION"){
                    std::cout << "Continuing on mission" << std::endl;
                    this->abortType = NOTHING;
                    this->state = ON_MISSION;
                }
                else{
                    set_mode("AUTO.MISSION");
                }
            }
            else if(!armed){
                this->abortType = NOTHING;
                std::cout << "Mission arborted!" << std::endl;
                this->state = MISSION_DONE;
            }
            break;

        case WAIT_FOR_MAVROS:
            if(this->connected){
                std::cout << "MAVROS regained, continuing..." << std::endl;
                this->state = this->lastState;
            }
            break;

    }

    return true;
}
