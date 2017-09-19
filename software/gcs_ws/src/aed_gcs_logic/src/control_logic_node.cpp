#include "ros/ros.h"
#include "aed_gcs_logic/drone_logic.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "control_logic_node");
    ros::NodeHandle n;
    ros::Rate rate(20.0);

    drone_handler handler;

    bool drone_ready = (handler.wait_for_connection() && handler.setup()) ? true : false;

    if(drone_ready){
        std::cout << "Drone Ready!" << std::endl << "Starting control..." << std::endl;

        while(ros::ok()){
            handler.run_state_machine();
            ros::spinOnce();
            rate.sleep();
        }
    }
    else{
        std::cout << "Drone did not get connection or setup failed" << std::endl;
        std::cout << "Exiting..." << std::endl;

    }
    // while(ros::ok()){
    //     // MAIN SWITCH CASE
    //     switch(StateOfFlight){
    //         case STATE_IDLE:
    //             /* IDLE STATE */
    //             std::cout << "Time: " << ros::Time::now() - last_request << std::endl;
    //             if( !off_board_requested && (ros::Time::now() - last_request > ros::Duration(1.0))){
    //                 mavros_msgs::SetMode offboard_msg;
    //                 offboard_msg.request.custom_mode = "OFFBOARD";
    //                 set_mode_client.call(offboard_msg);
    //                 if(offboard_msg.response.mode_sent){
    //                     StateOfFlight = STATE_OFF_BOARD_ENABLED;
    //                 }
    //                 else{
    //                     last_request = ros::Time::now();
    //                 }
    //             }
    //             break;
    //         case STATE_OFF_BOARD_ENABLED:
    //             /* OFF BOARD ENABLED */
    //             if(!armed){
    //                 mavros_msgs::CommandBool arm_msg;
    //                 arm_msg.request.value = true;
    //                 arming_client.call(arm_msg);
    //                 if(arm_msg.response.success){
    //                     StateOfFlight = STATE_ARMED;
    //                 }
    //             }
    //             else{
    //                 StateOfFlight = STATE_ARMED;
    //             }
    //             break;
    //         case STATE_ARMED:
    //             /* ARMED STATE */
    //             if(armed){
    //                 mavros_msgs::WaypointPush mission_srv;
    //                 mavros_msgs::Waypoint temp_wp;
    //
    //                 temp_wp.frame = 3;
    //                 temp_wp.command = 22;
    //                 temp_wp.is_current = true;
    //                 temp_wp.autocontinue = true;
    //                 temp_wp.x_lat = 47.3977419;
    //                 temp_wp.y_long = 8.5455944;
    //                 temp_wp.z_alt = 5;
    //
    //                 mission_srv.request.waypoints.push_back(temp_wp);
    //
    //                 // 47.3977419, 8.5455944
    //
    //                 temp_wp.command = 16;
    //                 temp_wp.is_current = false;
    //                 temp_wp.x_lat = 47.3980419;
    //                 temp_wp.y_long = 8.5450944;
    //                 temp_wp.z_alt = 10;
    //
    //                 mission_srv.request.waypoints.push_back(temp_wp);
    //
    //                 temp_wp.x_lat = 47.3975419;
    //                 temp_wp.y_long = 8.5452944;
    //                 temp_wp.z_alt = 10;
    //
    //                 mission_srv.request.waypoints.push_back(temp_wp);
    //
    //                 temp_wp.z_alt = 0;
    //                 temp_wp.command = 21;
    //
    //                 mission_srv.request.waypoints.push_back(temp_wp);
    //                 mission_srv.request.start_index = 0;
    //
    //                 client.call(mission_srv);
    //                 std::cout << "Mission ";
    //                 if(mission_srv.response.success){
    //                     StateOfFlight = STATE_MISSION_READY;
    //                     std::cout << "Success!" << std::endl;
    //                 }
    //                 else{
    //                     std::cout << "Failed!" << std::endl;
    //                 }
    //                 std::cout << "Mission wp sent: " << mission_srv.response.wp_transfered << std::endl;
    //             }
    //             break;
    //         case STATE_MISSION_READY:
    //             /* TAKING OFF */
    //
    //             break;
    //         case STATE_IN_AIR:
    //             /* IDLE STATE */
    //             break;
    //         case STATE_LANDING:
    //             /* IDLE STATE */
    //             break;
    //         default:
    //             std::cout << "Error: Default switch case, value of StateOfFlight: " << StateOfFlight << std::endl;
    //             break;
    //     }
    //     // Feedback
    //     std::cout << "State: " << StateOfFlight << std::endl;
    //     ros::spinOnce();
    //     rate.sleep();
    // }

    return 0;
}
