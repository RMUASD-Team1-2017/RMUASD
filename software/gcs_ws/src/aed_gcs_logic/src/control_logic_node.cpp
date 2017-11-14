#include "ros/ros.h"
#include "aed_gcs_logic/drone_logic.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "control_logic_node");
    ros::NodeHandle n;
    ros::Rate rate(5.0);

    drone_handler handler;

    bool drone_ready = (handler.wait_for_connection() && handler.setup()) ? true : false;

    if(drone_ready){
        std::cout << "Drone Ready!" << std::endl << "Starting control..." << std::endl;
        ros::AsyncSpinner spinner(1);
        spinner.start();

        while(ros::ok()){
            if(!handler.run_state_machine()){
                break;
            }
            rate.sleep();
        }
        std::cout << "Drone logic exiting..." << std::endl;
    }
    else{
        std::cout << "Drone did not get connection or setup failed" << std::endl;
        std::cout << "Exiting..." << std::endl;

    }

    return 0;
}
