#include "aed_gcs_logic/path_planner.h"
#include <iostream>

int main(int argc, char *argv[])
{
    std::cout << "Path Planner Node" << std::endl;

    ros::init(argc, argv, "path_planner_node");
    ros::NodeHandle nh;

    /*
    Setup class object
    ...
    */

    ros::spin();
}
