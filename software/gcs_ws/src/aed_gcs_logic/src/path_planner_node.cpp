#include "ros/ros.h"
#include "geometry_msgs/PoseArray.h"
#include "aed_gcs_logic/mission_request.h"

#include <iostream>

int main(int argc, char *argv[])
{
    std::cout << "Path Planner Node" << std::endl;

    ros::init(argc, argv, "path_planner_node");
    ros::NodeHandle nh;

    
    ros::spin();
}
