#include "ros/ros.h"
#include <iostream>

int main(int argc, char *argv[])
{
    std::cout << "Starting Health Check node" << std::endl;

    ros::init(argc, argv, "health_check_node");
    ros::NodeHandle nh;

    ros::spin();
}
