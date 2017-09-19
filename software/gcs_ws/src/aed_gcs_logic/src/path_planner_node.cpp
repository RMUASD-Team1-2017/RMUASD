#include "aed_gcs_logic/path_planner.h"

#include "ros/ros.h"
#include "geometry_msgs/PoseArray.h"
#include "aed_gcs_logic/mission_request.h"
#include "std_msgs/Int8.h"

#include <iostream>

bool plan_path(aed_gcs_logic::mission_request::Request &req, aed_gcs_logic::mission_request::Response &res){
    
    path_planner planner("resources/geofence.csv", "landingspots.csv");
    std::vector<Node*> waypoints = planner.aStar(&planner.nodes[0], &planner.nodes[10]);
    planner.draw();
    std::cout << waypoints.size() << std::endl;
    for (int i = 0; i < waypoints.size(); i++){
        std::cout << waypoints[i]->id << std::endl;
    }
    return true;
}

int main(int argc, char *argv[])
{
    std::cout << "Path Planner Node" << std::endl;

    ros::init(argc, argv, "path_planner_node");
    ros::NodeHandle nh;

    ros::ServiceServer service = nh.advertiseService("plan_path", plan_path);

    ros::spin();
}
