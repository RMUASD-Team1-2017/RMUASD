#include "aed_gcs_logic/path_planner.h"

#include "ros/ros.h"
#include "geometry_msgs/PoseArray.h"
#include "aed_gcs_logic/mission_request.h"
#include "aed_gcs_logic/waypoints.h"
#include "std_msgs/Int8.h"

#include <iostream>

ros::Publisher pub;
bool plan_path(aed_gcs_logic::mission_request::Request &req, aed_gcs_logic::mission_request::Response &res)
{
    path_planner planner("resources/geofence.csv", "resources/landingspots.csv");
    std::cout << req.start.latitude << std::endl;
    Coord start(req.start.latitude, req.start.longitude, req.start.altitude);
    Coord goal(req.end.latitude, req.end.longitude, req.end.altitude);
    Coord landingPos = planner.getNearestLandingSpot(goal);
    res.result.latitude = landingPos.x;
    res.result.longitude = landingPos.y;
    res.result.altitude = landingPos.z;
    std::vector<Node*> waypoints = planner.aStar(start, landingPos);
    planner.draw(500);
    std::cout << waypoints.size() << std::endl;
    for (int i = 0; i < waypoints.size(); i++){
        std::cout << waypoints[i]->id << ": " << waypoints[i]->coord.x << ", " << waypoints[i]->coord.y << ", " << waypoints[i]->coord.z << std::endl;
    }

    aed_gcs_logic::waypoints path;
    for (int i = 0; i < waypoints.size(); i++){
        sensor_msgs::NavSatFix pos;
        pos.latitude = waypoints[i]->coord.x;
        pos.longitude = waypoints[i]->coord.y;
        pos.altitude = waypoints[i]->coord.z;
        path.path.push_back(pos);
    }

    pub.publish(path);
    return true;
}

int main(int argc, char *argv[])
{
    std::cout << "Path Planner Node" << std::endl;

    ros::init(argc, argv, "path_planner_node");
    ros::NodeHandle nh;

    pub =  nh.advertise<aed_gcs_logic::waypoints>("path", 5);
    ros::ServiceServer service = nh.advertiseService("plan_path", plan_path);

    ros::spin();
}
