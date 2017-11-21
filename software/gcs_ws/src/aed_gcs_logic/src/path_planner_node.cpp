#include "aed_gcs_logic/path_planner.h"

#include "ros/ros.h"
#include <ros/package.h>
#include "geometry_msgs/PoseArray.h"
#include "aed_gcs_logic/mission_request.h"
#include "aed_gcs_logic/waypoints.h"

#include <iostream>

ros::Publisher pub;
bool plan_path(aed_gcs_logic::mission_request::Request &req, aed_gcs_logic::mission_request::Response &res)
{
    std::string inner_geofence_file;
    ros::param::param<std::string>("/inner_geofence_file", inner_geofence_file, ros::package::getPath("aed_gcs_logic") + "/resources/polygon_inner_fence.fence");
    std::string geofence_file;
    ros::param::param<std::string>("/geofence_file", geofence_file, ros::package::getPath("aed_gcs_logic") + "/resources/polygon_fence.fence");
    std::string landing_spots_file;
    ros::param::param<std::string>("/landing_spots_file", landing_spots_file, ros::package::getPath("aed_gcs_logic") + "/resources/testrally.rally");

    path_planner planner(
        inner_geofence_file,  // shrinken geofence
        geofence_file,        // geofence
        landing_spots_file         // landing spots
    );
    Coord start(req.start.latitude, req.start.longitude);
    Coord goal(req.end.latitude, req.end.longitude);
    Coord landingPos = planner.getNearestLandingSpot(goal);

    Coord redirectedGoal(landingPos.geo.latitude, landingPos.geo.longitude);

    std::vector<Node*> waypoints = planner.aStar(start, landingPos);

#ifdef SDL
    planner.draw(960);
#endif
    aed_gcs_logic::waypoints path;
    for (int i = waypoints.size() - 1; i >= 0; i--){
        sensor_msgs::NavSatFix node;
        node.latitude = waypoints[i]->coord.geo.latitude;
        node.longitude = waypoints[i]->coord.geo.longitude;
        path.path.push_back(node);
        res.path.push_back(node);
    }

    std::cout << std::endl;
    std::cout << "Start position: " << std::endl;
    std::cout << "\tLatitude:   " << req.start.latitude << std::endl;
    std::cout << "\tLongtitude: " << req.start.longitude << std::endl;
    std::cout << std::endl;
    std::cout << "Requested goal position:" << std::endl;
    std::cout << "\tLatitude:   " << req.end.latitude << std::endl;
    std::cout << "\tLongtitude: " << req.end.longitude << std::endl;
    std::cout << std::endl;
    std::cout << "Redirected goal position:" << std::endl;
    std::cout << "\tLatitude:   " << redirectedGoal.geo.latitude << std::endl;
    std::cout << "\tLongtitude: " << redirectedGoal.geo.longitude << std::endl;
    std::cout << std::endl;
    std::cout << "The path consists of " << waypoints.size() << " waypoints:" << std::endl;

    for (int i = waypoints.size() - 1; i >= 0; i--){
        if (waypoints[i]->id == -2) std::cout << "Node id: Docking station" << waypoints[i]->id << std::endl;
        else if (waypoints[i]->id == -1) std::cout << "Node id: Landing spot" << std::endl;
        else std::cout << "Node id: " << waypoints[i]->id << std::endl;
        std::cout << "\tLatitude:   " << waypoints[i]->coord.geo.latitude << std::endl;
        std::cout << "\tLongtitude: " << waypoints[i]->coord.geo.longitude << std::endl;
    }
    std::cout << std::endl;
    
    path.path.erase(path.path.begin());
    pub.publish(path);
    return true;
}

int main(int argc, char *argv[])
{
    std::cout << "Starting \"Path Planner Node\"" << std::endl;

    ros::init(argc, argv, "path_planner_node");
    ros::NodeHandle nh;

    pub =  nh.advertise<aed_gcs_logic::waypoints>("path", 5);
    ros::ServiceServer service = nh.advertiseService("plan_path", plan_path);

    ros::spin();
}
