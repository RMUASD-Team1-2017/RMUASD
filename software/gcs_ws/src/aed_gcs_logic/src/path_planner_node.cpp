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
    path_planner planner(
        ros::package::getPath("aed_gcs_logic") + "/resources/airport_inner.fence",  // geofence
        ros::package::getPath("aed_gcs_logic") + "/resources/airport.fence",  // shrinken geofence
        ros::package::getPath("aed_gcs_logic") + "/resources/airport_rally.rally"       // landing spots
    );
    Coord start;
    start.latitude = req.start.latitude;
    start.longitude = req.start.longitude;
    Coord goal;
    goal.latitude = req.end.latitude;
    goal.longitude = req.end.longitude;
    Coord landingPos = planner.getNearestLandingSpot(goal);

    res.goal.latitude = landingPos.latitude;
    res.goal.longitude = landingPos.longitude;
    res.goal.altitude = landingPos.altitude;



    std::vector<Node*> waypoints = planner.aStar(start, landingPos);

#ifdef SDL
    planner.draw(960);
#endif
    aed_gcs_logic::waypoints path;
    for (int i = waypoints.size() - 1; i >= 0; i--){
        sensor_msgs::NavSatFix node;
        node.latitude = waypoints[i]->coord.latitude;
        node.longitude = waypoints[i]->coord.longitude;
        node.altitude = waypoints[i]->coord.altitude;
        path.path.push_back(node);
    }

    std::cout << std::endl;
    std::cout << "Start position: " << std::endl;
    std::cout << "\tLatitude:   " << req.start.latitude << std::endl;
    std::cout << "\tLongtitude: " << req.start.longitude << std::endl;
    std::cout << "\tAltitude:   " << req.start.altitude << std::endl;
    std::cout << std::endl;
    std::cout << "Requested goal position:" << std::endl;
    std::cout << "\tLatitude:   " << req.end.latitude << std::endl;
    std::cout << "\tLongtitude: " << req.end.longitude << std::endl;
    std::cout << "\tAltitude:   " << req.end.altitude << std::endl;
    std::cout << std::endl;
    std::cout << "Redirected goal position:" << std::endl;
    std::cout << "\tLatitude:   " << res.goal.latitude << std::endl;
    std::cout << "\tLongtitude: " << res.goal.longitude << std::endl;
    std::cout << "\tAltitude:   " << res.goal.altitude << std::endl;
    std::cout << std::endl;
    std::cout << "The path consists of " << waypoints.size() << " waypoints:" << std::endl;

    for (int i = waypoints.size() - 1; i >= 0; i--){
        if (waypoints[i]->id == -2) std::cout << "Node id: Docking station" << waypoints[i]->id << std::endl;
        else if (waypoints[i]->id == -1) std::cout << "Node id: Landing spot" << std::endl;
        else std::cout << "Node id: " << waypoints[i]->id << std::endl;
        std::cout << "\tLatitude:   " << waypoints[i]->coord.latitude << std::endl;
        std::cout << "\tLongtitude: " << waypoints[i]->coord.longitude << std::endl;
        std::cout << "\tAltitude:   " << waypoints[i]->coord.altitude << std::endl;
    }
    std::cout << std::endl;

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
