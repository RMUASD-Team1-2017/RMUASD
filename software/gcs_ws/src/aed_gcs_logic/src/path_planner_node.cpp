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
    path_planner planner("resources/geofence2.csv", "resources/landingspots.csv");
    sensor_msgs::NavSatFix start = req.start;
    sensor_msgs::NavSatFix goal = req.end;
    sensor_msgs::NavSatFix landingPos = planner.getNearestLandingSpot(goal);
    res.result.latitude = landingPos.latitude;
    res.result.longitude = landingPos.longitude;
    res.result.altitude = landingPos.altitude;

    std::vector<Node*> waypoints = planner.aStar(start, landingPos);
#ifdef SDL
    planner.draw(500);
#endif
    aed_gcs_logic::waypoints path;
    for (int i = waypoints.size() - 1; i >= 0; i--){
        path.path.push_back(waypoints[i]->coord);
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
    std::cout << "\tLatitude:   " << res.result.latitude << std::endl;
    std::cout << "\tLongtitude: " << res.result.longitude << std::endl;
    std::cout << "\tAltitude:   " << res.result.altitude << std::endl;
    std::cout << std::endl;
    std::cout << "The path consists of " << waypoints.size() << " waypoints:" << std::endl;

    for (int i = 0; i < waypoints.size(); i++){
        std::cout << "Node id: " << waypoints[i]->id << std::endl;
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
