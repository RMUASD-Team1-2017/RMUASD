#pragma once

// Includes
#include <limits>
#include <vector>
#include <string>
#include "sensor_msgs/NavSatFix.h"

#ifdef SDL
#include <2D.hpp>
#endif

// Node inside the graph
struct Node{
    Node(sensor_msgs::NavSatFix _coord, int _id){
        coord = _coord;
        id = _id;
        cameFrom = nullptr;
        gScore = std::numeric_limits<double>::max();
        fScore = std::numeric_limits<double>::max();
    }
    sensor_msgs::NavSatFix coord;
    int id;
    std::vector<Node*> link;
    Node *cameFrom;
    double gScore;
    double fScore;
};

class path_planner
{
    public:

        path_planner(std::string geofence, std::string landingspotFile);
        void loadGeofence(std::string fileName);
        void loadMap(std::string fileName);
        void connectNodes();
        std::vector<Node*> aStar(sensor_msgs::NavSatFix start, sensor_msgs::NavSatFix goal);
        sensor_msgs::NavSatFix getNearestLandingSpot(sensor_msgs::NavSatFix start);

        static void printNode(Node *node);
        static void printList(std::vector<Node*> &list);
        static void printList(std::vector<Node> &list);
        static void printCoord(sensor_msgs::NavSatFix coord);

#ifdef SDL
        void draw(int size);
#endif

        std::vector<std::pair<sensor_msgs::NavSatFix, sensor_msgs::NavSatFix>> geofence;
        std::vector<Node> nodes;
        std::vector<sensor_msgs::NavSatFix> landingspot;
        std::vector<Node*> path;

    private:

        bool outOfBounds(Node *node1, Node *node2);
        int getIndex(sensor_msgs::NavSatFix coord);

        void loadLandingSpots(std::string flieName);
        bool intersection(std::pair<sensor_msgs::NavSatFix, sensor_msgs::NavSatFix> l1, std::pair<sensor_msgs::NavSatFix, sensor_msgs::NavSatFix> l2);
        double getAngle(sensor_msgs::NavSatFix c1, sensor_msgs::NavSatFix c2);
        void addLink(Node *node1, Node *node2);
        Node *getLowestFScore(std::vector<Node*> &list);
        void removeNode(std::vector<Node*> &list, Node *node);
        bool inList(std::vector<Node*> &list, Node *node);
        double dist(Node *start, Node *goal);
        double dist(sensor_msgs::NavSatFix *start, sensor_msgs::NavSatFix *goal);
        std::vector<Node*> reconstruc_path(Node *current);
};
