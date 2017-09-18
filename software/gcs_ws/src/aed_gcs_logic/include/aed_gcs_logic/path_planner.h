#pragma once

// Includes
#include "ros/ros.h"
#include "geometry_msgs/PoseArray.h"
#include "aed_gcs_logic/mission_request.h"

// Structs
struct Coord{
    Coord(){
        x = 0;
        y = 0;
    };
    Coord(double _x, double _y){
        x = _x;
        y = _y;
    }
    double x;
    double y;
};

// Node inside the graph
struct Node{
    Node(Coord _coord, int _id){
        coord = _coord;
        id = _id;
        cameFrom = NULL;
        gScore = ULONG_MAX;
        fScore = ULONG_MAX;
    }
    Coord coord;
    int id;
    std::vector<Node*> link;
    Node *cameFrom;
    unsigned long gScore;
    unsigned long fScore;
};

class path_planner
{
    public:
        void loadGeofence(std::string fileName);
        void loadMap(std::string fileName);
        void connectNodes();
        std::vector<Node*> aStar(Node *start, Node *goal);

    private:
        static void printNode(Node *node);
        static void printList(std::vector<Node*> &list);
        static bool intersection(std::pair<Coord, Coord> l1, std::pair<Coord, Coord> l2);
        static double getAngle(Coord &c1, Coord &c2);
        static int getIndex(Coord coord, std::vector<std::pair<Coord, Coord>> &geofence);
        static bool outOfBounds(Node *node1, Node *node2, std::vector<std::pair<Coord, Coord>> &geofence);
        static void addLink(Node *node1, Node *node2);
        static Node *getLowestFScore(std::vector<Node*> &list);
        static void removeNode(std::vector<Node*> &list, Node *node);
        static bool inList(std::vector<Node*> &list, Node *node);
        static int dist(Node *start, Node *goal);
        static std::vector<Node*> reconstruc_path(Node *current);

        std::vector<std::pair<Coord, Coord>> geofence = loadGeofence("geofence.csv");

        std::vector<Node> nodes = loadMap("geofence.csv");
        connectNodes(nodes, geofence);

        std::vector<Node*> path = aStar(&nodes[0], &nodes[9]);

};
