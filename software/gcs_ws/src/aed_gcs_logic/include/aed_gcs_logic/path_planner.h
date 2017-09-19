#pragma once

// Includes
#include <limits>
#include <vector>

#ifdef SDL
#include <2D.hpp>
#endif

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
        gScore = std::numeric_limits<double>::max();
        fScore = std::numeric_limits<double>::max();
    }
    Coord coord;
    int id;
    std::vector<Node*> link;
    Node *cameFrom;
    double gScore;
    double fScore;
};

class path_planner
{
    public:

        path_planner(std::string geofence, std::string landingspots);
        void loadGeofence(std::string fileName);
        void loadMap(std::string fileName);
        void connectNodes();
        std::vector<Node*> aStar(Node *start, Node *goal);
        Coord getNearestLandingSpot(Coord start);

#ifdef SDL
        void draw();
#endif

        std::vector<std::pair<Coord, Coord>> geofence;
        std::vector<Node> nodes;
        std::vector<Node*> path;
#ifdef SDL
        Window *window;
#endif

    private:

        bool outOfBounds(Node *node1, Node *node2);
        int getIndex(Coord coord);

        void printNode(Node *node);
        void printList(std::vector<Node*> &list);
        void printList(std::vector<Node> &list);
        bool intersection(std::pair<Coord, Coord> l1, std::pair<Coord, Coord> l2);
        double getAngle(Coord c1, Coord c2);
        void addLink(Node *node1, Node *node2);
        Node *getLowestFScore(std::vector<Node*> &list);
        void removeNode(std::vector<Node*> &list, Node *node);
        bool inList(std::vector<Node*> &list, Node *node);
        int dist(Node *start, Node *goal);
        std::vector<Node*> reconstruc_path(Node *current);
/*
        std::vector<std::pair<Coord, Coord>> geofence = loadGeofence("geofence.csv");

        std::vector<Node> nodes = loadMap("geofence.csv");
        connectNodes(nodes, geofence);

        std::vector<Node*> path = aStar(&nodes[0], &nodes[9]);
*/
};
