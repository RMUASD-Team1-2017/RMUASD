#pragma once

// Includes
#include <limits>
#include <vector>
#include <string>
#include "sensor_msgs/NavSatFix.h"

#ifdef SDL
#include <2D.hpp>
#endif

struct UTM{
    double x;
    double y;
};

struct GeoCoord{
    double latitude;
    double longitude;
    bool operator==(const GeoCoord& other)
    {
      return (this->latitude == other.latitude and this->longitude == other.longitude);
    }
};

struct Coord{
    Coord(double latitude, double longitude);
    UTM utm;
    GeoCoord geo;
};

// Node inside the graph
struct Node{
    Node(Coord _coord, int _id)
    : coord(_coord), id(_id)
    {
        id = _id;
        cameFrom = nullptr;
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

        path_planner(std::string geofence, std::string inner_geofence, std::string landingspotFile);
        void loadGeofence(std::string fileName);
        void loadMap(std::string fileName);
        void connectNodes();
        std::vector<Node*> aStar(Coord start, Coord goal);
        Coord getNearestLandingSpot(Coord goal);
        bool insideGeoFence(Coord *n);
        static void printNode(Node *node);
        static void printList(std::vector<Node*> &list);
        static void printList(std::vector<Node> &list);
        static void printCoord(Coord coord);

#ifdef SDL
        void draw(int size);
#endif

        std::vector<std::pair<Coord, Coord>> geofence;
        std::vector<Node> nodes;
        std::vector<Coord> landingspot;
        std::vector<Node*> path;

    private:

        void generateUTMCoords();

        bool insideGeoFence(Node *node1, Node *node2);
        int getIndex(Coord coord);

        void loadLandingSpots(std::string flieName);
        bool intersection(std::pair<Coord, Coord> l1, std::pair<Coord, Coord> l2);
        double getAngle(Coord c1, Coord c2);
        void addLink(Node *node1, Node *node2);
        Node *getLowestFScore(std::vector<Node*> &list);
        void removeNode(std::vector<Node*> &list, Node *node);
        bool inList(std::vector<Node*> &list, Node *node);
        double dist(Node *start, Node *goal);
        double dist(Coord *start, Coord *goal);
        std::vector<Node*> reconstruc_path(Node *current);
};
