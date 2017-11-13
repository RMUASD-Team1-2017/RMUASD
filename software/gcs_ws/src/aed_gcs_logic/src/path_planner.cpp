#include <iostream>
#include <vector>
#include <math.h>
#include <sstream>
#include <fstream>
#include <string>
#include <exception>
#include "aed_gcs_logic/json.hpp"
#include "aed_gcs_logic/path_planner.h"
#include <GeographicLib/GeoCoords.hpp>

#ifdef SDL
#include <2D.hpp>
#endif

path_planner::path_planner(std::string geo_fence, std::string inner_geofence, std::string landingspotFile){
    loadMap(geo_fence);
    loadGeofence(inner_geofence);
    loadLandingSpots(landingspotFile);
}

// Debugging function for printing all specs of a node
void path_planner::printNode(Node *node){
    std::cout << "Id: " << node->id << "\t";
    std::cout << "Geo coord: " << node->coord.geo.latitude << ", " << node->coord.geo.longitude << "\t";
    std::cout << "UTM coord: " << node->coord.utm.x << ", " << node->coord.utm.y << "\t";
    if (node->cameFrom == NULL) std::cout << "From: Unknown!" << "\t";
    else std::cout << "From: " << node->cameFrom->id << "\t";
    std::cout << "G Score: " << node->gScore << "\t";
    std::cout << "F Score: " << node->fScore << "\t";
    // std::cout << "Link:" << "\t";
    // for (int i = 0; i < node->link.size(); i++){
    //     std::cout << "\t" << node->link[i]->id << "\t";
    // }
    std::cout << std::endl;
}

// Debugging function for printing all specs of a coord
void path_planner::printCoord(Coord coord){
    std::cout << "Coord: " << coord.geo.latitude << ", " << coord.geo.longitude << std::endl;
    std::cout << std::endl;
}

// Debugging function for printing all nodes in a list
void path_planner::printList(std::vector<Node*> &list){
    for (int i = 0; i < list.size(); i++) printNode(list[i]);
}

// Debugging function for printing all nodes in a list
void path_planner::printList(std::vector<Node> &list){
    for (int i = 0; i < list.size(); i++) printNode(&list[i]);
}

UTM path_planner::geo2utm(GeoCoord &geo){
    GeographicLib::GeoCoords geoCoords(geo.latitude, geo.longitude);
    UTM utm;
    utm.x = geoCoords.Easting();
    utm.y = geoCoords.Northing();
    return utm;
}

void path_planner::generateUTMCoords(){
    for (int i = 0; i < geofence.size(); i++){
        geofence[i].first.utm = geo2utm(geofence[i].first.geo);
    }
}

// Load comma-seperated file into list of nodes for the geofence
void path_planner::loadGeofence(std::string fileName){
    geofence = std::vector<std::pair<Coord, Coord>>();

    std::ifstream file(fileName);
    if (!file) std::cout << "Can't find geofence file: " << fileName << std::endl;

    nlohmann::json j;

    file >> j;

    Coord prev;

    for (int i = 0; i < j["polygon"].size(); i++){
        Coord tempCoord;
        tempCoord.geo.latitude = j["polygon"][i][0];
        tempCoord.geo.longitude = j["polygon"][i][1];
        if (i) geofence.push_back(std::pair<Coord, Coord>(prev, tempCoord));
        prev = tempCoord;
    }

    Coord tempCoord;
    tempCoord.geo.latitude = j["polygon"][0][0];
    tempCoord.geo.longitude = j["polygon"][0][1];
    geofence.push_back(std::pair<Coord, Coord>(prev, tempCoord));
}

// Load comma-seperated file into list of nodes for the map
void path_planner::loadMap(std::string fileName){
    nodes = std::vector<Node>();

    std::ifstream file(fileName);
    if (!file) std::cout << "Can't find map file: " << fileName << std::endl;

    nlohmann::json j;

    file >> j;

    for (int i = 0; i < j["polygon"].size(); i++){
        Coord tempCoord;
        tempCoord.geo.latitude = j["polygon"][i][0];
        tempCoord.geo.longitude = j["polygon"][i][1];
        nodes.push_back(Node(tempCoord, i));
    }

    for (int i = 1; i < nodes.size(); i++){
        nodes[i - 1].link.push_back(&nodes[i]);
        nodes[i].link.push_back(&nodes[i - 1]);
    }
}

void path_planner::loadLandingSpots(std::string fileName){

    std::ifstream file(fileName);
    if (!file) std::cout << "Can't find landingspot file: " << fileName << std::endl;

    nlohmann::json j;

    file >> j;

    std::cout << j["points"].size() << std::endl;

    for (int i = 0; i < j["points"].size(); i++){
        std::cout << i << ", " << j["points"].size() << std::endl;
        Coord tempCoord;
        tempCoord.geo.latitude = j["points"][i][0];
        tempCoord.geo.longitude = j["points"][i][1];
        landingspot.push_back(tempCoord);
    }
}

bool path_planner::intersection(std::pair<Coord, Coord> l1, std::pair<Coord, Coord> l2){
    double x1 = l1.first.geo.latitude;
    double y1 = l1.first.geo.longitude;
    double x2 = l1.second.geo.latitude;
    double y2 = l1.second.geo.longitude;
    double x3 = l2.first.geo.latitude;
    double y3 = l2.first.geo.longitude;
    double x4 = l2.second.geo.latitude;
    double y4 = l2.second.geo.longitude;
    double ta =
        ((y3 - y4) * (x1 - x3) + (x4 - x3) * (y1 - y3)) /
        ((x4 - x3) * (y1 - y2) - (x1 - x2) * (y4 - y3));
    double tb =
        ((y1 - y2) * (x1 - x3) + (x2 - x1) * (y1 - y3)) /
        ((x4 - x3) * (y1 - y2) - (x1 - x2) * (y4 - y3));
    return ta > 0 && ta < 1 && tb > 0 && tb < 1;
}

double path_planner::getAngle(Coord c1, Coord c2){
    double x = c2.geo.longitude - c1.geo.longitude;
    double y = c2.geo.latitude - c1.geo.latitude;
    return atan2(y, x);
}

int path_planner::getIndex(Coord coord){
    for (int i = 0; i < geofence.size(); i++){
        if (coord.geo.latitude == geofence[i].first.geo.latitude && coord.geo.longitude == geofence[i].first.geo.longitude) return i;
    }
    return -1;
}

bool path_planner::outOfBounds(Node *node1, Node *node2){

    int index = getIndex(node1->coord);
    if (index == -1 || index == 0) return false;

    double nodeAngle = getAngle(node1->coord, node2->coord);
    double geoFence0 = getAngle(geofence[index - 1].second, geofence[index - 1].first);
    double geoFence1 = getAngle(geofence[index].first, geofence[index].second);

    nodeAngle += M_PI * 2 - geoFence0;
    if (nodeAngle > M_PI * 2) nodeAngle -= M_PI * 2;

    geoFence1 += M_PI * 2 - geoFence0;
    if (geoFence1 > M_PI * 2) geoFence1 -= M_PI * 2;

    geoFence0 = 0;

    return geoFence1 > nodeAngle;
}

void path_planner::addLink(Node *node1, Node *node2){
    node1->link.push_back(node2);
    node2->link.push_back(node1);
}

void path_planner::connectNodes(){
    for (int i = 0; i < nodes.size(); i++){
        for (int j = 0; j < i; j++){
            if (i == j) continue;
            if (outOfBounds(&nodes[i], &nodes[j]) || outOfBounds(&nodes[j], &nodes[i])) continue;
            for (int k = 0; k < geofence.size(); k++){
                if (intersection(std::pair<Coord, Coord>(nodes[i].coord, nodes[j].coord), geofence[k])) break;
                if (k == geofence.size() - 1){
                    addLink(&nodes[i], &nodes[j]);
                }
            }
        }
    }
}

// Finds the node with the lowest F score
Node *path_planner::getLowestFScore(std::vector<Node*> &list){
    double score = std::numeric_limits<double>::max();
    unsigned long index = 0;
    for (int i = 0; i < list.size(); i++){
        if (score > list[i]->fScore){
            score = list[i]->fScore;
            index = i;
        }
    }
    if (score == std::numeric_limits<double>::max()) return NULL;
    else return list[index];
}

// Remove specific node from list
void path_planner::removeNode(std::vector<Node*> &list, Node *node){
    for (int i = 0; i < list.size(); i++){
        if (node == list[i]){
            list.erase(list.begin() + i);
            break;
        }
    }
}

// Determine if the node is represented in the list
bool path_planner::inList(std::vector<Node*> &list, Node *node){
    for (int i = 0; i < list.size(); i++){
        if (node == list[i]) return true;
    }
    return false;
}

// Return euclidean distance between two nodes
double path_planner::dist(Node *start, Node *goal){
    return sqrt(pow(start->coord.utm.x - goal->coord.utm.x, 2) + pow(start->coord.utm.y - goal->coord.utm.y, 2));
}

// Return euclidean distance between two nodes
double path_planner::dist(Coord *start, Coord *goal){
    return sqrt(pow(start->utm.x - goal->utm.x, 2) + pow(start->utm.y - goal->utm.y, 2));
}

// Recunstruct path after A*-algorithm
std::vector<Node*> path_planner::reconstruc_path(Node *current){
    std::vector<Node*> path = {current};
    while (current->cameFrom != NULL){
        current = current->cameFrom;
        path.push_back(current);
    }
    return path;
}

// The standard A* algorithm
std::vector<Node*> path_planner::aStar(Coord startCoord, Coord goalCoord){

    startCoord.utm = geo2utm(startCoord.geo);
    goalCoord.utm = geo2utm(goalCoord.geo);

    nodes.push_back(Node(startCoord, -1));
    Node *start = &nodes.back();


    nodes.push_back(Node(goalCoord, -2));
    Node *goal = &nodes.back();

    connectNodes();

    std::vector<Node*> closedSet;
    std::vector<Node*> openSet = {start};
    start->gScore = 0;
    start->fScore = dist(start, goal);



    while (openSet.size()){

        Node *current = getLowestFScore(openSet);
        if (current == goal){
            path = reconstruc_path(current);
            return path;
        }

        removeNode(openSet, current);
        closedSet.push_back(current);

        for (int i = 0; i < current->link.size(); i++){
            if (inList(closedSet, current->link[i])) continue;
            if (!inList(openSet, current->link[i])) openSet.push_back(current->link[i]);
            double tantative_gScore = current->gScore + dist(current, current->link[i]);
            if (tantative_gScore >= current->link[i]->gScore) continue;
            current->link[i]->cameFrom = current;
            current->link[i]->gScore = tantative_gScore;
            current->link[i]->fScore = tantative_gScore + dist(current->link[i], goal);
        }
    }

    return std::vector<Node*>();
}

Coord path_planner::getNearestLandingSpot(Coord goal){
    target = goal;
    Coord nearest;
    double distance = std::numeric_limits<double>::max();
    for (int i = 0; i < landingspot.size(); i++){
        if (dist(&goal, &landingspot[i]) < distance){
            nearest = landingspot[i];
            distance = dist(&goal, &landingspot[i]);
        }
    }
    return nearest;
}

#ifdef SDL
void path_planner::draw(int size){

    double minX = std::numeric_limits<double>::max();
    double maxX = std::numeric_limits<double>::min();
    double minY = std::numeric_limits<double>::max();
    double maxY = std::numeric_limits<double>::min();

    for (int i = 0; i < nodes.size(); i++){
        minX = std::min(minX, nodes[i].coord.geo.longitude);
        maxX = std::max(maxX, nodes[i].coord.geo.longitude);
        minY = std::min(minY, nodes[i].coord.geo.latitude);
        maxY = std::max(maxY, nodes[i].coord.geo.latitude);
    }

    double scale = size * 0.9 / std::max(maxX - minX, maxY - minY);

    int height = size / 10 + (maxY - minY) * scale;
    int width = size / 10 + (maxX - minX) * scale;

    Vector scaleOffset(-minX, -minY);

    static Window *window = new Window(Vector(1920 / 2, 0),Vector(width, height), "Path Planner");

    window->background(Black);
    Vector offset(size / 20, size / 20);

    Vector v1 = (Vector(target.geo.longitude, target.geo.latitude) + scaleOffset) * scale + offset;
    window->circle(Vector(v1.x, height - v1.y), 10, Black, Cyan);

    for (int i = 0; i < landingspot.size(); i++){
        Vector v1 = (Vector(landingspot[i].geo.longitude, landingspot[i].geo.latitude) + scaleOffset) * scale + offset;
        window->circle(Vector(v1.x, height - v1.y), 10, Black, Yellow);
    }

    for (int i = 0; i < nodes.size(); i++){
        Vector v1 = Vector(nodes[i].coord.geo.longitude, nodes[i].coord.geo.latitude);
        for (int j = 0; j < nodes[i].link.size(); j++){
            Vector v2 = Vector(nodes[i].link[j]->coord.geo.longitude, nodes[i].link[j]->coord.geo.latitude);
            Vector v3 = (v1 + scaleOffset) * scale + offset;
            Vector v4 = (v2 + scaleOffset) * scale + offset;
            if (v3.y < 0 || v4.y < 0);// std::cout << "Error " << i << ", " << j << std::endl;
            else window->line(Vector(v3.x, height - v3.y), Vector(v4.x, height - v4.y), Green);
        }
    }

    for (int i = 0; i < geofence.size(); i++){
        Vector v1 = Vector(geofence[i].first.geo.longitude, geofence[i].first.geo.latitude);
        Vector v2 = Vector(geofence[i].second.geo.longitude, geofence[i].second.geo.latitude);
        Vector v3 = (v1 + scaleOffset) * scale + offset;
        Vector v4 = (v2 + scaleOffset) * scale + offset;
        window->line(Vector(v3.x, height - v3.y), Vector(v4.x, height - v4.y), Red);
    }

    for (int i = 1; i < path.size(); i++){
        Vector v1 = Vector(path[i - 1]->coord.geo.longitude, path[i - 1]->coord.geo.latitude);
        Vector v2 = Vector(path[i]->coord.geo.longitude, path[i]->coord.geo.latitude);
        Vector v3 = (v1 + scaleOffset) * scale + offset;
        Vector v4 = (v2 + scaleOffset) * scale + offset;
        window->line(Vector(v3.x, height - v3.y), Vector(v4.x, height - v4.y), Blue);
    }

    for (int i = 0; i < nodes.size(); i++){
        Vector v1 = (Vector(nodes[i].coord.geo.longitude, nodes[i].coord.geo.latitude) + scaleOffset) * scale + offset;
        window->text(Vector(v1.x, height - v1.y), nodes[i].id, White);
    }

    window->draw();
}
#endif
