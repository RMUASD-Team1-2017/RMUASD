#include <iostream>
#include <vector>
#include <math.h>
#include <sstream>
#include <fstream>
#include <string>

#include "aed_gcs_logic/path_planner.h"

#ifdef SDL
#include <2D.hpp>
#endif

path_planner::path_planner(std::string map, std::string landingspotFile){
    loadMap(map);
    loadGeofence(map);
    loadLandingSpots(landingspotFile);
}

// Debugging function for printing all specs of a node
void path_planner::printNode(Node *node){
    std::cout << "Id: " << node->id << "\t";
    std::cout << "Coord: " << node->coord.x << ", " << node->coord.y << ", " << node->coord.z << "\t";
    if (node->cameFrom == NULL) std::cout << "From: Unknown!" << "\t";
    else std::cout << "From: " << node->cameFrom->id << "\t";
    std::cout << "G Score: " << node->gScore << "\t";
    std::cout << "F Score: " << node->fScore << "\t";
    std::cout << "Link:" << "\t";
    for (int i = 0; i < node->link.size(); i++){
        std::cout << "\t" << node->link[i]->id << "\t";
    }
    std::cout << std::endl;
}

// Debugging function for printing all specs of a coord
void path_planner::printCoord(Coord coord){
    std::cout << "Coord: " << coord.x << ", " << coord.y << ", " << coord.z << std::endl;
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

// Load comma-seperated file into list of nodes for the geofence
void path_planner::loadGeofence(std::string fileName){
    geofence = std::vector<std::pair<Coord, Coord>>();

    std::ifstream file(fileName);
    if (!file) std::cout << "Can't find geofence file: " << fileName << std::endl;

    int index = 0;
    Coord prev;

    while (file){
        std::string s;
        if (!getline(file, s)) break;
        std::istringstream ss(s);
        std::vector<double> coord;
        while (ss){
            std::string s;
            if (!getline(ss, s, ',')) break;
            coord.push_back(std::stod(s));
        }
        if (index) geofence.push_back(std::pair<Coord, Coord>(prev, Coord(coord[0], coord[1], coord[2])));
        prev = Coord(coord[0], coord[1], coord[2]);
        index++;
    }
}

// Load comma-seperated file into list of nodes for the map
void path_planner::loadMap(std::string fileName){
    nodes = std::vector<Node>();

    std::ifstream file(fileName);
    if (!file) std::cout << "Can't find map file: " << fileName << std::endl;

    int index = 0;
    while (file){
        std::string s;
        if (!getline(file, s)) break;
        std::istringstream ss(s);
        std::vector<double> coord;
        while (ss){
            std::string s;
            if (!getline(ss, s, ',')) break;
            coord.push_back(std::stod(s));
        }
        nodes.push_back(Node(Coord(coord[0], coord[1], coord[2]), index));
        index++;
    }

    for (int i = 1; i < nodes.size(); i++){
        nodes[i - 1].link.push_back(&nodes[i]);
        nodes[i].link.push_back(&nodes[i - 1]);
    }
}

void path_planner::loadLandingSpots(std::string fileName){
    std::ifstream file(fileName);
    if (!file) std::cout << "Can't find landingspot file: " << fileName << std::endl;

    int index = 0;
    while (file){
        std::string s;
        if (!getline(file, s)) break;
        std::istringstream ss(s);
        std::vector<double> coord;
        while (ss){
            std::string s;
            if (!getline(ss, s, ',')) break;
            coord.push_back(std::stod(s));
        }
        landingspot.push_back(Coord(coord[0], coord[1], coord[2]));
        index++;
    }
}

bool path_planner::intersection(std::pair<Coord, Coord> l1, std::pair<Coord, Coord> l2){
    double x1 = l1.first.x;
    double y1 = l1.first.y;
    double x2 = l1.second.x;
    double y2 = l1.second.y;
    double x3 = l2.first.x;
    double y3 = l2.first.y;
    double x4 = l2.second.x;
    double y4 = l2.second.y;
    double ta =
        ((y3 - y4) * (x1 - x3) + (x4 - x3) * (y1 - y3)) /
        ((x4 - x3) * (y1 - y2) - (x1 - x2) * (y4 - y3));
    double tb =
        ((y1 - y2) * (x1 - x3) + (x2 - x1) * (y1 - y3)) /
        ((x4 - x3) * (y1 - y2) - (x1 - x2) * (y4 - y3));
    return ta > 0 && ta < 1 && tb > 0 && tb < 1;
}

double path_planner::getAngle(Coord c1, Coord c2){
    double x = c2.x - c1.x;
    double y = c2.y - c1.y;
    return atan2(y, x);
}

int path_planner::getIndex(Coord coord){
    for (int i = 0; i < geofence.size(); i++){
        if (coord.x == geofence[i].first.x && coord.y == geofence[i].first.y && coord.z == geofence[i].first.z) return i;
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
                else if (k == geofence.size() - 1){
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
    return sqrt(pow(start->coord.x - goal->coord.x, 2) + pow(start->coord.y - goal->coord.y, 2) + pow(start->coord.z - goal->coord.z, 2));
}

// Return euclidean distance between two nodes
double path_planner::dist(Coord *start, Coord *goal){
    return sqrt(pow(start->x - goal->x, 2) + pow(start->y - goal->y, 2) + pow(start->z - goal->z, 2));
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

        // std::cout << "openSet" << std::endl;
        // printList(openSet);
        // std::cout << std::endl;
        // std::cout << std::endl;

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
            int tantative_gScore = current->gScore + dist(current, current->link[i]);
            if (tantative_gScore >= current->link[i]->gScore) continue;
            current->link[i]->cameFrom = current;
            current->link[i]->gScore = tantative_gScore;
            current->link[i]->fScore = tantative_gScore + dist(current->link[i], goal);
        }
    }

    return std::vector<Node*>();
}

Coord path_planner::getNearestLandingSpot(Coord start){
    Coord nearest;
    std::cout << "Size: " << landingspot.size() << std::endl;
    double distance = std::numeric_limits<double>::max();
    for (int i = 0; i < landingspot.size(); i++){
        std::cout << i << ": ";
        printCoord(landingspot[i]);
        if (dist(&start, &landingspot[i]) < distance){
            nearest = landingspot[i];
            distance = dist(&start, &landingspot[i]);
        }
    }
    printCoord(nearest);
    return nearest;
}

#ifdef SDL
void path_planner::draw(int size){

    static Window *window = new Window(Vector(1920 / 2, 0),Vector(size, size), "Path Planner");

    double minX = std::numeric_limits<double>::max();
    double maxX = std::numeric_limits<double>::min();
    double minY = std::numeric_limits<double>::max();
    double maxY = std::numeric_limits<double>::min();

    for (int i = 0; i < nodes.size(); i++){
        minX = std::min(minX, nodes[i].coord.x);
        maxX = std::max(maxX, nodes[i].coord.x);
        minY = std::min(minY, nodes[i].coord.y);
        maxY = std::max(maxY, nodes[i].coord.y);
    }

    double scale = size * 0.9 / std::max(maxX - minX, maxY - minY);
    Vector scaleOffset(-minX, -minY);

    window->background(White);
    Vector offset(size / 20, size / 20);

    for (int i = 0; i < nodes.size(); i++){
        Vector v1 = Vector(nodes[i].coord.x, nodes[i].coord.y);
        for (int j = 0; j < nodes[i].link.size(); j++){
            Vector v2 = Vector(nodes[i].link[j]->coord.x, nodes[i].link[j]->coord.y);
            window->line((v1 + scaleOffset) * scale + offset, (v2 + scaleOffset) * scale + offset, Green);
        }
    }

    for (int i = 0; i < geofence.size(); i++){
        Vector v1 = Vector(geofence[i].first.x, geofence[i].first.y);
        Vector v2 = Vector(geofence[i].second.x, geofence[i].second.y);
        window->line((v1 + scaleOffset) * scale + offset, (v2 + scaleOffset) * scale + offset, Red);
    }

    for (int i = 1; i < path.size(); i++){
        Vector v1 = Vector(path[i - 1]->coord.x, path[i - 1]->coord.y);
        Vector v2 = Vector(path[i]->coord.x, path[i]->coord.y);
        window->line((v1 + scaleOffset) * scale + offset, (v2 + scaleOffset) * scale + offset, Blue);
    }

    for (int i = 0; i < nodes.size(); i++){
        Vector v1 = Vector(nodes[i].coord.x, nodes[i].coord.y);
        window->text((v1 + scaleOffset) * scale + offset, nodes[i].id, Black);
    }

    window->draw();
}
#endif
