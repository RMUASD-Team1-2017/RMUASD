#include <iostream>
#include <vector>
#include <math.h>
#include <sstream>
#include <fstream>
#include <string>

#ifdef SDL
#include <2D.hpp>
#endif

#define ULONG_MAX 4294967295

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

// Debugging function for printing all specs of a node
void printNode(Node *node){
    std::cout << "Id: " << node->id << "\t";
    std::cout << "Coord: " << node->coord.x << ", " << node->coord.y << "\t";
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

// Debugging function for printing all nodes in a list
void printList(std::vector<Node*> &list){
    for (int i = 0; i < list.size(); i++) printNode(list[i]);
}

// Load comma-seperated file into list of nodes for the geofence
std::vector<std::pair<Coord, Coord>> loadGeofence(std::string fileName){
    std::vector<std::pair<Coord, Coord>> list;

    std::ifstream file(fileName);

    int index = 0;
    Coord prev;

    while (file){
        std::string s;
        if (!getline(file, s)) break;
        std::istringstream ss(s);
        std::vector<std::string> coord;
        while (ss){
            std::string s;
            if (!getline(ss, s, ',')) break;
            coord.push_back(s);
        }
        if (index) list.push_back(std::pair<Coord, Coord>(prev, Coord(std::stoi(coord[0]), std::stoi(coord[1]))));
        prev = Coord(std::stoi(coord[0]), std::stoi(coord[1]));
        index++;
    }

    return list;
}

// Load comma-seperated file into list of nodes for the map
std::vector<Node> loadMap(std::string fileName){
    std::vector<Node> list;

    std::ifstream file(fileName);

    int index = 0;

    while (file){
        std::string s;
        if (!getline(file, s)) break;
        std::istringstream ss(s);
        std::vector<std::string> coord;
        while (ss){
            std::string s;
            if (!getline(ss, s, ',')) break;
            coord.push_back(s);
        }
        list.push_back(Node(Coord(std::stoi(coord[0]), std::stoi(coord[1])), index));
        index++;
    }

    for (int i = 1; i < list.size(); i++){
        list[i - 1].link.push_back(&list[i]);
        list[i].link.push_back(&list[i - 1]);
    }

    return list;
}

bool intersection(std::pair<Coord, Coord> l1, std::pair<Coord, Coord> l2){
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

double getAngle(Coord &c1, Coord &c2){
    double x = c1.x - c2.x;
    double y = c1.y - c2.y;
	if (y > 0){
		if (x > 0) return atan(y / x);
		else if (x < 0) return M_PI / 2 - atan(x / y);
		else return M_PI / 2;
	}
	else if (y < 0){
		if (x > 0) return M_PI / 2 * 3 - atan(x / y);
		else if (x < 0) return M_PI + atan(y / x);
		else return M_PI / 2 * 3;
	}
	else{
		if (x > 0) return 0;
		else if (x < 0) return M_PI;
		else return 0;
	}
}

int getIndex(Coord coord, std::vector<std::pair<Coord, Coord>> &geofence){
    for (int i = 0; i < geofence.size(); i++){
        if (coord.x == geofence[i].first.x && coord.y == geofence[i].first.y) return i;
    }
    return -1;
}

bool outOfBounds(Node *node1, Node *node2, std::vector<std::pair<Coord, Coord>> &geofence){
    return false;
}

void addLink(Node *node1, Node *node2){
    node1->link.push_back(node2);
    node2->link.push_back(node1);
}

void connectNodes(std::vector<Node> &nodes, std::vector<std::pair<Coord, Coord>> &geofence){
    for (int i = 0; i < nodes.size(); i++){
        for (int j = 0; j < nodes.size(); j++){
            for (int k = 0; k < geofence.size(); k++){
                if (intersection(std::pair<Coord, Coord>(nodes[i].coord, nodes[j].coord), geofence[k]) || outOfBounds(&nodes[i], &nodes[j], geofence)) break;
                else if (k == geofence.size() - 1){
                    addLink(&nodes[i], &nodes[j]);
                }
            }
        }
    }
}

// Finds the node with the lowest F score
Node *getLowestFScore(std::vector<Node*> &list){
    unsigned long score = ULONG_MAX;
    unsigned long index = 0;
    for (int i = 0; i < list.size(); i++){
        if (score > list[i]->fScore){
            score = list[i]->fScore;
            index = i;
        }
    }
    if (score == ULONG_MAX) return NULL;
    else return list[index];
}

// Remove specific node from list
void removeNode(std::vector<Node*> &list, Node *node){
    for (int i = 0; i < list.size(); i++){
        if (node == list[i]){
            list.erase(list.begin() + i);
            break;
        }
    }
}

// Determine if the node is represented in the list
bool inList(std::vector<Node*> &list, Node *node){
    for (int i = 0; i < list.size(); i++){
        if (node == list[i]) return true;
    }
    return false;
}

// Return euclidean distance between two nodes
int dist(Node *start, Node *goal){
    return sqrt(pow(start->coord.x - goal->coord.x, 2) + pow(start->coord.y - goal->coord.y, 2));
}

// Recunstruct path after A*-algorithm
std::vector<Node*> reconstruc_path(Node *current){
    std::vector<Node*> path = {current};
    while (current->cameFrom != NULL){
        current = current->cameFrom;
        path.push_back(current);
    }
    return path;
}

// The standard A* algorithm
std::vector<Node*> aStar(Node *start, Node *goal){
    std::vector<Node*> closedSet;
    std::vector<Node*> openSet = {start};
    start->gScore = 0;
    start->fScore = dist(start, goal);

    while (openSet.size()){
/*
        std::cout << "openSet" << std::endl;
        printList(openSet);
        std::cout << std::endl;
        std::cout << std::endl;
*/
        Node *current = getLowestFScore(openSet);
        if (current == goal){
            return reconstruc_path(current);
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

int main(){

    std::vector<std::pair<Coord, Coord>> geofence = loadGeofence("geofence.csv");

    std::vector<Node> nodes = loadMap("geofence.csv");
    connectNodes(nodes, geofence);

    std::vector<Node*> path = aStar(&nodes[0], &nodes[9]);

#ifdef SDL
    Window window(WINDOWED, Vector(500, 500), "Path Planner");

    bool quit = false;

    Coord offset(50, 50);

    while (!quit){
		SDL_Event event;
		while (SDL_PollEvent(&event)){
			if (event.type == SDL_KEYDOWN) quit = true;
		}
        window.background(White);
        for (int i = 0; i < nodes.size(); i++){
            Coord c1 = nodes[i].coord;
            for (int j = 0; j < nodes[i].link.size(); j++){
                Coord c2 = nodes[i].link[j]->coord;
                window.line(Vector(c1.x + offset.x, c1.y + offset.y), Vector(c2.x + offset.x, c2.y + offset.y), Green);
            }
        }
        for (int i = 0; i < geofence.size(); i++){
            Coord c1 = geofence[i].first;
            Coord c2 = geofence[i].second;
            window.line(Vector(c1.x + offset.x, c1.y + offset.y), Vector(c2.x + offset.x, c2.y + offset.y), Red);
        }
        for (int i = 1; i < path.size(); i++){
            Coord c1 = path[i - 1]->coord;
            Coord c2 = path[i]->coord;
            window.line(Vector(c1.x + offset.x, c1.y + offset.y), Vector(c2.x + offset.x, c2.y + offset.y), Blue);
        }
        for (int i = 0; i < nodes.size(); i++){
            window.text(Vector(nodes[i].coord.x + offset.x, nodes[i].coord.y + offset.y), nodes[i].id, Black);
        }
        window.draw();
    }
#endif

    return 0;
}
