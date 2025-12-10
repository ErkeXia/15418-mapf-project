#include "grid.h"
#include <iostream>
#include <fstream>
#include <cmath>
#include <vector>
using namespace std;

static const std::vector<Position> DIRS = {
    {0,0}, {-1,0}, {1,0}, {0,-1}, {0,1}
};

bool GridWorld::in_bounds(Position pos) const {
    return pos.first >= 0 && pos.first < height && pos.second >= 0 && pos.second < width;
}

bool GridWorld::passable(Position pos) const {
    return obstacles.find(pos) == obstacles.end();
}

vector<Position> GridWorld::neighbors(Position pos) const {
    vector<Position> results;
    for (const auto& d : DIRS) {
        Position n = {pos.first + d.first, pos.second + d.second};
        if (in_bounds(n) && passable(n)) {
            results.push_back(n);
        }
    }
    return results;
}

int manhattan(Position p1, Position p2) {
    return abs(p1.first - p2.first) + abs(p1.second - p2.second);
}

bool load_scenario(const string& filename, GridWorld& grid, vector<Agent>& agents) {
    ifstream infile(filename);
    if (!infile.is_open()) {
        cerr << "Error: Could not open " << filename << endl;
        return false;
    }

    int height, width;
    infile >> height >> width;

    // Load Obstacles
    int num_obstacles;
    infile >> num_obstacles;
    set<Position> obstacles;
    for (int i = 0; i < num_obstacles; ++i) {
        int r, c;
        infile >> r >> c;
        obstacles.insert({r, c});
    }

    grid = GridWorld(width, height, obstacles);

    // Load Agents
    int num_agents;
    infile >> num_agents;
    for (int i = 0; i < num_agents; ++i) {
        int id, sr, sc, gr, gc;
        infile >> id >> sr >> sc >> gr >> gc;
        
        Agent new_agent;
        new_agent.id = id;
        new_agent.start = {sr, sc};
        new_agent.goal = {gr, gc};
        agents.push_back(new_agent);
    }

    return true;
}