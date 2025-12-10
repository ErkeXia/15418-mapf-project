#pragma once
#include <vector>
#include <set>
#include <map>
#include <utility>
#include <string>

using Position = std::pair<int,int>;
using Path = std::vector<Position>;
using Constraint = std::pair<int, Position>; // (time, cell)
using AgentSolution = std::pair<Position, Path>;
using Solution = std::map<int, AgentSolution>;

struct Agent {
    int id;
    Position start;
    Position goal;
};

class GridWorld {
public:
    int width;
    int height;
    std::set<Position> obstacles;

    GridWorld(int w=0, int h=0, std::set<Position> obs = {})
        : width(w), height(h), obstacles(std::move(obs)) {}

    bool in_bounds(Position pos) const;
    bool passable(Position pos) const;
    std::vector<Position> neighbors(Position pos) const;
};

int manhattan(Position p1, Position p2);

// scenario loading
bool load_scenario(const std::string& filename,
                   GridWorld& grid,
                   std::vector<Agent>& agents);
