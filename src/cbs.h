#pragma once
#include "grid.h"
#include "planner.h"

struct ConflictInfo {
    bool found;
    int agent_i;
    int agent_j;
    int time;
    Position pos;
};

Solution CBS(GridWorld& grid,
             std::vector<Agent>& agents,
             PlannerFunc planner);

void print_paths(const Solution& sol);

bool check_solution(const GridWorld& grid, const std::vector<Agent>& agents, const Solution& sol);
