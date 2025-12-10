#pragma once
#include "grid.h"
#include <set>

// function pointer type for any low-level SAPF planner
using PlannerFunc =
    Path(*)(const GridWorld& grid,
            Position start,
            Position goal,
            const std::set<Constraint>& constraints);

// serial A* version
Path plan_path_serial(const GridWorld& grid,
                      Position start,
                      Position goal,
                      const std::set<Constraint>& constraints);

// GPU BFS version (later in .cu)
Path plan_path_gpu_bfs(const GridWorld& grid,
                       Position start,
                       Position goal,
                       const std::set<Constraint>& constraints);
