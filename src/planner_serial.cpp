#include "planner.h"

#include <queue>
#include <set>
#include <limits>
#include <map>
#include <queue>
#include <cmath>
#include <algorithm>
#include <tuple>
#include <climits>
#include <chrono>
using namespace std;


struct AStarNode {
    int f_score;
    int g_score; // time
    Position current;
    Path path;

    // Priority Queue needs to be a Min-Heap, so we invert the operator
    bool operator>(const AStarNode& other) const {
        return f_score > other.f_score;
    }
};

Path plan_path_serial(const GridWorld& grid, Position start, Position goal, const set<Constraint>& constraints) {

    int latest_constraint_time = 0;
    for (const auto& c : constraints) {
        latest_constraint_time = max(latest_constraint_time, c.first);
    }

    priority_queue<AStarNode, vector<AStarNode>, greater<AStarNode>> open_set;
    
    Path initial_path;
    initial_path.push_back(start);
    open_set.push({manhattan(start, goal), 0, start, initial_path});

    set<pair<Position, int>> visited; // (position, time)

    while (!open_set.empty()) {
        AStarNode node = open_set.top();
        open_set.pop();

        if (visited.find({node.current, node.g_score}) != visited.end()) {
            continue;
        }
        visited.insert({node.current, node.g_score});

        if (node.current == goal && node.g_score >= latest_constraint_time) {
            return node.path;
        }

        int next_time = node.g_score + 1;
        vector<Position> nbrs = grid.neighbors(node.current);

        for (const auto& neighbor : nbrs) {
            // Check constraints
            if (constraints.find({next_time, neighbor}) != constraints.end()) {
                continue;
            }

            Path new_path = node.path;
            new_path.push_back(neighbor);
            
            int f = (int)new_path.size() + manhattan(neighbor, goal);
            open_set.push({f, next_time, neighbor, new_path});
        }
    }
    return {}; // Empty path if fail
}