#include "cbs.h"
#include <queue>
#include <iostream>
#include <climits>

using namespace std;

pair<bool, std::pair<int, Position>> detect_conflict(const Path& p1, const Path& p2) {
    size_t max_len = max(p1.size(), p2.size());
    
    for (size_t t = 0; t < max_len; ++t) {
        Position a1 = p1[min(t, p1.size() - 1)];
        Position a2 = p2[min(t, p2.size() - 1)];

        // Vertex Conflict
        if (a1 == a2) {
            return {true, {(int)t, a1}};
        }

        // Edge Conflict (Swap)
        if (t > 0) {
            Position a1_prev = p1[min(t - 1, p1.size() - 1)];
            Position a2_prev = p2[min(t - 1, p2.size() - 1)];
            if (a1 == a2_prev && a2 == a1_prev) {
                return {true, {(int)t, a1}};
            }
        }
    }
    return {false, {-1, {-1, -1}}};
}

int SIC(const Solution& sol) {
    int cost = 0;
    for (const auto& kv : sol) {
        Path path = kv.second.second; // get the path
        Position goal = kv.second.first; // get the goal
        
        if (path.empty()) return INT_MAX;

        int i = path.size();
        // Remove padding from cost calculation
        while (i > 0 && path[i - 1] == goal) {
            i--;
        }
        // If path is just start->...->goal, i is the time steps (length - 1)
        // But if start==goal initially, cost is 0.  
        
        cost += max(0, i - 1); 
    }
    return cost;
}


ConflictInfo validate(const Solution& sol) {
    vector<int> agent_ids;
    for(auto const& [key, val] : sol) agent_ids.push_back(key);

    for (size_t i = 0; i < agent_ids.size(); ++i) {
        for (size_t j = i + 1; j < agent_ids.size(); ++j) {
            int id_i = agent_ids[i];
            int id_j = agent_ids[j];
            
            auto res = detect_conflict(sol.at(id_i).second, sol.at(id_j).second);
            if (res.first) {
                return {true, id_i, id_j, res.second.first, res.second.second};
            }
        }
    }
    return {false, -1, -1, -1, {-1, -1}};
}

struct CBSNode {
    int cost;
    Solution sol;
    map<int, set<Constraint>> constraints;

    // Min-heap for CBS open set
    bool operator>(const CBSNode& other) const {
        return cost > other.cost;
    }
};


Solution CBS(GridWorld& grid, vector<Agent>& agents, PlannerFunc planner) {
    map<int, set<Constraint>> root_constraints;
    Solution root_sol;

    // Initial Plans
    for (const auto& agent : agents) {
        Path p = planner(grid, agent.start, agent.goal, {});
        if (p.empty()) {
            cout << "Failed to find path for agent " << agent.id << endl;
            return {};
        }
        root_sol[agent.id] = {agent.goal, p};
    }

    int root_cost = SIC(root_sol);
    
    // Priority Queue
    priority_queue<CBSNode, vector<CBSNode>, greater<CBSNode>> open_set;
    open_set.push({root_cost, root_sol, root_constraints});

    int iterations = 0;

    while (!open_set.empty()) {
        CBSNode node = open_set.top();
        open_set.pop();

        iterations++;
        if (iterations % 100 == 0) {
            cout << "Iter: " << iterations 
                 << " | Cost: " << node.cost 
                 << " | Queue Size: " << open_set.size() << endl;
        }

        ConflictInfo conflict = validate(node.sol);

        if (!conflict.found) {
            return node.sol;
        }

        // Branching
        int ids[2] = {conflict.agent_i, conflict.agent_j};
        
        for (int id : ids) {
            map<int, set<Constraint>> new_constraints = node.constraints;
            new_constraints[id].insert({conflict.time, conflict.pos});

            int current_agent_idx = -1;
            // Find agent object for this ID
            for(int k=0; k<agents.size(); k++) if(agents[k].id == id) current_agent_idx = k;

            Path p = planner(grid, agents[current_agent_idx].start, agents[current_agent_idx].goal, new_constraints[id]);
            
            if (!p.empty()) {
                Solution new_sol = node.sol;
                new_sol[id] = {agents[current_agent_idx].goal, p};
                
                int new_cost = SIC(new_sol);
                if (new_cost != INT_MAX) {
                    open_set.push({new_cost, new_sol, new_constraints});
                }
            }
        }
    }
    
    return {};
}


void print_paths(const Solution& sol) {
    if (sol.empty()) return;

    size_t max_time = 0;
    for (auto const& [id, val] : sol) {
        max_time = max(max_time, val.second.size());
    }

    for (size_t t = 0; t < max_time; ++t) {
        cout << "Time " << t << endl;
        vector<string> grid_display(8, string(16, '.')); // simple 8x8 viz
        
        // Prepare grid string (adding spaces for readability)
        for(int r=0; r<8; ++r) grid_display[r] = ". . . . . . . . ";

        for (auto const& [id, val] : sol) {
            Path p = val.second;
            Position pos = p[min(t, p.size() - 1)];
            
            // Map 8x8 to string indices (col * 2)
            if (pos.first < 8 && pos.second < 8) {
                 grid_display[pos.first][pos.second * 2] = '0' + id;
            }
        }

        for (const auto& row : grid_display) {
            cout << row << endl;
        }
        cout << endl;
    }
}