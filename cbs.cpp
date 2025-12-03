#include <iostream>
#include <fstream>
#include <vector>
#include <set>
#include <map>
#include <queue>
#include <cmath>
#include <algorithm>
#include <tuple>
#include <climits>
#include <yaml-cpp/yaml.h>


using namespace std;

// --- Type Definitions ---
using Position = pair<int, int>;
using Path = vector<Position>;
using Constraint = pair<int, Position>; // (time, cell)
using AgentSolution = pair<Position, Path>; // (Goal, Path)
using Solution = map<int, AgentSolution>; // AgentID -> (Goal, Path)

// Directions: stay, up, down, left, right
const vector<Position> DIRS = {{0, 0}, {-1, 0}, {1, 0}, {0, -1}, {0, 1}};

// --- Structures ---

struct Agent {
    int id;
    Position start;
    Position goal;
};

class GridWorld {
public:
    int width;
    int height;
    set<Position> obstacles;

    GridWorld(int w, int h, set<Position> obs) : width(w), height(h), obstacles(obs) {}

    bool in_bounds(Position pos) const {
        return pos.first >= 0 && pos.first < height && pos.second >= 0 && pos.second < width;
    }

    bool passable(Position pos) const {
        return obstacles.find(pos) == obstacles.end();
    }

    vector<Position> neighbors(Position pos) const {
        vector<Position> results;
        for (const auto& d : DIRS) {
            Position n = {pos.first + d.first, pos.second + d.second};
            if (in_bounds(n) && passable(n)) {
                results.push_back(n);
            }
        }
        return results;
    }
};

// Helper for Manhattan distance
int manhattan(Position p1, Position p2) {
    return abs(p1.first - p2.first) + abs(p1.second - p2.second);
}

// --- A* Low Level Search ---

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

Path plan_path(const GridWorld& grid, Position start, Position goal, const set<Constraint>& constraints) {
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

        if (node.current == goal) {
            // Pad to avoid early disappear (matching Python logic)
            Path final_path = node.path;
            for(int i=0; i<5; ++i) final_path.push_back(goal);
            return final_path;
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

// --- Conflict Detection ---

// Returns: {time, position} or nullopt
pair<bool, pair<int, Position>> detect_conflict(const Path& p1, const Path& p2) {
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
        // Using logic: cost += max(0, i - 1) based on Python snippet
        // However, python snippet logic `path[i-1] == pos` checks against 'pos' which was stored in tuple
        // In python: Root_sol[i] = (agent.goal, path). 
        
        cost += max(0, i - 1); 
    }
    return cost;
}

// Returns: (agent_i, agent_j, time, position)
struct ConflictInfo {
    bool found;
    int agent_i;
    int agent_j;
    int time;
    Position pos;
};

ConflictInfo validate(const Solution& sol) {
    // Convert map to vector for indexed access to match python loop style
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

// --- CBS High Level Search ---

struct CBSNode {
    int cost;
    Solution sol;
    map<int, set<Constraint>> constraints;

    // Min-heap for CBS open set
    bool operator>(const CBSNode& other) const {
        return cost > other.cost;
    }
};

Solution CBS(GridWorld& grid, vector<Agent>& agents) {
    map<int, set<Constraint>> root_constraints;
    Solution root_sol;

    // Initial Plans
    for (const auto& agent : agents) {
        Path p = plan_path(grid, agent.start, agent.goal, {});
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

    while (!open_set.empty()) {
        CBSNode node = open_set.top();
        open_set.pop();

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

            Path p = plan_path(grid, agents[current_agent_idx].start, agents[current_agent_idx].goal, new_constraints[id]);
            
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

// --- Visualization (Text) ---

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

bool load_scenario(const string& filename, GridWorld& grid, vector<Agent>& agents) {
    try {
        YAML::Node config = YAML::LoadFile(filename);

        // 1. Load Dimensions
        int height = config["map"]["dimensions"][0].as<int>();
        int width = config["map"]["dimensions"][1].as<int>();

        // 2. Load Obstacles
        set<Position> obstacles;
        for (const auto& node : config["map"]["obstacles"]) {
            // YAML format is [row, col]
            int r = node[0].as<int>();
            int c = node[1].as<int>();
            obstacles.insert({r, c});
        }

        // Initialize GridWorld
        // Note: We are assigning to the passed 'grid' object
        grid = GridWorld(width, height, obstacles);

        // 3. Load Agents
        int id_counter = 0;
        for (const auto& agent_node : config["agents"]) {
            int start_r = agent_node["start"][0].as<int>();
            int start_c = agent_node["start"][1].as<int>();
            
            int goal_r = agent_node["goal"][0].as<int>();
            int goal_c = agent_node["goal"][1].as<int>();

            // Create Agent and add to vector
            Agent new_agent;
            new_agent.id = id_counter++;
            new_agent.start = {start_r, start_c};
            new_agent.goal = {goal_r, goal_c};
            // Optional: Store name if your Agent struct supports it
            // new_agent.name = agent_node["name"].as<string>(); 

            agents.push_back(new_agent);
        }

        return true;

    } catch (const YAML::Exception& e) {
        cerr << "Error loading YAML: " << e.what() << endl;
        return false;
    }
}

int main() {
    // Placeholder objects
    set<Position> empty_obs;
    GridWorld grid(0, 0, empty_obs); 
    vector<Agent> agents;

    // Load data from file
    if (!load_scenario("../dataset/maze-32-32-2/4-0.yaml", grid, agents)) {
        return -1;
    }

    cout << "Loaded Map: " << grid.height << "x" << grid.width << endl;
    cout << "Loaded " << agents.size() << " agents." << endl;

    // Run CBS
    cout << "Running CBS..." << endl;
    Solution sol = CBS(grid, agents);

    if (!sol.empty()) {
        cout << "Solution Found!" << endl;
        print_paths(sol);
    } else {
        cout << "No Solution Found." << endl;
    }

    return 0;
}