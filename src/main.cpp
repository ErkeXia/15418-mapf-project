#include <iostream>
#include <chrono>
#include <string>

#include "grid.h"
#include "planner.h"
#include "cbs.h"

int main(int argc, char** argv) {
    GridWorld grid;
    std::vector<Agent> agents;

    // Change path as needed
    // if (!load_scenario("../dataset/Paris_1_256/4-0.txt", grid, agents)) {
    // if (!load_scenario("../dataset/w_woundedcoast/4-0.txt", grid, agents)) {
    // if (!load_scenario("../dataset/maze-32-32-2/4-0.txt", grid, agents)) {
    if (!load_scenario("../dataset/sample.txt", grid, agents)) {
        return -1;
    }

    std::cout << "Loaded Map: " << grid.height << "x" << grid.width << std::endl;
    std::cout << "Loaded " << agents.size() << " agents" << std::endl;

    // Choose planner: serial by default, later you can parse argv for GPU
    PlannerFunc planner = plan_path_serial;
    if (argc > 1 && std::string(argv[1]) == "gpu") {
        planner = plan_path_gpu_bfs;
        std::cout << "Using GPU planner (BFS-based)\n";
    } else {
        std::cout << "Using serial A* planner\n";
    }

    std::cout << "Running CBS" << std::endl;
    auto start = std::chrono::high_resolution_clock::now();
    Solution sol = CBS(grid, agents, planner);
    auto end = std::chrono::high_resolution_clock::now();

    if (!sol.empty()) {
        std::cout << "Solution Found!" << std::endl;
        // print_paths(sol);
    } else {
        std::cout << "No Solution Found" << std::endl;
    }

    auto duration_ms =
        std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count();

    std::cout << "CBS took " << duration_ms << " ms" << std::endl;

    return 0;
}
