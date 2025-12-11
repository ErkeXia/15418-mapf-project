#include "planner.h"
#include "grid.h"


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
#include <iostream>
#include <cuda_runtime.h>
#include <vector>

using namespace std;


#define BLOCKSIZE 256

#define DEBUG
#ifdef DEBUG
#define cudaCheckError(ans) cudaAssert((ans), __FILE__, __LINE__);
inline void cudaAssert(cudaError_t code, const char *file, int line, bool abort=true)
{
    if (code != cudaSuccess)
{
fprintf(stderr, "CUDA Error: %s at %s:%d\n",
cudaGetErrorString(code), file, line);
if (abort) exit(code);
}
}
#else
#define cudaCheckError(ans) ans
#endif



struct DeviceGrid {
    int width;
    int height;
    unsigned char* obs = nullptr;
};



static __device__ bool in_bounds(int r, int c, int width, int height) {
    return (r >= 0 && r < height && c >= 0 && c < width);
}

static __device__ bool is_free_cell(int r, int c, int width, int height, const unsigned char* obs) {
    if (!in_bounds(r, c, width, height)) return false;
    int idx = r * width + c;
    return (obs[idx] == 0);
}


__global__ void bfs_kernel_multi(
    int width, int height,
    const unsigned char* obstacles,
    const int* start_r, const int* start_c,
    const int* goal_r,  const int* goal_c,
    int* parent,
    unsigned int* visited,
    int* frontier_curr,
    int* frontier_next,
    int* found_global,
    int num_cells)
{
    int agent_id = blockIdx.x;
    int tid = threadIdx.x;
    int numThreads = blockDim.x;

    int offset = agent_id * num_cells;
    int* parent_a = parent + offset;
    unsigned int* visited_a = visited + offset;
    int* frontier_curr_a = frontier_curr + offset;
    int* frontier_next_a = frontier_next + offset;
    int* found_a = found_global + agent_id;

    __shared__ int frontier_curr_size;
    __shared__ int frontier_next_size;
    __shared__ int found_local;

    for (int i = tid; i < num_cells; i += numThreads) {
        visited_a[i] = 0;
        parent_a[i]  = -1;
    }
    if (tid == 0) {
        frontier_curr_size = 0;
        frontier_next_size = 0;
        found_local        = 0;
    }
    __syncthreads();

    int start_cell = start_r[agent_id] * width + start_c[agent_id];
    int goal_cell = goal_r[agent_id] * width + goal_c[agent_id];

    if (tid == 0) {
        if (obstacles[start_cell] || obstacles[goal_cell]) {
            found_local = 0;
            *found_a = 0;
        } else {
            visited_a[start_cell] = 1;
            parent_a[start_cell] = start_cell;
            frontier_curr_a[0] = start_cell;
            frontier_curr_size = 1;
        }
    }
    __syncthreads();

    while (true) {
        int f_size = frontier_curr_size;
        if (f_size == 0) {
            break;
        }
        for (int idx = tid; idx < f_size; idx += blockDim.x) {
            int cell = frontier_curr_a[idx];
            if (cell == goal_cell) {
                found_local = 1;
                continue;
            }
            int r = cell / width;
            int c = cell % width;

            const int dr[4] = { -1, 1, 0, 0 };
            const int dc[4] = { 0, 0, -1, 1 };

            for (int k = 0; k < 4; ++k) {
                int nr = r + dr[k];
                int nc = c + dc[k];

                if (!in_bounds(nr, nc, width, height))
                    continue;
                int ncell = nr * width + nc;

                if (obstacles[ncell]) continue;

                if (!visited_a[ncell]) {
                    if (atomicExch(&visited_a[ncell], 1) == 0) {
                        parent_a[ncell] = cell;
                        int pos = atomicAdd(&frontier_next_size, 1);
                        frontier_next_a[pos] = ncell;
                    }
                }
            }
        }

        __syncthreads();

        if (found_local) {
            break;
        }

        if (tid == 0) {
            frontier_curr_size = frontier_next_size;
            frontier_next_size = 0;
        }

        int* tmp = frontier_curr_a;
        frontier_curr_a = frontier_next_a;
        frontier_next_a = tmp;

        __syncthreads();
    }

    if (tid == 0) {
        *found_a = found_local;
    }
}

static DeviceGrid Dgrid;
static void init_device_grid(const GridWorld& grid) {
    if (Dgrid.obs != nullptr) return;
    Dgrid.width  = grid.width;
    Dgrid.height = grid.height;
    size_t num_cells = static_cast<size_t>(grid.width) * grid.height;

    std::vector<unsigned char> obstacles(num_cells, 0);
    for (const auto& pos : grid.obstacles) {
        int r = pos.first;
        int c = pos.second;
        int idx = r * grid.width + c;
        obstacles[idx] = 1;
    }
    cudaMalloc(&Dgrid.obs, num_cells * sizeof(unsigned char));
    cudaMemcpy(Dgrid.obs, obstacles.data(), num_cells * sizeof(unsigned char), cudaMemcpyHostToDevice);
}

std::vector<Path> plan_path_init(const GridWorld& grid, const vector<Agent>& agents) {
    init_device_grid(grid);

    int num_agents = static_cast<int>(agents.size());
    int num_cells = grid.width * grid.height;
    
    dim3 gridDim(num_agents);
    dim3 blockDim(BLOCKSIZE);

    vector<int> h_start_r(num_agents), h_start_c(num_agents);
    vector<int> h_goal_r(num_agents), h_goal_c(num_agents);

    for (int i = 0; i < num_agents; ++i) {
        h_start_r[i] = agents[i].start.first;
        h_start_c[i] = agents[i].start.second;
        h_goal_r[i] = agents[i].goal.first;
        h_goal_c[i] = agents[i].goal.second;
    }

    int *d_start_r, *d_start_c, *d_goal_r, *d_goal_c;
    cudaMalloc(&d_start_r, num_agents * sizeof(int));
    cudaMalloc(&d_start_c, num_agents * sizeof(int));
    cudaMalloc(&d_goal_r,  num_agents * sizeof(int));
    cudaMalloc(&d_goal_c,  num_agents * sizeof(int));

    cudaMemcpy(d_start_r, h_start_r.data(), num_agents * sizeof(int), cudaMemcpyHostToDevice);
    cudaMemcpy(d_start_c, h_start_c.data(), num_agents * sizeof(int), cudaMemcpyHostToDevice);
    cudaMemcpy(d_goal_r,  h_goal_r.data(), num_agents * sizeof(int), cudaMemcpyHostToDevice);
    cudaMemcpy(d_goal_c,  h_goal_c.data(), num_agents * sizeof(int), cudaMemcpyHostToDevice);

    int* d_parent = nullptr;
    unsigned int* d_visited = nullptr;
    int* d_frontier_curr = nullptr;
    int* d_frontier_next = nullptr;
    int* d_found = nullptr;

    size_t cells_bytes = static_cast<size_t>(num_cells) * sizeof(int);

    cudaMalloc(&d_parent, num_agents * cells_bytes);
    cudaMalloc(&d_visited, num_agents * num_cells * sizeof(unsigned int));
    cudaMalloc(&d_frontier_curr, num_agents * cells_bytes);
    cudaMalloc(&d_frontier_next, num_agents * cells_bytes);
    cudaMalloc(&d_found, num_agents * sizeof(int));
    
    bfs_kernel_multi<<<gridDim, blockDim>>>(Dgrid.width, Dgrid.height, Dgrid.obs, d_start_r, d_start_c, d_goal_r,  d_goal_c,
        d_parent, d_visited, d_frontier_curr, d_frontier_next, d_found, num_cells);
    cudaCheckError(cudaDeviceSynchronize());

    std::vector<int> h_found(num_agents);
    cudaCheckError(cudaMemcpy(h_found.data(), d_found,
                              num_agents * sizeof(int),
                              cudaMemcpyDeviceToHost));

    std::vector<Path> all_paths(num_agents);

    std::vector<int> h_parent(num_cells);

    for (int a = 0; a < num_agents; ++a) {
        if (!h_found[a]) {
            std::cerr << "[GPU init] No path for agent " << agents[a].id
                      << "leaving path empty (CBS will fail or replan).\n";
            continue;
        }

        int offset = a * num_cells;
        cudaCheckError(cudaMemcpy(h_parent.data(),
                                  d_parent + offset,
                                  num_cells * sizeof(int),
                                  cudaMemcpyDeviceToHost));

        int start_cell = h_start_r[a] * grid.width + h_start_c[a];
        int goal_cell  = h_goal_r[a]  * grid.width + h_goal_c[a];

        int cur = goal_cell;
        std::vector<Position> rev;

        while (true) {
            if (cur < 0 || cur >= num_cells) break;
            int r = cur / grid.width;
            int c = cur % grid.width;
            rev.push_back({r, c});

            if (cur == start_cell) break;

            int p = h_parent[cur];
            if (p == cur || p == -1) {
                rev.clear();
                break;
            }
            cur = p;
        }

        if (!rev.empty() && rev.back() == agents[a].start) {
            all_paths[a].assign(rev.rbegin(), rev.rend());
        } else {
            std::cerr << "[GPU init] Failed to reconstruct path for agent "
                      << agents[a].id << " falling back to empty.\n";
        }
    }

    cudaFree(d_start_r);
    cudaFree(d_start_c);
    cudaFree(d_goal_r);
    cudaFree(d_goal_c);

    cudaFree(d_parent);
    cudaFree(d_visited);
    cudaFree(d_frontier_curr);
    cudaFree(d_frontier_next);
    cudaFree(d_found);

    return all_paths;
}