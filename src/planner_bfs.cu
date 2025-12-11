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

struct DeviceConstraint {
    int time;
    int row;
    int col;
};



static __device__ bool in_bounds(int r, int c, int width, int height) {
    return (r >= 0 && r < height && c >= 0 && c < width);
}

static __device__ bool is_free_cell(int r, int c, int width, int height, const unsigned char* obs) {
    if (!in_bounds(r, c, width, height)) return false;
    int idx = r * width + c;
    return (obs[idx] == 0);
}

__device__ bool violates_constraints(int row, int col, int time, const DeviceConstraint* d_constraints, int num_constraints) {
    for(int i = 0; i < num_constraints; i++){
        int c_time = d_constraints[i].time;
        int c_row = d_constraints[i].row;
        int c_col = d_constraints[i].col;
        if(c_time == time && c_row == row && c_col == col) return true;
    }
    return false;
}


__global__ void bfs_kernel(int width, int height,
                            const unsigned char* obstacles,
                            int start_r, int start_c,
                            int goal_r, int goal_c,
                            const DeviceConstraint* d_constraints,
                            int num_constraints,
                            int* parent,
                            unsigned int* visited,
                            int* frontier_curr,
                            int* frontier_next,
                            int* found_global)
{
    int tid = threadIdx.x;
    int num_cells = width * height;
    int numThreads = blockDim.x;

    // __shared__ unsigned char visited[num_cells];
    __shared__ int frontier_curr_size;
    __shared__ int frontier_next_size;
    __shared__ int found_local;


    for (int i = tid; i < num_cells; i += numThreads) {
        visited[i] = 0;
        parent[i] = -1;
    }
    if (tid == 0) {
        found_local = 0;
        frontier_curr_size = 0;
        frontier_next_size = 0;
    }
    __syncthreads();

    int start_cell = start_r * width + start_c;
    int goal_cell  = goal_r * width + goal_c;

    if (tid == 0) {
        if (obstacles[start_cell] || obstacles[goal_cell]) {
            return;
        }
        visited[start_cell] = 1;
        parent[start_cell] = start_cell;
        frontier_curr[0] = start_cell;
        frontier_curr_size = 1;
    }
    __syncthreads();
    int time = 0;

    while (true) {
        time += 1;
        int f_size = frontier_curr_size;
        if (f_size == 0) {
            break;
        }

        for (int idx = tid; idx < f_size; idx += blockDim.x) {
            int cell = frontier_curr[idx];
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

                if (obstacles[ncell] || violates_constraints(nr, nc, time, d_constraints, num_constraints)) continue;

                // Try to visit ncell
                if (!visited[ncell]) {
                    if (atomicExch(&visited[ncell], 1) == 0) {
                        parent[ncell] = cell;
                        int pos = atomicAdd(&frontier_next_size, 1);
                        frontier_next[pos] = ncell;
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
        
        int* temp = frontier_curr;
        frontier_curr = frontier_next;
        frontier_next = temp;

        __syncthreads();
    }
    if (tid == 0) {
        *found_global = found_local;
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

Path plan_path_gpu_bfs(const GridWorld& grid,
                        Position start,
                        Position goal,
                        const std::set<Constraint>& constraints) {
    init_device_grid(grid);

    std::vector<DeviceConstraint> constraints_copy;
    constraints_copy.reserve(constraints.size());
    for (const auto& c : constraints) {
        int t = c.first;
        int row = c.second.first;
        int col = c.second.second;
        constraints_copy.push_back({t, row, col});
    }

    DeviceConstraint* cudaDeviceconstraints = nullptr;
    int num_constraints = static_cast<int>(constraints_copy.size());
    if (num_constraints > 0) {
        cudaMalloc(&cudaDeviceconstraints, num_constraints * sizeof(DeviceConstraint));
        cudaMemcpy(cudaDeviceconstraints, constraints_copy.data(), num_constraints * sizeof(DeviceConstraint), cudaMemcpyHostToDevice);
    }



    dim3 gridDim(1);
    dim3 blockDim(BLOCKSIZE);

    int num_cells = grid.width * grid.height;

    int* parent = nullptr;
    unsigned int* visited = nullptr;
    int* frontier_curr = nullptr;
    int* frontier_next = nullptr;
    int* found_global = nullptr;

    cudaMalloc(&parent, num_cells * sizeof(int));
    cudaMalloc(&visited, num_cells * sizeof(unsigned int));
    cudaMalloc(&frontier_curr, num_cells * sizeof(int));
    cudaMalloc(&frontier_next, num_cells * sizeof(int));
    cudaMalloc(&found_global, sizeof(int));
    
    bfs_kernel<<<gridDim, blockDim>>>(Dgrid.width, Dgrid.height, Dgrid.obs, start.first, start.second, goal.first, goal.second, cudaDeviceconstraints, num_constraints, parent, visited, frontier_curr, frontier_next, found_global);

    cudaCheckError(cudaDeviceSynchronize());


    int found = 0;
    cudaMemcpy(&found, found_global, sizeof(int), cudaMemcpyDeviceToHost);
    Path result;
    if (found) {
        std::vector<int> h_parent(num_cells);
        cudaMemcpy(h_parent.data(), parent, num_cells * sizeof(int), cudaMemcpyDeviceToHost);

        int start_cell = start.first * grid.width + start.second;
        int goal_cell = goal.first * grid.width + goal.second;
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
        if (!rev.empty() && rev.back() == start) {
            result.assign(rev.rbegin(), rev.rend());
        }
    }
    else{
        std::cerr << "GPU BFS failed or found no path back to serial\n";
        result = plan_path_serial(grid, start, goal, constraints);
    }


    if (cudaDeviceconstraints) cudaFree(cudaDeviceconstraints);
    cudaFree(parent);
    cudaFree(visited);
    cudaFree(frontier_curr);
    cudaFree(frontier_next);
    cudaFree(found_global);

    return result;
}