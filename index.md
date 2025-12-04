<p>
  <a href="./" style="
      display:inline-block;
      padding:6px 12px;
      margin-right:8px;
      border-radius:4px;
      border:1px solid #0366d6;
      text-decoration:none;
      font-weight:600;
  ">
    Proposal
  </a>

  <a href="./milestone.html" style="
      display:inline-block;
      padding:6px 12px;
      border-radius:4px;
      border:1px solid #0366d6;
      text-decoration:none;
      font-weight:600;
  ">
    Milestone
  </a>
</p>


# Parallel Multi-Agent Path Finding  
Erke Xia (exia)

---

## URL
This page: https://erkexia.github.io/15418-mapf-project/

## SUMMARY
I plan to implement and evaluate parallel algorithms for Multi-Agent Path Finding (MAPF). I will begin by replicating a GPU-accelerated Conflict-Based Search (CBS) solver and explore multiple ways to expose parallelism across agents, constraint-tree nodes, and low-level search operations.

## BACKGROUND
Single-agent pathfinding refers to the problem of finding a path between two vertices in a graph. The multi-agent pathfinding (MAPF) problem is a generalization of the single-agent pathfinding problem for k > 1 agents. It consists of a graph and a number of agents. For each agent, a unique start state and a unique goal state are given, and the task is to find paths for all agents from their start states to their goal states, under the constraint that agents cannot collide during their movements. The multiple paths in the final solution suggests that the solver may benefit from parallelism. 

Conflict-based search (CBS) is a well-established two-level solver for the MAPF problem. The high-level solver constructs a constraint tree (CT) to assign time-space constraints for each agent to avoid collisions, and the low-level solver calculates a path under these constraints. A recent research presents (GPU-accelerated conflict based search) GACBS, which introduces a task coordination framework (TCF) on the GPU. High-level CBS spawns pathfinding “tasks” and low-level solver GATSA (a GPU-accelerated constrained shortest-path solver) runs many of these in parallel. I aim to start with replicating this framework.

## THE CHALLENGE
Replicating a GPU-accelerated Conflict-Based Search (CBS) solver presents two fundamental challenges.
First, the coordination between CPU-side high-level CBS and GPU-side low-level searches must be lightweight. Any heavy or fine-grained synchronization across components can severely restrict the GPU’s ability to run many tasks concurrently, leading to poor utilization and large stalls.
Second, the single-agent constrained pathfinding solver itself is complex to implement efficiently on a GPU, since A* and similar graph-search algorithms have highly irregular structure that does not map naturally onto GPU.

At the algorithmic level, CBS introduces several sources of irregularity. High-level CT (Constraint Tree) nodes are partially ordered as each child node depends on its parent’s constraints and paths. However, once a CT node has been created, its low-level path computations are independent of other CT nodes and can be executed in parallel. Even within a single low-level search, A* expands nodes sequentially based on the minimum frontier cost, but each expansion step generates a batch of successor states that can be processed in parallel by GPU threads. This mixture of sequential control flow and parallel inner loops makes designing a scalable GPU version nontrivial.

A key difficulty comes from the irregular workload and the communication boundary between CPU and GPU. We intend to use a lightweight task list as the paper does: the CPU inserts task descriptors (agent, start/goal, constraint set) into a shared array, launches the GPU kernel, and later inspects tasks marked as completed. The overhead of writing descriptors, launching kernels, reading back paths, and synchronizing must remain small relative to the amount of useful work each task performs. If not, communication costs will overwhelm computation.

The low-level searches are challenging as well. Constrained A* and BFS are inherently branchy, irregular graph searches with dynamic priority queues, hash tables, and variable-size frontiers. These components are difficult to implement efficiently on GPUs. The search frontier moves unpredictably across the grid, resulting in non-coalesced memory accesses, warp divergence, and limited locality. Designing GPU-friendly data structures becomes essential but complicated.

Another major challenge is load imbalance. Different CT nodes and agents produce subproblems of drastically different sizes: some terminate after a few expansions, while others may explore a large fraction of the map. On a GPU, this leads to thread blocks finishing at different times, leaving SMs underutilized unless an efficient task scheduling mechanism is used. Managing this imbalance while keeping synchronization overhead low is a core part of the system design.

## RESOURCES
- Machines: GHC lab machines and EC2 GPU instances.
- Code: I will start with a standard CBS implementation that I previously developed.
- Papers: Tang, M., Xin, R., Fang, C. et al. GPU-accelerated Conflict-based Search for Multi-agent Embodied Intelligence. Mach. Intell. Res. 22, 641–654 (2025). https://doi.org/10.1007/s11633-025-1568-y

## GOALS AND DELIVERABLES
**Plan to achieve:**
- Implement a correct baseline serial CBS solver and evaluate it on MAPF benchmarks.
- Replicate the GPU-accelerated CBS algorithm (task coordination + GPU low-level search).
- Measure speedups versus the baseline across number of agents and map sizes.

**Hope to achieve:**
- Design and implement an additional parallel MAPF solver, potentially using hierarchical or HNA*-style parallelism.

**If things go slowly:**
- Focus on the GPU-accelerated low-level A* search, and evaluate speedup from low-level parallelism alone.

I will use standard MAPF benchmark environments with three grid sizes: small, medium, and large, specifically: maze-32-32-2, Paris_1_256, and w_woundedcoast, following GACBS. I aim to understand how CPU–GPU cooperation influences overall performance.

## PLATFORM CHOICE
I plan to use C++ and the GHC machines for development. The availability of multicore CPUs and CUDA-capable GPUs makes these platforms appropriate for parallel CBS and GPU-accelerated pathfinding.

## SCHEDULE
11.17–11.24: Implement baseline serial CBS and evaluate on benchmarks.

11.24–12.1: Implement the GPU-parallel low-level search algorithm.

12.1–12.4: Integrate the high-level CPU CT logic with the GPU task system.

12.4–12.8: Perform full evaluation, tune kernel configurations, analyze CPU–GPU interaction, and prepare results.
