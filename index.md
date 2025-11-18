# Parallel Multi-Agent Path Finding on GPUs and Multicore CPUs

**Course:** 15-418/618 Fall 2024  
**Students:** YOUR NAME (and partner’s name, Andrew IDs)  

---

## URL
This page: https://YOUR-USERNAME.github.io/15418-mapf-project/

## SUMMARY
We plan to implement and evaluate parallel algorithms for Multi-Agent Path Finding (MAPF) on both multicore CPUs and GPUs. We will start from a baseline CBS / A*-based MAPF solver and explore different ways to expose parallelism across agents and search nodes.

## BACKGROUND
(Write a short description of MAPF here — grid maps, many agents, collision constraints, CBS/A*, why it’s expensive, etc.)

## THE CHALLENGE
(Explain why MAPF is hard to parallelize: irregular search tree, varying path lengths, conflicts, memory access patterns, potential divergence on GPU, etc.)

## RESOURCES
- Machines: GHC lab machines / latedays cluster / EC2 GPU instance (specify what you’ll actually use).
- Code: (e.g., you can mention if you’ll start from an existing MAPF codebase or implement from scratch.)
- Papers: cite a couple of MAPF + GPU / CBS papers you’ll use as参考.

## GOALS AND DELIVERABLES
**Plan to achieve:**
- Implement a correct baseline serial MAPF solver (e.g., CBS or HNA*).
- Design and implement at least one parallel CPU version (OpenMP / pthreads).
- Implement a GPU-based MAPF component (e.g., batched low-level A* search).
- Measure speedup vs baseline across number of agents and map sizes.

**Hope to achieve (stretch goals):**
- Multiple GPU kernels or alternative parallelization strategies (task vs data parallel).
- Compare heterogeneous setups (CPU-only vs GPU-only vs hybrid).
- Additional visualizations (per-agent completion time, conflict statistics).

**If things go slowly:**
- Focus on a single platform (e.g., GPU only) but with deeper performance analysis.

## PLATFORM CHOICE
(Describe why GPU + multicore CPU are appropriate for MAPF: lots of per-agent work, potential for massive parallelism, etc.)

## SCHEDULE
Week 1: Implement / clean up baseline serial MAPF, define benchmarks  
Week 2: Parallel CPU version + initial performance results  
Week 3: GPU prototype, tune kernel and memory layout  
Week 4: Full evaluation, graphs, and poster preparation
