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
      background:#0366d6;
      color:white;
  ">
    Milestone
  </a>
</p>

# Parallel Multi-Agent Path Finding  
Erke Xia (exia)

---

## Milestone Progress
I have fully implemented a serial CBS solver in C++ and verified correctness on MAPF test cases. I benchmarked its performance by measuring end-to-end solving time on standard grid maps. This provides a baseline for later comparison with GPU-accelerated versions.

I then studied the GPU-parallel low-level search algorithm described in the GACBS paper in detail. Their design is significantly more complex than I originally anticipated: each block runs a parallel A* search for a single constrained single-agent path, storing states in a per-block node array, maintaining multiple thread-local heaps, and using a parallel hash table with PHR duplicate detection. They further rely on prefix-sum-based compaction to remove invalid nodes and reintegrate survivors into the hash table. After reviewing these components carefully, I determined that fully replicating GATSA is beyond the current scope of my project timeline.

Instead, I shifted to implementing a simplified GPU-parallel BFS search algorithm inspired by their architecture. Like GACBS, I assign one low-level pathfinding task per block, but I replace A* and heap-based frontier management with a level-synchronous BFS. Within each time step, threads in the block expand different nodes in the frontier in parallel, which distributes work efficiently once the search wavefront grows large. Although some threads are idle early in the search, parallel expansion produces increasing speedup relative to the serial solver as the frontier widens.

I am currently integrating this GPU-parallel search with CBS and preparing benchmark comparisons against the serial baseline. Progress has been slower than expected as I revised the low-level search design. Once integrated, I will evaluate performance on the full CBS pipeline and then move on to implementing the CPU-side constraint-tree logic with the task-based scheduling model. If time permits after a working end-to-end system, I may revisit more advanced GPU search algorithms closer to the full A* approach described in GACBS.

## Updated Schedule
12/1-12/4 Integrate the GPU-parallel search with CBS and compare the performance with serial baseline. 
12/4-12/6 Implement the CPU-side constraint-tree logic with the task-based scheduling model.
12/6-12/8 Benchmark full CBS pipeline, analyze speedup, prepare final results

## Goals
My initial goal of replicating the full GACBS algorithm and designing an additional parallel MAPF solver turned out to be too large in scope due to the complexity of the GPU parallel A* search. I now shift my goal to a more realistic and well-defined target: to implement a simplified version of GPU-accelerated CBS. I aim to compare runtime of serial CBS vs GPU-parallel BFS-based CBS and investigate how frontier size, task batching, and map characteristics affect acceleration
