## 2024-05-24 - A* Global Planner Performance Learnings
**Learning:** O(N) upfront allocation of huge 2D numpy grids (`g_score = np.full((rows, cols), np.inf)`) causes significant memory overhead and initial performance hits during route planning on large map resolutions, especially on resource-constrained devices like Raspberry Pi. Additionally, using Euclidean distance (`math.hypot`) as a heuristic is computationally expensive and less optimal for 8-connected grid maps compared to the Octile distance. Cost-map penalties (like hallway mapping) can also dramatically increase search path length.

**Action:**
1. Use sparse dictionaries (`{(r, c): cost}`) for tracking `g_score` and `came_from` in A* implementations, saving time and memory by only allocating observed nodes.
2. Implement inline Octile distance heuristics (`dx + dy + (1.414 - 2) * min(dx, dy)`) instead of `math.hypot` to get a tighter, more computationally efficient admissible bound for 8-way grids.
3. When using penalty-based costmaps in A*, ensure the iteration limit (`max_iters`) is scaled up significantly (e.g. `rows * cols * 4`) to allow the search to explore detours without prematurely failing.
