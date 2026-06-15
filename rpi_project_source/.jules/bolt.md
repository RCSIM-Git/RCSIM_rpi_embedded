## 2025-02-14 - Python Loop Overhead with min/max
**Learning:** Python built-in functions `min()` and `max()` have significant overhead when called in tight inner loops (e.g., iterating through a 360-point LiDAR scan array 4 times per second). Replacing them with simple `if/elif` statements cuts the execution time for the structure packing by about 60%.
**Action:** Always prefer basic conditional blocks (`if/elif/else`) over `min()`/`max()` for clamping values inside tight, performance-critical inner loops where pure Python overhead is noticeable.
