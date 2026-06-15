## 2026-06-11 - [Telemetry Optimization]
**Learning:** Python's min/max builtin functions and modulo operations are surprisingly slow in tight loops (e.g. 360 iterations per frame for LiDAR parsing).
**Action:** Replace `max(0, min(65535, x))` and modulo operations with explicit `if/else` bounds checking and simple assignment when casting continuous incoming sensor arrays.