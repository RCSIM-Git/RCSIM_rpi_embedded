## 2024-06-15 - Fast Path for Primitive Serialization Checks
**Learning:** In deeply recursive sanitization functions (`sanitize_payload`), placing an early exit for common primitive types using strict type checking (`type(obj) in (str, int, float, bool, type(None))`) provides substantial speedups over doing multiple `isinstance` checks for complex objects first.
**Action:** Next time you see recursive serialization or tree-traversal logic processing lots of primitive leaf nodes, try putting an exact type check at the beginning.
