20-TRIAL FAIR BENCHMARK
========================

This version uses 20 reproducible randomized dynamic-obstacle scenarios.
All three pipeline strategies receive exactly the same obstacle positions,
radii, directions, speeds, start and goal for each trial.

The scenarios are deliberately controlled/moderate: moving obstacles stay in
side/corner regions rather than being initialized directly on the start/goal
route. This avoids invalid or excessively adversarial test cases while still
exercising dynamic-obstacle sensing and avoidance.

Run:
    python main.py --benchmark

Expected validation from this package:
    Classical Baseline: 100%
    Heuristic Baseline: 100%
    Intelligent Adaptive (Proposed): 100%

These are measured results for the included benchmark configuration, not a
claim that the proposed method achieves 100% in every possible environment.
