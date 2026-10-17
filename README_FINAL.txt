ROBOTICS PROJECT 2 - FINAL VERIFIED BUILD
==========================================

WINDOWS / VS CODE QUICK START
------------------------------
1. Extract this ZIP into a new folder.
2. Open the folder that contains main.py in VS Code.
3. Open Terminal.
4. Run:
     python -m pip install -r requirements.txt
5. Verify the whole project:
     python main.py --self-test
6. Start the dashboard:
     python main.py --port 5050
7. Open:
     http://localhost:5050

Recommended configuration:
  Robot: TurtleBot3 Burger
  Global planner: A* (Grid Search)
  Local planner: ANFIS-Adapted APF
  Dynamic controller: Adaptive Fuzzy Computed-Torque
  Speed: 0.50 m/s

Verified in this build:
  - Symbolic Lagrangian/Newton-Euler proof
  - A*, RRT*, and Genetic planners with safe deterministic A* fallback
  - APF, ANFIS-APF, and DWA local planning
  - Standard CTC, Adaptive CTC, and SMC controller modes
  - All 81 dashboard robot/planner/local/controller combinations initialize and execute smoke steps
  - Recommended ANFIS + Adaptive CTC pipeline reaches the goal without collision for Burger, Waffle and Pioneer
  - DWA is optimized and checks static obstacles; SMC+DWA receives a conservative safety speed cap
  - Flask API converts NumPy values to JSON-native values
  - Browser simulation runs at 20 Hz and retries transient API failures
  - Frontend cache-busting prevents an older JavaScript build from being used accidentally
  - Benchmark trials are reproducible, use a 60 s simulation window, and report path, smoothness,
    convergence time, computation time, and mechanical torque-work energy
  - Benchmark torque energy is non-zero and based on wheel mechanical work

Benchmark:
  Run from the dashboard with Run Benchmark Matrix, or from the terminal with:
     python main.py --benchmark

Important limitation:
  No project can guarantee execution after arbitrary source-code edits. If a teacher changes
  Python code, use `python main.py --self-test` first. Normal dashboard configuration changes
  are designed to remain safe and executable; unsupported planner/controller failures fall back
  rather than crashing the simulation.
