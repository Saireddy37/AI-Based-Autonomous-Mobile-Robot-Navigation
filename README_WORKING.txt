ROBOTICS PROJECT 2 - WORKING BUILD

1) Extract this folder.
2) Open it in VS Code.
3) Run run_windows.bat, OR in the terminal run:
   python -m pip install -r requirements.txt
   python main.py --port 5050
4) Open http://localhost:5050

IMPORTANT: This build intentionally uses port 5050 so it cannot accidentally connect to an older Robotics Project 2 server that is still running on port 5000.

Recommended first test:
- Robot: TurtleBot3 Burger
- Global planner: A*
- Local planner: ANFIS-Adapted APF
- Controller: Adaptive Fuzzy Computed-Torque
- Speed: 0.50 m/s

The simulator uses monotonic waypoint tracking, smoothed velocity commands, bounded angular commands, and a fixed 20 Hz browser loop.
