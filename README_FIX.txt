# Robotics Project 2 — Navigation Fix

The original project runs, but the navigation controller can loop around a global-path waypoint.

Main fixes:
1. Global-path waypoint selection is now monotonic instead of repeatedly selecting the first waypoint farther than 0.6 m.
2. Target heading is calculated from the robot position to the target point.
3. Kinematic commands are clipped to physical limits.
4. Computed-torque derivative gain is reduced and requested acceleration is bounded to prevent torque saturation/oscillation.
5. Dynamic controller states are reset when a simulation is initialized.

Run:
    python -u "robotics project 2/main.py"

Then open http://localhost:5000


SECOND STABILITY PATCH
- A* is now the default global planner because its smoothed path is deterministic for the default map.
- APF angular command is limited to 1.2 rad/s.
- APF velocity commands are rate-limited before CTC/SMC tracking to reduce oscillation and torque reversals.
- Actual simulated linear speed is capped at 0.5 m/s for stable default operation.
- Web speed slider maximum is 0.80 m/s.
- API default global planner is A*.
