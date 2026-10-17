ROBOTICS PROJECT 2 - VERIFIED BUILD

Run:   python -m pip install -r requirements.txt
       python main.py --port 5050      (open http://localhost:5050)
Test:  python -m unittest tests.test_full_execution tests.test_system
       (or: python main.py --self-test)
NOTE: stop any older server first, or you may see old behaviour.

What was fixed (see paper: Loganathan & Ahmad 2023, AMR navigation review)
GLOBAL PLANNERS
- A*     : unchanged (8-connected, inflated grid, shortcut smoothing).
- RRT*   : steer no longer overshoots; best-parent + rewiring now propagates cost
           to descendants; keeps refining after first solution; path ends at goal.
- GA     : rewritten per paper Algorithm 1 - global+local population init,
           length+collision fitness, tournament selection, one-point crossover,
           mutation, elitism. Previously it never found a valid path and was
           silently replaced by A*. /api/init now returns "planner_used".
LOCAL PLANNERS
- APF / ANFIS-APF: GNRON-style goal weighting capped, tangential (vortex) force to
           escape local minima, static walls/pillars repel too.
- DWA    : unchanged (falls back to APF if it stalls).
CONTROLLERS
- Adaptive CTC: fuzzy tuner now gets velocity error and its rate (was goal distance,
           which pinned the gain at maximum and made turning oscillate).
- CTC / SMC: verified by step response - converge on all 3 robots.
OTHER: /api/step no longer errors after a run finishes.

START / GOAL SELECTION (new)
- In the dashboard: press "Set Start" or "Set Goal", then click the map; or type X/Y
  and press Apply. "Default" restores (1,1) -> (9,9). The blue dot is the start,
  the green dot is the goal. Everything else (planners, controllers) is unchanged
  and uses the chosen points.
- The server rejects points inside walls/pillars/too close to them, outside the map,
  closer than 1 m to each other, or with no collision-free path; the previous valid
  points are kept and a message is shown.
- Moving obstacles are never spawned within 1.3 m of the chosen start/goal.

3D VIEW (new)
- In the dashboard header above the map press "3D" (and "2D" to go back).
- Camera modes: Orbit (drag to rotate, mouse wheel to zoom), Top (bird's-eye) and
  Chase (camera follows behind the robot).
- Shows: floor grid, boundary walls, pillars, moving obstacles (red cylinders) with
  velocity arrows, robot model (chassis, wheels, lidar puck, heading arrow), planned
  path (yellow dashed), driven trajectory (magenta), lidar rays, start ring (blue),
  goal flag (green). Robot models are drawn 3x larger so they are visible on a 10 m map.
- "Set Start" / "Set Goal" also work by clicking directly on the 3D floor.
- Implemented in web/static/js/view3d.js: a self-contained perspective renderer
  (no Three.js / no internet needed). Map orientation: x to the right, y away from you.

PYTHON 3D VIEWER (new, no browser)
- python run_3d.py                       -> interactive 3D window (matplotlib)
- python run_3d.py --robot waffle --planner RRTStar --local DWA --controller SMC
- python run_3d.py --start 1 9 --goal 9 1 --seed 3
- python run_3d.py --save demo.gif --no-show     (animated GIF)
- python run_3d.py --png final.png --no-show     (image of the final frame)
- Add --spin to rotate the camera automatically. Needs: pip install matplotlib
