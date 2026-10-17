"""
run_3d.py - Python-only 3D visualisation of the navigation simulation (matplotlib).

It runs the SAME simulation engine as the web dashboard (global planner + local
planner + dynamic torque controller) and then plays it back as an interactive 3D
animation. You can rotate/zoom the 3D window with the mouse while it plays.

Examples
    python run_3d.py
    python run_3d.py --robot waffle --planner RRTStar --local DWA --controller SMC
    python run_3d.py --start 1 9 --goal 9 1 --seed 3
    python run_3d.py --save demo.gif --no-show          # write an animated GIF
    python run_3d.py --png final.png --no-show          # write the final frame as an image
"""
import argparse
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import numpy as np

from simulation.simulator import SimulationEngine

ROBOT_VIS_SCALE = 3.0          # robot is drawn 3x bigger so it is visible on a 10 m map
MAP_SIZE = 10.0


# --------------------------------------------------------------------------- simulation
def run_simulation(args):
    sim = SimulationEngine(args.robot)
    sim.setup_default_map(scenario_seed=args.seed, start=tuple(args.start), goal=tuple(args.goal))
    for label, pt in (("Start", args.start), ("Goal", args.goal)):
        err = sim.validate_point(pt, label)
        if err:
            sys.exit(f"Error: {err}")
    if np.hypot(args.goal[0] - args.start[0], args.goal[1] - args.start[1]) < 1.0:
        sys.exit("Error: start and goal must be at least 1 m apart.")
    sim.set_start_and_goal(tuple(args.start), tuple(args.goal))
    path = sim.plan_global_path(args.planner)
    if not sim.path_is_safe:
        sys.exit("Error: no collision-free path between these start and goal points.")
    if list(args.start) != [1.0, 1.0] or list(args.goal) != [9.0, 9.0]:
        sim.pose[2] = float(np.arctan2(path[1][1] - args.start[1], path[1][0] - args.start[0]))

    print(f"Planner used : {sim.planner_used}")
    print(f"Robot        : {sim.config.name}   Local: {args.local}   Controller: {args.controller}")

    frames = []
    for n in range(args.max_steps):
        sim.step(0.05, args.controller, args.local, args.speed)
        if n % args.stride == 0 or sim.is_done:
            frames.append({
                "pose": sim.pose.copy(),
                "obs": [(o.pos.copy(), o.radius) for o in sim.dynamic_obstacles],
                "t": sim.sim_time,
            })
        if sim.is_done:
            break
    status = "COLLISION" if sim.collision_occurred else ("GOAL REACHED" if sim.is_done else "TIME OUT")
    print(f"Result       : {status} after {sim.sim_time:.1f} s ({len(frames)} frames)")
    return sim, path, frames, status


# --------------------------------------------------------------------------- geometry
def merge_rects(grid, res):
    """Merge occupied grid cells into a small number of rectangles."""
    rows, cols = grid.shape
    seen = np.zeros_like(grid, dtype=bool)
    out = []
    for r in range(rows):
        for c in range(cols):
            if grid[r, c] != 1 or seen[r, c]:
                continue
            w = 1
            while c + w < cols and grid[r, c + w] == 1 and not seen[r, c + w]:
                w += 1
            h = 1
            while r + h < rows and np.all(grid[r + h, c:c + w] == 1) and not seen[r + h, c:c + w].any():
                h += 1
            seen[r:r + h, c:c + w] = True
            x0, x1 = (c - 0.5) * res, (c + w - 0.5) * res
            y0, y1 = (r - 0.5) * res, (r + h - 0.5) * res
            border = x0 <= 0.06 or y0 <= 0.06 or x1 >= MAP_SIZE - 0.15 or y1 >= MAP_SIZE - 0.15
            out.append((x0, y0, x1, y1, border))
    return out


def box_faces(x0, y0, x1, y1, z0, z1):
    return [
        [(x0, y0, z1), (x1, y0, z1), (x1, y1, z1), (x0, y1, z1)],   # top
        [(x0, y0, z0), (x1, y0, z0), (x1, y0, z1), (x0, y0, z1)],
        [(x0, y1, z0), (x1, y1, z0), (x1, y1, z1), (x0, y1, z1)],
        [(x0, y0, z0), (x0, y1, z0), (x0, y1, z1), (x0, y0, z1)],
        [(x1, y0, z0), (x1, y1, z0), (x1, y1, z1), (x1, y0, z1)],
    ]


def cylinder_faces(cx, cy, z0, z1, r, n=18, transform=None):
    T = transform or (lambda p: p)
    ring = [(cx + r * np.cos(a), cy + r * np.sin(a)) for a in np.linspace(0, 2 * np.pi, n, endpoint=False)]
    faces = []
    for k in range(n):
        p, q = ring[k], ring[(k + 1) % n]
        faces.append([T((p[0], p[1], z0)), T((q[0], q[1], z0)), T((q[0], q[1], z1)), T((p[0], p[1], z1))])
    faces.append([T((p[0], p[1], z1)) for p in ring])
    return faces


def robot_faces(pose, track, wheel_r):
    """Robot model (chassis + wheels + lidar + heading arrow) in world coordinates."""
    s = ROBOT_VIS_SCALE
    ct, st = np.cos(pose[2]), np.sin(pose[2])

    def T(p):                                   # robot frame -> world (with visual scale)
        return (pose[0] + s * (p[0] * ct - p[1] * st), pose[1] + s * (p[0] * st + p[1] * ct), s * p[2])

    R = max(track * 0.36, 0.06)
    z0 = max(0.02, wheel_r * 0.6)
    z_top = z0 + 0.07 + R * 0.35
    parts = []   # (faces, colour)
    parts.append((cylinder_faces(0, 0, z0, z_top, R, 22, T), "#00bcd4"))                    # chassis
    parts.append((cylinder_faces(-R * 0.12, 0, z_top, z_top + 0.06, R * 0.28, 14, T), "#111827"))   # lidar puck
    parts.append(([[T((R * 0.95, 0, z_top + 0.002)), T((R * 0.42, R * 0.3, z_top + 0.002)),
                    T((R * 0.42, -R * 0.3, z_top + 0.002))]], "#facc15"))                   # heading arrow
    for sg in (-1, 1):                                                                     # wheels (boxes)
        yc = sg * (track / 2 + 0.01)
        parts.append((box_wheel(yc, wheel_r, T), "#2b2b35"))
    return parts


def box_wheel(yc, wr, T, w=0.04):
    pts = lambda x, y, z: T((x, y, z))
    x0, x1, y0, y1, z0, z1 = -wr, wr, yc - w / 2, yc + w / 2, 0.0, 2 * wr
    c = lambda a, b, d: pts(a, b, d)
    return [
        [c(x0, y0, z1), c(x1, y0, z1), c(x1, y1, z1), c(x0, y1, z1)],
        [c(x0, y0, z0), c(x1, y0, z0), c(x1, y0, z1), c(x0, y0, z1)],
        [c(x0, y1, z0), c(x1, y1, z0), c(x1, y1, z1), c(x0, y1, z1)],
        [c(x0, y0, z0), c(x0, y1, z0), c(x0, y1, z1), c(x0, y0, z1)],
        [c(x1, y0, z0), c(x1, y1, z0), c(x1, y1, z1), c(x1, y0, z1)],
    ]


# --------------------------------------------------------------------------- drawing
def build_scene(plt, Poly3DCollection, sim, path, frames, status, args):
    fig = plt.figure(figsize=(9, 8), facecolor="#0b1220")
    ax = fig.add_subplot(111, projection="3d", facecolor="#0b1220")
    ax.set_xlim(0, MAP_SIZE); ax.set_ylim(0, MAP_SIZE); ax.set_zlim(0, 3.0)
    ax.set_box_aspect((MAP_SIZE, MAP_SIZE, 2.4))
    ax.view_init(elev=38, azim=-62)
    ax.set_axis_off()

    try:
        ax.computed_zorder = False          # use explicit layer order (floor < lines < solids)
    except Exception:
        pass

    # floor (checkerboard) - bottom layer
    floor = []
    colours = []
    for i in range(int(MAP_SIZE)):
        for j in range(int(MAP_SIZE)):
            floor.append([(i, j, 0), (i + 1, j, 0), (i + 1, j + 1, 0), (i, j + 1, 0)])
            colours.append("#1b2a44" if (i + j) % 2 else "#17233a")
    fl = Poly3DCollection(floor, facecolors=colours, edgecolors="none")
    fl.set_zorder(0)
    ax.add_collection3d(fl)

    # static solids (boundary walls, pillars, goal flag) collected as faces + colours
    static_faces, static_cols = [], []
    for (x0, y0, x1, y1, border) in merge_rects(sim.grid, sim.resolution):
        h, col = (0.45, "#64748b") if border else (0.95, "#8b78c4")
        f = box_faces(x0, y0, x1, y1, 0, h)
        static_faces += f
        static_cols += [col] * len(f)
    g0 = np.array(sim.goal, float)
    pole = box_faces(g0[0] - 0.02, g0[1] - 0.02, g0[0] + 0.02, g0[1] + 0.02, 0, 1.3)
    static_faces += pole + [[(g0[0], g0[1], 1.3), (g0[0] + 0.5, g0[1], 1.12), (g0[0], g0[1], 0.94)]]
    static_cols += ["#e5e7eb"] * len(pole) + ["#00e676"]

    # ground-level lines (middle layer): planned path, start ring, goal ring
    px, py = zip(*path)
    ln, = ax.plot(px, py, [0.03] * len(px), color="#ffd600", lw=2.5, ls="--")
    ln.set_zorder(1)
    th = np.linspace(0, 2 * np.pi, 40)
    s0 = np.array(args.start, float)
    for cx, cy, r, col in ((s0[0], s0[1], 0.32, "#2979ff"), (g0[0], g0[1], 0.42, "#00e676")):
        rl, = ax.plot(cx + r * np.cos(th), cy + r * np.sin(th), [0.03] * 40, color=col, lw=3)
        rl.set_zorder(1)

    trail, = ax.plot([], [], [], color="#e100ff", lw=2.5)
    trail.set_zorder(1)
    title = ax.text2D(0.5, 0.96, "", transform=ax.transAxes, ha="center", color="white", fontsize=12)
    ax.text2D(0.5, 0.02, f"{sim.config.name} | {sim.planner_used} + {args.local} + {args.controller}   "
              "(drag with the mouse to rotate, scroll to zoom)", transform=ax.transAxes,
              ha="center", color="#cbd5e1", fontsize=9)
    return fig, ax, trail, title, static_faces, static_cols


def main():
    ap = argparse.ArgumentParser(description="Python 3D visualisation of the robot navigation simulation")
    ap.add_argument("--robot", default="burger", choices=["burger", "waffle", "pioneer"])
    ap.add_argument("--planner", default="AStar", choices=["AStar", "RRTStar", "Genetic"])
    ap.add_argument("--local", default="ANFIS_APF", choices=["APF", "ANFIS_APF", "DWA"])
    ap.add_argument("--controller", default="Adaptive_CTC", choices=["CTC", "Adaptive_CTC", "SMC"])
    ap.add_argument("--start", type=float, nargs=2, default=[1.0, 1.0], metavar=("X", "Y"))
    ap.add_argument("--goal", type=float, nargs=2, default=[9.0, 9.0], metavar=("X", "Y"))
    ap.add_argument("--speed", type=float, default=0.5, help="speed regulator (m/s)")
    ap.add_argument("--seed", type=int, default=None, help="scenario seed (same seed = same obstacles)")
    ap.add_argument("--stride", type=int, default=5, help="simulation steps per animation frame")
    ap.add_argument("--max-steps", type=int, default=2400)
    ap.add_argument("--save", metavar="FILE", help="save animation (.gif, or .mp4 if ffmpeg is installed)")
    ap.add_argument("--png", metavar="FILE", help="save the final frame as an image")
    ap.add_argument("--no-show", action="store_true", help="do not open a window")
    ap.add_argument("--spin", action="store_true", help="slowly rotate the camera during playback")
    args = ap.parse_args()

    import matplotlib
    if args.no_show:
        matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from matplotlib.animation import FuncAnimation
    from mpl_toolkits.mplot3d.art3d import Poly3DCollection

    sim, path, frames, status = run_simulation(args)
    fig, ax, trail, title, static_faces, static_cols = build_scene(plt, Poly3DCollection, sim, path, frames, status, args)
    track, wheel_r = sim.config.track_width, sim.config.wheel_radius
    dynamic = []   # artists redrawn every frame
    history = []

    def draw(k):
        k = min(k, len(frames) - 1)
        fr = frames[k]
        for art in dynamic:
            art.remove()
        dynamic.clear()
        faces = list(static_faces)
        cols = list(static_cols)
        for pos, r in fr["obs"]:
            f = cylinder_faces(pos[0], pos[1], 0, 0.7, r, 16)
            faces += f; cols += ["#dc3c3c"] * len(f)
        for rf, colour in robot_faces(fr["pose"], track, wheel_r):
            faces += rf; cols += [colour] * len(rf)
        coll = Poly3DCollection(faces, facecolors=cols, edgecolors="#0f172a", linewidths=0.3)
        coll.set_zorder(2)
        dynamic.append(ax.add_collection3d(coll))
        history.append(fr["pose"][:2].copy())
        xs, ys = zip(*[frames[i]["pose"][:2] for i in range(k + 1)])
        trail.set_data(xs, ys)
        trail.set_3d_properties([0.04] * len(xs))
        last = k == len(frames) - 1
        title.set_text(f"t = {fr['t']:.1f} s" + (f"   -   {status}" if last else ""))
        if args.spin:
            ax.view_init(elev=38, azim=-62 + k * 0.6)
        return dynamic

    anim = FuncAnimation(fig, draw, frames=len(frames) + 8, interval=60, repeat=False, blit=False)

    if args.save:
        print(f"Saving animation to {args.save} ... (this can take a minute)")
        if args.save.lower().endswith(".gif"):
            from matplotlib.animation import PillowWriter
            anim.save(args.save, writer=PillowWriter(fps=15))
        else:
            anim.save(args.save, fps=15)
        print("Saved.")
    if args.png:
        draw(len(frames) - 1)
        fig.savefig(args.png, dpi=130, facecolor=fig.get_facecolor())
        print(f"Saved {args.png}")
    if not args.no_show:
        plt.show()


if __name__ == "__main__":
    main()
