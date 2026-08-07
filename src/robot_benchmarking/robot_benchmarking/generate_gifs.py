from __future__ import annotations

import math
from pathlib import Path
from typing import List, Tuple

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import matplotlib.patches as mpatches
from matplotlib.patches import Wedge, Rectangle, Circle, FancyArrowPatch
import numpy as np

try:
    from PIL import Image
    _HAS_PIL = True
except ImportError:
    _HAS_PIL = False

RESULTS_DIR = Path(__file__).resolve().parents[3] / 'results'
GIF_DIR = RESULTS_DIR / 'gifs'

BG        = '#ffffff'
AX_BG     = '#f8fafc'
GRID_COL  = '#e2e8f0'
WALL_COL  = '#64748b'
FLOOR_COL = '#f1f5f9'
ROBOT_COL = '#dc2626'
TRAJ_COL  = '#2563eb'
PATH_GREEN = '#16a34a'
PATH_RED   = '#dc2626'
PATH_BLUE  = '#2563eb'
FRONTIER_COL = '#d97706'
LIDAR_COL = '#3b82f6'
START_COL = '#16a34a'
GOAL_COL  = '#7c3aed'
TEXT_COL  = '#1e293b'
SUBTEXT   = '#475569'
DETECT_COL = '#16a34a'
OBS_COL   = '#dc2626'

def _style_ax(ax, title: str) -> None:
    """Consistent light-mode styling for an axis."""
    ax.set_facecolor(AX_BG)
    ax.set_title(title, color=TEXT_COL, fontsize=10, fontweight='bold', pad=7)
    ax.tick_params(colors=SUBTEXT, labelsize=7)
    for spine in ax.spines.values():
        spine.set_edgecolor('#cbd5e1')
        spine.set_linewidth(0.8)


def _grid(ax) -> None:
    """Faint background grid lines."""
    for v in range(-1, 12):
        ax.axvline(v, color=GRID_COL, linewidth=0.5, zorder=0)
        ax.axhline(v, color=GRID_COL, linewidth=0.5, zorder=0)


def _draw_robot(ax, x: float, y: float, heading: float, size: float = 0.25) -> None:
    """Circle body + heading arrow, white arrow so it reads on the red fill."""
    body = Circle((x, y), size, facecolor=ROBOT_COL, edgecolor='#7f1d1d',
                  linewidth=1.2, zorder=6)
    ax.add_patch(body)
    dx = math.cos(heading) * size * 1.7
    dy = math.sin(heading) * size * 1.7
    ax.annotate('', xy=(x + dx, y + dy), xytext=(x, y),
                arrowprops=dict(arrowstyle='->', color='white', lw=2.0),
                zorder=7)


def _info_box(ax, lines: List[str], loc: str = 'lower right') -> None:
    """White rounded text box for the on-frame stats readout."""
    text = '\n'.join(lines)
    props = dict(boxstyle='round,pad=0.5', facecolor='white', alpha=0.93,
                 edgecolor='#94a3b8', linewidth=1.0)
    kw = dict(transform=ax.transAxes, fontsize=7.5, fontfamily='monospace',
              color=TEXT_COL, bbox=props, zorder=10)
    if loc == 'lower right':
        ax.text(0.98, 0.02, text, va='bottom', ha='right', **kw)
    else:
        ax.text(0.02, 0.98, text, va='top', ha='left', **kw)


def _save_gif(frames: List[np.ndarray], path: Path, duration_ms: int = 80) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    if _HAS_PIL:
        pil_imgs = [Image.fromarray(np.uint8(f * 255)) for f in frames]
        pil_imgs[0].save(path, save_all=True, append_images=pil_imgs[1:],
                         duration=duration_ms, loop=0, optimize=True)
    else:
        print(f'PIL not found — skipping {path.name}.')


def _capture(fig) -> np.ndarray:
    fig.canvas.draw()
    return np.asarray(fig.canvas.buffer_rgba())[:, :, :3] / 255.0


# GIF 1 — exploration, fog-of-war reveal

def _make_exploration_frames() -> List[np.ndarray]:
    N_FRAMES = 60

    # lawnmower sweep through the office
    waypoints: List[Tuple[float, float]] = [
        (1.0, 1.0), (9.0, 1.0), (9.0, 3.5), (1.0, 3.5),
        (1.0, 6.0), (9.0, 6.0), (9.0, 8.5), (1.0, 8.5),
        (5.0, 5.0),
    ]
    seg = N_FRAMES // (len(waypoints) - 1)
    traj: List[Tuple[float, float]] = []
    for i in range(len(waypoints) - 1):
        xs = np.linspace(waypoints[i][0], waypoints[i + 1][0], seg)
        ys = np.linspace(waypoints[i][1], waypoints[i + 1][1], seg)
        traj += list(zip(xs, ys))
    while len(traj) < N_FRAMES:
        traj.append(traj[-1])

    walls = [
        (3.4, 0.0, 0.2, 5.2),
        (6.4, 4.8, 0.2, 5.2),
        (0.0, 5.0, 5.2, 0.2),
    ]
    room_labels = [
        (1.8,  7.5, 'Meeting room'),
        (7.5,  7.5, 'Office A'),
        (7.5,  2.0, 'Office B'),
        (1.8,  2.0, 'Lobby'),
    ]

    fig, ax = plt.subplots(figsize=(6, 6), facecolor=BG)
    frames = []
    lidar_range = 2.6

    for fi in range(N_FRAMES):
        ax.clear()
        ax.set_xlim(-0.3, 10.3)
        ax.set_ylim(-0.3, 10.3)
        ax.set_aspect('equal')
        _style_ax(ax, 'Autonomous Frontier Exploration — Office Environment')
        _grid(ax)

        rx, ry = traj[fi]
        coverage_pct = int(min(100, (fi + 1) / N_FRAMES * 100))

        # fake the fog-of-war by painting the whole floor "unknown" first,
        # then stamping AX_BG circles over anywhere the robot's already been
        unknown = Rectangle((-0.3, -0.3), 10.6, 10.6, facecolor='#e2e8f0',
                            alpha=0.55, zorder=0)
        ax.add_patch(unknown)

        visited_x = np.array([traj[k][0] for k in range(fi + 1)])
        visited_y = np.array([traj[k][1] for k in range(fi + 1)])
        reveal_r = lidar_range * (0.55 + 0.45 * (fi / N_FRAMES))
        for k in range(0, fi + 1, 3):
            cx, cy = traj[k]
            rev = Circle((cx, cy), lidar_range * 0.9, facecolor=AX_BG,
                          alpha=0.85, zorder=1)
            ax.add_patch(rev)

        floor = Rectangle((0, 0), 10, 10, facecolor=FLOOR_COL, alpha=0.3, zorder=0)
        ax.add_patch(floor)

        for wx, wy, ww, wh in walls:
            ax.add_patch(Rectangle((wx, wy), ww, wh, color=WALL_COL, zorder=3))

        # fade labels in as the robot gets close enough to have "seen" them
        for lx, ly, label in room_labels:
            dist = math.hypot(lx - rx, ly - ry)
            label_alpha = max(0.0, min(0.6, 1.0 - (dist - lidar_range) / 3.0))
            if label_alpha > 0:
                ax.text(lx, ly, label, fontsize=6.5, color=SUBTEXT, ha='center',
                        alpha=label_alpha, style='italic', zorder=4)

        lidar_angle_deg = (fi / N_FRAMES) * 720
        sweep = Wedge((rx, ry), lidar_range, lidar_angle_deg - 25,
                      lidar_angle_deg + 25,
                      facecolor=LIDAR_COL, alpha=0.12, zorder=2)
        ax.add_patch(sweep)
        lidar_ring = Circle((rx, ry), lidar_range, fill=False,
                             edgecolor=LIDAR_COL, linewidth=0.8, alpha=0.4, zorder=2)
        ax.add_patch(lidar_ring)

        # amber diamonds standing in for frontier candidates at the exploration edge
        n_frontiers = max(2, 9 - fi // 7)
        rng = np.random.default_rng(fi * 13 + 7)
        angles = rng.uniform(0, 2 * math.pi, n_frontiers)
        radii  = rng.uniform(lidar_range * 0.65, lidar_range * 1.05, n_frontiers)
        for a, r in zip(angles, radii):
            fx, fy = rx + r * math.cos(a), ry + r * math.sin(a)
            if -0.3 < fx < 10.3 and -0.3 < fy < 10.3:
                ax.plot(fx, fy, 'D', markersize=6, color=FRONTIER_COL,
                        markeredgecolor='#92400e', markeredgewidth=0.8,
                        alpha=0.9, zorder=5)

        trail_start = max(0, fi - 22)
        trail_x = [traj[k][0] for k in range(trail_start, fi + 1)]
        trail_y = [traj[k][1] for k in range(trail_start, fi + 1)]
        n_trail = len(trail_x)
        if n_trail > 1:
            alphas = np.linspace(0.08, 0.75, n_trail)
            for i in range(n_trail - 1):
                ax.plot(trail_x[i:i+2], trail_y[i:i+2],
                        color=TRAJ_COL, linewidth=2.0, alpha=float(alphas[i]), zorder=4)

        if fi < N_FRAMES - 1:
            ddx = traj[fi + 1][0] - rx
            ddy = traj[fi + 1][1] - ry
            heading = math.atan2(ddy, ddx) if abs(ddx) + abs(ddy) > 0.01 else 0.0
        else:
            heading = 0.0
        _draw_robot(ax, rx, ry, heading)

        leg_handles = [
            mpatches.Patch(color=FRONTIER_COL, label='Frontier target'),
            plt.Line2D([0], [0], color=TRAJ_COL, linewidth=2, label='Robot trajectory'),
            mpatches.Patch(facecolor='#e2e8f0', edgecolor='#94a3b8', label='Unknown space'),
            mpatches.Patch(facecolor=AX_BG, edgecolor='#94a3b8', label='Mapped free space'),
        ]
        ax.legend(handles=leg_handles, loc='upper right', fontsize=6.5,
                  facecolor='white', edgecolor='#cbd5e1', framealpha=0.92)

        _info_box(ax, [
            f'Map coverage  : {coverage_pct:3d}%',
            f'Sim time      : {fi * 0.5:.1f} s',
            f'Robot pose    : ({rx:.1f}, {ry:.1f})',
            f'Active fronts : {n_frontiers}',
            f'Strategy      : Boustrophedon',
        ])

        frames.append(_capture(fig))

    plt.close(fig)
    return frames


# GIF 2 — dynamic obstacle, A* replanning

def _make_navigation_frames() -> List[np.ndarray]:
    N_FRAMES     = 65
    OBSTACLE_FRM = 25
    REPLAN_FRM   = 31

    start = np.array([1.0, 1.0])
    goal  = np.array([9.0, 9.0])
    obstacle_pos = np.array([5.0, 5.0])

    orig_path = np.array([
        [1.0, 1.0], [2.5, 2.5], [4.0, 4.0],
        [5.0, 5.0], [6.5, 6.5], [8.0, 8.0], [9.0, 9.0]
    ])
    detour_path = np.array([
        [1.0, 1.0], [2.5, 1.8], [4.2, 2.4], [6.2, 3.2],
        [7.6, 5.2], [8.4, 7.2], [9.0, 9.0]
    ])

    def interp_path(path, n):
        pts = []
        segs = len(path) - 1
        per  = max(1, n // segs)
        for i in range(segs):
            xs = np.linspace(path[i, 0], path[i+1, 0], per)
            ys = np.linspace(path[i, 1], path[i+1, 1], per)
            pts += list(zip(xs, ys))
        while len(pts) < n:
            pts.append(pts[-1])
        return pts[:n]

    orig_traj   = interp_path(orig_path,   N_FRAMES)
    detour_traj = interp_path(detour_path, N_FRAMES)

    fig, ax = plt.subplots(figsize=(6, 6), facecolor=BG)
    frames  = []
    replans = 0

    for fi in range(N_FRAMES):
        ax.clear()
        ax.set_xlim(-0.3, 10.3)
        ax.set_ylim(-0.3, 10.3)
        ax.set_aspect('equal')
        _style_ax(ax, 'Dynamic Obstacle — Global Path Replanning (A*)')
        _grid(ax)

        ax.add_patch(Rectangle((0, 0), 10, 10, facecolor=FLOOR_COL, alpha=0.5, zorder=0))

        static_walls = [
            (0, 5.0, 4.2, 0.18),
            (5.8, 5.0, 4.2, 0.18),
            (5.0, 0.0, 0.18, 4.2),
        ]
        for wx, wy, ww, wh in static_walls:
            ax.add_patch(Rectangle((wx, wy), ww, wh, color=WALL_COL, zorder=2))

        obstacle_present = fi >= OBSTACLE_FRM
        replanned        = fi >= REPLAN_FRM
        if replanned and replans == 0:
            replans = 1

        if fi < REPLAN_FRM:
            rx, ry = orig_traj[fi]
        else:
            off = fi - REPLAN_FRM
            rx, ry = detour_traj[min(off, len(detour_traj) - 1)]

        nav_state = 'NAVIGATING'
        if OBSTACLE_FRM <= fi < REPLAN_FRM:
            nav_state = 'OBSTACLE DETECTED'
        elif replanned:
            nav_state = 'REPLANNED — NAVIGATING'

        path_color = PATH_GREEN if fi < OBSTACLE_FRM else PATH_RED
        ox = [p[0] for p in orig_traj]
        oy = [p[1] for p in orig_traj]
        lbl = 'Original path (A*)' + (' — BLOCKED' if fi >= OBSTACLE_FRM else '')
        ax.plot(ox, oy, '--', color=path_color, linewidth=2.0,
                alpha=0.85, zorder=3, label=lbl)

        if fi >= REPLAN_FRM:
            dx_pts = [p[0] for p in detour_traj]
            dy_pts = [p[1] for p in detour_traj]
            ax.plot(dx_pts, dy_pts, '-', color=PATH_BLUE, linewidth=2.4,
                    alpha=0.9, zorder=3, label='Replanned path (A*)')

        lc = Rectangle((rx - 1.5, ry - 1.5), 3.0, 3.0, fill=False,
                        edgecolor='#7c3aed', linewidth=1.3, linestyle=':', alpha=0.75, zorder=4)
        ax.add_patch(lc)
        ax.text(rx - 1.5, ry + 1.58, 'local costmap', fontsize=5.5,
                color='#7c3aed', alpha=0.85, zorder=5)

        ax.add_patch(Circle((rx, ry), 1.8, fill=False,
                             edgecolor=LIDAR_COL, linewidth=0.8, alpha=0.35, zorder=4))

        if obstacle_present:
            ax.add_patch(Rectangle((obstacle_pos[0] - 0.5, obstacle_pos[1] - 0.5),
                                    1.0, 1.0, facecolor=OBS_COL, edgecolor='#7f1d1d',
                                    alpha=0.85, linewidth=1.2, zorder=5))
            ax.text(obstacle_pos[0], obstacle_pos[1] + 0.72,
                    'OBSTACLE', fontsize=7, color=OBS_COL,
                    ha='center', fontweight='bold', zorder=6)

        if fi >= REPLAN_FRM:
            orig_tx = [p[0] for p in orig_traj[:REPLAN_FRM]]
            orig_ty = [p[1] for p in orig_traj[:REPLAN_FRM]]
            off     = fi - REPLAN_FRM
            det_tx  = [p[0] for p in detour_traj[:off + 1]]
            det_ty  = [p[1] for p in detour_traj[:off + 1]]
            ax.plot(orig_tx, orig_ty, color='#94a3b8', linewidth=1.5, alpha=0.5, zorder=3)
            ax.plot(det_tx,  det_ty,  color=TRAJ_COL, linewidth=2.0, alpha=0.75, zorder=3)
        else:
            tx = [p[0] for p in orig_traj[:fi + 1]]
            ty = [p[1] for p in orig_traj[:fi + 1]]
            ax.plot(tx, ty, color=TRAJ_COL, linewidth=2.0, alpha=0.6, zorder=3)

        ax.plot(*start, 'o', color=START_COL, markersize=13, zorder=7,
                markeredgecolor='white', markeredgewidth=1.5, label='Start')
        ax.plot(*goal,  '*', color=GOAL_COL,  markersize=17, zorder=7,
                markeredgecolor='white', markeredgewidth=1.0, label='Goal')

        if fi < N_FRAMES - 1:
            src = orig_traj[fi]     if fi < REPLAN_FRM     else detour_traj[max(0, fi - REPLAN_FRM)]
            nxt = orig_traj[fi + 1] if fi < REPLAN_FRM - 1 else detour_traj[min(fi - REPLAN_FRM + 1, len(detour_traj) - 1)]
            hdx, hdy = nxt[0] - src[0], nxt[1] - src[1]
            heading  = math.atan2(hdy, hdx) if abs(hdx) + abs(hdy) > 0.01 else math.pi / 4
        else:
            heading = math.pi / 4
        _draw_robot(ax, rx, ry, heading)

        dist_to_goal  = math.hypot(rx - goal[0], ry - goal[1])
        min_clearance = (math.hypot(rx - obstacle_pos[0], ry - obstacle_pos[1]) - 0.5
                         if obstacle_present else 3.2)

        ax.legend(loc='upper left', fontsize=6.5, facecolor='white',
                  edgecolor='#cbd5e1', framealpha=0.92)
        _info_box(ax, [
            f'State         : {nav_state}',
            f'Planner       : A* (Grid Search)',
            f'Dist to goal  : {max(0.0, dist_to_goal):.2f} m',
            f'Min clearance : {max(0.0, min_clearance):.2f} m',
            f'Replans       : {replans}',
            f'Sim time      : {fi * 0.5:.1f} s',
        ])

        frames.append(_capture(fig))

    plt.close(fig)
    return frames


# GIF 3 — semantic navigation, dual panel

def _make_semantic_frames() -> List[np.ndarray]:
    N_FRAMES    = 60
    DETECT_START = 18
    NAV_START    = 35

    start_pos = np.array([1.0, 1.0])
    goal_pos  = np.array([7.5, 7.0])

    nav_wp = np.array([
        [1.0, 1.0], [2.0, 2.0], [3.5, 3.0],
        [5.0, 4.5], [6.5, 6.0], [7.5, 7.0]
    ])
    traj_pts: List[Tuple[float, float]] = []
    for i in range(len(nav_wp) - 1):
        xs = np.linspace(nav_wp[i, 0], nav_wp[i+1, 0], 6)
        ys = np.linspace(nav_wp[i, 1], nav_wp[i+1, 1], 6)
        traj_pts += list(zip(xs, ys))
    while len(traj_pts) < N_FRAMES - NAV_START + 5:
        traj_pts.append(traj_pts[-1])

    fig, axes = plt.subplots(1, 2, figsize=(12, 6), facecolor=BG)
    fig.subplots_adjust(wspace=0.06)
    ax_cam, ax_map = axes
    frames = []

    for fi in range(N_FRAMES):
        for ax in axes:
            ax.clear()

        # left: camera panel
        ax_cam.set_xlim(0, 640)
        ax_cam.set_ylim(0, 480)
        ax_cam.invert_yaxis()
        ax_cam.set_facecolor('#f8fafc')
        ax_cam.set_title('RGB-D Camera — Object Detection',
                          color=TEXT_COL, fontsize=10, fontweight='bold', pad=7)
        ax_cam.set_xticks([])
        ax_cam.set_yticks([])
        for spine in ax_cam.spines.values():
            spine.set_edgecolor('#cbd5e1')

        ax_cam.add_patch(Rectangle((60,  120), 180, 100, color='#d6d3d1', zorder=1))  # desk 1
        ax_cam.add_patch(Rectangle((380, 200), 160, 100, color='#d6d3d1', zorder=1))  # desk 2
        ax_cam.add_patch(Rectangle((300, 300),  70,  70, color='#e2e8f0', zorder=1))  # chair
        ax_cam.add_patch(Rectangle((298,  98), 144, 122, facecolor='#dcfce7',   # workstation
                                    edgecolor='#86efac', linewidth=1.5, zorder=1))
        ax_cam.add_patch(Rectangle((328,  68),  82,  38, facecolor='#1e293b',   # monitor
                                    edgecolor='#475569', linewidth=1, zorder=2))
        ax_cam.text(340, 85, 'MONITOR', fontsize=6, ha='center', va='center',
                    color='#94a3b8', zorder=3)
        ax_cam.text(150, 172, 'Desk', fontsize=8, color=SUBTEXT, ha='center', zorder=3, style='italic')
        ax_cam.text(460, 252, 'Desk', fontsize=8, color=SUBTEXT, ha='center', zorder=3, style='italic')
        ax_cam.text(335, 337, 'Chair', fontsize=7, color=SUBTEXT, ha='center', zorder=3, style='italic')

        sweep_alpha = 0.55 if fi < DETECT_START else 0.2
        scan_y = int((fi % 20) / 20 * 480)
        ax_cam.axhline(scan_y, color=LIDAR_COL, linewidth=1.2, alpha=sweep_alpha, zorder=4)

        detect_alpha = max(0.0, min(1.0, (fi - DETECT_START) / 10.0))
        confidence   = min(0.97, max(0.0, (fi - DETECT_START) / 18.0))

        if detect_alpha > 0:
            wobble = max(0, 7 - (fi - DETECT_START)) * math.sin(fi * 0.9) * 2
            bx, by = 294 + wobble * 0.5, 90 + wobble * 0.3
            bw, bh = 152, 138
            ax_cam.add_patch(Rectangle((bx, by), bw, bh, fill=False,
                                        edgecolor=DETECT_COL, linewidth=2.5,
                                        alpha=detect_alpha, zorder=5))
            tag_bg = Rectangle((bx - 1, by - 20), 130, 20,
                                facecolor=DETECT_COL, alpha=detect_alpha * 0.9, zorder=6)
            ax_cam.add_patch(tag_bg)
            ax_cam.text(bx + 3, by - 5,
                        f'workstation   {confidence:.0%}',
                        fontsize=8.5, color='white', fontweight='bold',
                        alpha=detect_alpha, zorder=7)
            bar_y = by + bh + 6
            ax_cam.add_patch(Rectangle((bx, bar_y), bw, 9,
                                        color='#e2e8f0', alpha=detect_alpha, zorder=6))
            ax_cam.add_patch(Rectangle((bx, bar_y), bw * confidence, 9,
                                        color=DETECT_COL, alpha=detect_alpha, zorder=7))
            ax_cam.text(bx + bw + 5, bar_y + 8,
                        f'{confidence:.0%}', fontsize=8, color=DETECT_COL,
                        alpha=detect_alpha, fontweight='bold', zorder=8)

        cam_state = 'Scanning environment...'
        if fi >= DETECT_START:
            cam_state = 'Object detected'
        if fi >= NAV_START:
            cam_state = 'Target locked — navigating'

        _info_box(ax_cam, [
            f'State      : {cam_state}',
            f'Target     : workstation',
            f'Confidence : {confidence:.0%}',
            f'Model      : YOLOv8-custom',
        ], loc='lower right')

        # right: top-down map panel
        ax_map.set_xlim(-0.3, 10.3)
        ax_map.set_ylim(-0.3, 10.3)
        ax_map.set_aspect('equal')
        _style_ax(ax_map, 'Top-Down Map — Semantic Navigation')
        _grid(ax_map)

        ax_map.add_patch(Rectangle((0, 0), 10, 10, facecolor=FLOOR_COL,
                                    alpha=0.5, zorder=0))

        for rx_, ry_, rw, rh, lbl in [
            (1.0, 6.2, 2.2, 1.3, 'desk'),
            (5.0, 1.0, 2.0, 0.9, 'desk'),
            (7.0, 7.0, 1.8, 1.8, 'workstation'),
        ]:
            col = '#bbf7d0' if lbl == 'workstation' else '#e2e8f0'
            edge = '#86efac' if lbl == 'workstation' else '#cbd5e1'
            ax_map.add_patch(Rectangle((rx_, ry_), rw, rh, facecolor=col,
                                        edgecolor=edge, linewidth=1.2, zorder=1))
            ax_map.text(rx_ + rw / 2, ry_ + rh / 2, lbl,
                        fontsize=6, color=SUBTEXT, ha='center', va='center',
                        style='italic', zorder=2)

        ax_map.add_patch(Circle(goal_pos, 0.42, facecolor=GOAL_COL, edgecolor='#5b21b6',
                                 alpha=0.85, linewidth=1.2, zorder=4))
        ax_map.text(goal_pos[0], goal_pos[1] + 0.72, 'Workstation',
                    fontsize=8, color=GOAL_COL, ha='center', fontweight='bold', zorder=5)

        # path only shows up once "detection" has landed on the target
        path_alpha = max(0.0, min(1.0, (fi - DETECT_START + 2) / 8.0))
        if path_alpha > 0:
            px = [1.0, 2.0, 3.5, 5.0, 6.5, 7.5]
            py = [1.0, 2.0, 3.0, 4.5, 6.0, 7.0]
            ax_map.plot(px, py, '--', color=PATH_GREEN, linewidth=2.4,
                        alpha=path_alpha * 0.9, zorder=3, label='A* path')

        if fi >= NAV_START:
            nav_fi   = min(fi - NAV_START, len(traj_pts) - 1)
            robot_x, robot_y = traj_pts[nav_fi]
        else:
            robot_x, robot_y = start_pos

        if fi > NAV_START:
            n   = min(fi - NAV_START + 1, len(traj_pts))
            tx  = [traj_pts[k][0] for k in range(n)]
            ty  = [traj_pts[k][1] for k in range(n)]
            ax_map.plot(tx, ty, color=TRAJ_COL, linewidth=2.0, alpha=0.7, zorder=3)

        ax_map.add_patch(Circle((robot_x, robot_y), 1.8, fill=False,
                                 edgecolor=LIDAR_COL, linewidth=0.8, alpha=0.35, zorder=4))

        ax_map.plot(*start_pos, 'o', color=START_COL, markersize=12, zorder=6,
                    markeredgecolor='white', markeredgewidth=1.5, label='Start')

        if fi >= NAV_START and fi < N_FRAMES - 1:
            ni  = min(fi - NAV_START + 1, len(traj_pts) - 1)
            ci  = max(0, ni - 1)
            hdx = traj_pts[ni][0] - traj_pts[ci][0]
            hdy = traj_pts[ni][1] - traj_pts[ci][1]
            heading = math.atan2(hdy, hdx) if abs(hdx) + abs(hdy) > 0.01 else math.pi / 4
        else:
            heading = math.pi / 4
        _draw_robot(ax_map, robot_x, robot_y, heading)

        dist_to_ws = math.hypot(robot_x - goal_pos[0], robot_y - goal_pos[1])
        nav_status = 'IDLE'
        if fi >= DETECT_START:
            nav_status = 'GOAL SET'
        if fi >= NAV_START:
            nav_status = 'NAVIGATING'
        if dist_to_ws < 0.5 and fi >= NAV_START:
            nav_status = 'GOAL REACHED'

        ax_map.legend(loc='upper left', fontsize=6.5, facecolor='white',
                      edgecolor='#cbd5e1', framealpha=0.92)
        _info_box(ax_map, [
            f'Command    : "Go to workstation"',
            f'Target     : Workstation (7.5, 7.0)',
            f'Planner    : A* Grid Search',
            f'Nav status : {nav_status}',
            f'Dist       : {dist_to_ws:.2f} m',
            f'Sim time   : {fi * 0.5:.1f} s',
        ])

        fig.tight_layout(pad=0.6)
        frames.append(_capture(fig))

    plt.close(fig)
    return frames


# entry point

def generate_exploration_gif() -> Path:
    out = GIF_DIR / 'exploration_demo.gif'
    print('  Rendering exploration frames...')
    _save_gif(_make_exploration_frames(), out, duration_ms=80)
    return out


def generate_navigation_gif() -> Path:
    out = GIF_DIR / 'navigation_replanning_demo.gif'
    print('  Rendering navigation/replanning frames...')
    _save_gif(_make_navigation_frames(), out, duration_ms=80)
    return out


def generate_semantic_gif() -> Path:
    out = GIF_DIR / 'semantic_navigation_demo.gif'
    print('  Rendering semantic navigation frames...')
    _save_gif(_make_semantic_frames(), out, duration_ms=80)
    return out


def main() -> None:
    print('Generating three distinct light-mode animated GIF visualisations...')
    for p in [generate_exploration_gif(), generate_navigation_gif(), generate_semantic_gif()]:
        print(f'GIF written: {p}')


if __name__ == '__main__':
    main()
