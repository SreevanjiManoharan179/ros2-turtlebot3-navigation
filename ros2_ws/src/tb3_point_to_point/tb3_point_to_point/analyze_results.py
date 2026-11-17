"""Summarise navigation runs into tables (Markdown / CSV) and figures.

    ros2 run tb3_point_to_point analyze_results results/evaluation_20261117_101500
    ros2 run tb3_point_to_point analyze_results results/run_a results/run_b --out results/combined

Input: result folders written by mission_runner (run.json, trials.csv, map.yaml).
Output (in --out, default: the first folder): summary.md, summary.csv,
trajectories.png (paths drawn on the map) and metrics.png (bar charts).
"""
import argparse
import csv
import json
import math
import os
import sys

from . import metrics as M
from .map_tools import OccupancyMap

TABLE_FIELDS = [
    ('trial_id', 'Test'), ('route', 'Route'), ('result', 'Result'),
    ('navigation_time_s', 'Time [s]'), ('path_length_m', 'Driven [m]'),
    ('planned_path_length_m', 'Planned [m]'), ('path_efficiency', 'Efficiency'),
    ('final_position_error_m', 'Pos. error [m]'), ('final_yaw_error_deg', 'Yaw error [deg]'),
    ('recoveries', 'Recoveries'), ('min_clearance_m', 'Min clearance [m]'),
]


def _num(v):
    try:
        f = float(v)
        return None if math.isnan(f) else f
    except (TypeError, ValueError):
        return None


def _fmt(v, d=2):
    f = _num(v)
    return '-' if f is None else f'{f:.{d}f}'


def load_run(path):
    """Return {'dir', 'trials', 'map_yaml', 'meta'} for a results folder / json / csv."""
    if os.path.isdir(path):
        folder = path
        rj = os.path.join(path, 'run.json')
        if os.path.exists(rj):
            path = rj
        else:
            path = os.path.join(path, 'trials.csv')
    else:
        folder = os.path.dirname(os.path.abspath(path))
    if path.endswith('.json'):
        with open(path) as f:
            meta = json.load(f)
        trials = meta.get('trials', [])
        map_yaml = meta.get('map_yaml')
    else:
        with open(path, newline='') as f:
            trials = list(csv.DictReader(f))
        meta, map_yaml = {}, 'map.yaml'
    if map_yaml and not os.path.isabs(map_yaml):
        map_yaml = os.path.join(folder, map_yaml)
    if map_yaml and not os.path.exists(map_yaml):
        map_yaml = None
    return {'dir': folder, 'trials': trials, 'map_yaml': map_yaml, 'meta': meta}


def _route(t):
    start = t.get('start') or 'current'
    goal = t.get('goal') or t.get('label') or '?'
    obstacle = ' (+obstacle)' if t.get('obstacle') else ''
    return f'{start} -> {goal}{obstacle}'


def _trial_name(t, many_repeats):
    tid = str(t.get('trial_id') or t.get('label') or '?')
    rep = t.get('repeat')
    return f'{tid}.{rep}' if many_repeats and rep not in (None, '') else tid


def write_tables(trials, summary, out_dir, sources):
    many = len({str(t.get('repeat')) for t in trials}) > 1
    with open(os.path.join(out_dir, 'summary.csv'), 'w', newline='') as f:
        w = csv.writer(f)
        w.writerow([h for _, h in TABLE_FIELDS])
        for t in trials:
            w.writerow([_trial_name(t, many) if k == 'trial_id' else
                        (_route(t) if k == 'route' else
                         (t.get(k) if k == 'result' else _fmt(t.get(k), 3)))
                        for k, _ in TABLE_FIELDS])

    lines = ['# Point-to-point navigation - results', '']
    lines.append(f'Source run(s): {", ".join(os.path.basename(os.path.normpath(s)) for s in sources)}  ')
    lines.append(f'Goals: **{summary["trials"]}**, reached: **{summary["successful"]}**, '
                 f'success rate: **{summary["success_rate_pct"]:.0f} %**')
    lines += ['', '## Per-goal results', '']
    lines.append('| ' + ' | '.join(h for _, h in TABLE_FIELDS) + ' |')
    lines.append('|' + '|'.join(['---'] * len(TABLE_FIELDS)) + '|')
    for t in trials:
        cells = []
        for k, _ in TABLE_FIELDS:
            if k == 'trial_id':
                cells.append(_trial_name(t, many))
            elif k == 'route':
                cells.append(_route(t))
            elif k == 'result':
                cells.append(str(t.get('result', '-')))
            elif k in ('final_yaw_error_deg', 'navigation_time_s'):
                cells.append(_fmt(t.get(k), 1))
            elif k == 'recoveries':
                cells.append(_fmt(t.get(k), 0))
            else:
                cells.append(_fmt(t.get(k), 3 if 'error' in k else 2))
        lines.append('| ' + ' | '.join(cells) + ' |')

    lines += ['', '## Aggregate (successful goals)', '', '| Metric | Mean | Std |', '|---|---|---|']
    labels = {
        'final_position_error_m': 'Final position error [m]',
        'final_yaw_error_deg': 'Final yaw error [deg]',
        'navigation_time_s': 'Navigation time [s]',
        'path_length_m': 'Driven path length [m]',
        'planned_path_length_m': 'Planned path length [m]',
        'straight_line_m': 'Straight-line distance [m]',
        'path_efficiency': 'Path efficiency (planned / driven)',
        'average_speed_mps': 'Average speed [m/s]',
        'recoveries': 'Recovery behaviours per goal',
        'min_clearance_m': 'Minimum clearance to obstacles [m]',
        'localization_error_m': 'Localisation error (AMCL vs truth) [m]',
    }
    for key, label in labels.items():
        s = summary.get(key) or {}
        if s.get('mean') is not None:
            d = 1 if 'deg' in key or 'time' in key else 3
            lines.append(f'| {label} | {s["mean"]:.{d}f} | {s["std"]:.{d}f} |')

    lines += ['', '## Definitions', '',
              '- **Position error** E_p = sqrt((x_g - x_f)^2 + (y_g - y_f)^2): goal vs final pose. '
              'In Gazebo Classic the final pose is the simulator ground truth (/odom = world pose); '
              'otherwise the AMCL estimate is used (column `ground_truth` in trials.csv).',
              '- **Yaw error**: |wrap(yaw_g - yaw_f)|.',
              '- **Driven path length** L = sum of distances between consecutive recorded poses.',
              '- **Planned path length**: length of the Nav2 global plan computed before the robot moves.',
              '- **Path efficiency** = planned length / driven length (1.0 = drove exactly the planned distance).',
              '- **Success rate** = goals reached / goals attempted x 100 %.',
              '- **Min clearance** = closest LiDAR range during the run minus the robot radius (0.105 m).',
              '- Goals are accepted by Nav2 within xy_goal_tolerance / yaw_goal_tolerance '
              '(config/nav2_params_*.yaml), so small final errors are expected by design.']
    with open(os.path.join(out_dir, 'summary.md'), 'w') as f:
        f.write('\n'.join(lines) + '\n')


def write_plots(runs, trials, out_dir, map_yaml=None, locations_file=None):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt

    many = len({str(t.get('repeat')) for t in trials}) > 1
    names = [_trial_name(t, many) for t in trials]
    ok = [str(t.get('success')).lower() in ('true', '1') for t in trials]
    colors = plt.cm.tab10.colors

    # ---------------------------------------------------------- trajectories
    map_yaml = map_yaml or next((r['map_yaml'] for r in runs if r['map_yaml']), None)
    fig, ax = plt.subplots(figsize=(12, 8.5))
    if map_yaml:
        grid = OccupancyMap.from_yaml(map_yaml)
        x0, y0, x1, y1 = grid.extent()
        ax.imshow(grid.image_rows, cmap='gray', vmin=0, vmax=255, extent=(x0, x1, y0, y1),
                  origin='upper', interpolation='nearest')
    if locations_file and os.path.exists(locations_file):
        try:
            from .locations import LocationBook
            book = LocationBook(locations_file)
            for rname, (rx0, ry0, rx1, ry1) in book.rooms.items():
                ax.text((rx0 + rx1) / 2, (ry0 + ry1) / 2 if rname == 'Corridor' else ry1 - 0.35, rname,
                        ha='center', va='center', fontsize=9, color='0.35', alpha=0.9)
        except Exception:
            pass
    drawn = set()
    for t in trials:   # obstacles placed during a trial (not in the map): orange boxes
        box = t.get('obstacle_box')
        if isinstance(box, str):
            try:
                box = json.loads(box)
            except ValueError:
                box = None
        if not box or len(box) < 5 or tuple(box) in drawn:
            continue
        drawn.add(tuple(box))
        from matplotlib.patches import Rectangle
        bx, by, byaw, sx, sy = (float(v) for v in box[:5])
        c, s = math.cos(math.radians(byaw)), math.sin(math.radians(byaw))
        corner = (bx - (sx / 2) * c + (sy / 2) * s, by - (sx / 2) * s - (sy / 2) * c)
        ax.add_patch(Rectangle(corner, sx, sy, angle=byaw, facecolor=(1.0, 0.45, 0.0), edgecolor='k',
                               lw=0.8, alpha=0.9, zorder=3))
        ax.text(bx - sx / 2 - 0.08, by, 'obstacle\n(not in map)', ha='right', va='center', fontsize=8,
                color=(0.75, 0.3, 0.0), zorder=4)
    for i, t in enumerate(trials):
        traj = t.get('trajectory') or []
        if len(traj) < 2:
            continue
        xs, ys = [p[0] for p in traj], [p[1] for p in traj]
        c = colors[i % len(colors)]
        ax.plot(xs, ys, '-', color=c, lw=2.0, alpha=0.9, label=f'{names[i]}: {_route(t)}')
        ax.plot(xs[0], ys[0], 'o', color=c, ms=7, mec='k', mew=0.6)
        gx, gy = _num(t.get('goal_x')), _num(t.get('goal_y'))
        if gx is not None:
            ax.plot(gx, gy, '*', color=c, ms=15, mec='k', mew=0.6)
    ax.set_aspect('equal')
    ax.set_xlabel('x [m]')
    ax.set_ylabel('y [m]')
    ax.set_title('Driven trajectories (o = start, * = goal)')
    if any(len(t.get('trajectory') or []) > 1 for t in trials):
        ax.legend(loc='upper center', bbox_to_anchor=(0.5, -0.08), ncol=2, fontsize=8, frameon=False)
    fig.savefig(os.path.join(out_dir, 'trajectories.png'), dpi=120, bbox_inches='tight')
    plt.close(fig)

    # --------------------------------------------------------------- metrics
    fig, axs = plt.subplots(2, 2, figsize=(13, 8))
    bar_c = ['tab:green' if k else 'tab:red' for k in ok]
    xs = range(len(trials))

    def vals(key, scale=1.0):
        return [(_num(t.get(key)) or 0.0) * scale for t in trials]

    axs[0, 0].bar(xs, vals('navigation_time_s'), color=bar_c)
    axs[0, 0].set_title('Navigation time [s]')
    w = 0.4
    axs[0, 1].bar([x - w / 2 for x in xs], vals('planned_path_length_m'), w, label='planned', color='tab:blue')
    axs[0, 1].bar([x + w / 2 for x in xs], vals('path_length_m'), w, label='driven', color='tab:orange')
    axs[0, 1].set_title('Path length [m]')
    axs[0, 1].legend(fontsize=8)
    axs[1, 0].bar(xs, vals('final_position_error_m', 100.0), color=bar_c)
    axs[1, 0].set_title('Final position error [cm]')
    axs[1, 1].bar(xs, vals('path_efficiency'), color=bar_c)
    axs[1, 1].axhline(1.0, color='k', lw=0.8, ls='--')
    axs[1, 1].set_ylim(0, 1.15)
    axs[1, 1].set_title('Path efficiency (planned / driven)')
    for a in axs.flat:
        a.set_xticks(list(xs))
        a.set_xticklabels(names, rotation=0, fontsize=8)
        a.grid(axis='y', alpha=0.3)
    s = M.summarize(trials)
    fig.suptitle(f'Point-to-point navigation: {s["successful"]}/{s["trials"]} goals reached '
                 f'({s["success_rate_pct"]:.0f} %)   green = reached, red = failed')
    fig.tight_layout()
    fig.savefig(os.path.join(out_dir, 'metrics.png'), dpi=110)
    plt.close(fig)


def write_report(paths, out_dir=None, quiet=False, map_yaml=None, locations_file=None):
    runs = [load_run(p) for p in paths]
    trials = [t for r in runs for t in r['trials']]
    out_dir = out_dir or runs[0]['dir']
    os.makedirs(out_dir, exist_ok=True)
    summary = M.summarize(trials)
    write_tables(trials, summary, out_dir, [r['dir'] for r in runs])
    if locations_file is None:
        locations_file = next((r['meta'].get('locations_file') for r in runs if r['meta'].get('locations_file')), None)
    try:
        write_plots(runs, trials, out_dir, map_yaml, locations_file)
        plots = True
    except ImportError:
        plots = False
    if not quiet:
        print(open(os.path.join(out_dir, 'summary.md')).read())
        print(f'written: {out_dir}/summary.md, summary.csv' + (', trajectories.png, metrics.png' if plots else
                                                                ' (install matplotlib for the figures)'))
    return summary


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('runs', nargs='+', help='result folders (or run.json / trials.csv files)')
    ap.add_argument('--out', default=None, help='output folder (default: first run folder)')
    ap.add_argument('--map', default=None, help='map YAML for the trajectory plot (default: copy in the run)')
    ap.add_argument('--locations', default=None, help='locations YAML (room names on the plot)')
    args = ap.parse_args(sys.argv[1:] if argv is None else argv)
    write_report(args.runs, args.out, map_yaml=args.map, locations_file=args.locations)
    return 0


if __name__ == '__main__':
    sys.exit(main())
