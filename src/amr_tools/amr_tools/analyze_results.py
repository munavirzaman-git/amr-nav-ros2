"""Summarise benchmark CSVs into markdown tables and comparison plots.

  ros2 run amr_tools analyze_results ~/amr_benchmarks
"""
import argparse
import csv
import glob
import math
import os
import statistics
from collections import defaultdict


def fnum(v):
    try:
        x = float(v)
        return None if math.isnan(x) else x
    except (TypeError, ValueError):
        return None


def mean(xs):
    xs = [x for x in xs if x is not None]
    return statistics.mean(xs) if xs else float('nan')


def read_rows(paths):
    nav, plan = [], []
    for p in paths:
        with open(p) as f:
            reader = csv.DictReader(f)
            cols = set(reader.fieldnames or [])
            if {'planner', 'query', 'success'} <= cols:
                plan.extend(reader)
            elif {'label', 'goal', 'success'} <= cols:
                nav.extend(reader)
            else:
                print(f'skipping {p}: not a benchmark CSV')
    return nav, plan


def nav_table(rows):
    by = defaultdict(list)
    for r in rows:
        by[r['label']].append(r)
    hdr = ('| stack | runs | success % | time [s] | path [m] | speed [m/s] | min obst. dist [m] '
           '| mean abs(dw/dt) [rad/s²] | recoveries | final err [m] |\n'
           '|---|---|---|---|---|---|---|---|---|---|\n')
    lines, summary = [], {}
    for label, rs in sorted(by.items()):
        ok = [r for r in rs if r['success'] == '1']
        s = {
            'runs': len(rs), 'success': 100.0 * len(ok) / len(rs),
            'time': mean(fnum(r['time_s']) for r in ok),
            'path': mean(fnum(r['path_length_m']) for r in ok),
            'speed': mean(fnum(r['mean_speed_mps']) for r in ok),
            'clear': min([x for x in (fnum(r['min_obstacle_dist_m']) for r in rs) if x is not None]
                         or [float('nan')]),
            'jerk': mean(fnum(r['mean_abs_dw_dt']) for r in ok),
            'rec': sum(int(fnum(r['recoveries']) or 0) for r in rs),
            'err': mean(fnum(r['final_pos_err_m']) for r in ok),
        }
        summary[label] = s
        lines.append(f"| {label} | {s['runs']} | {s['success']:.0f} | {s['time']:.1f} | "
                     f"{s['path']:.2f} | {s['speed']:.2f} | {s['clear']:.2f} | {s['jerk']:.2f} | "
                     f"{s['rec']} | {s['err']:.3f} |")
    return hdr + '\n'.join(lines) + '\n', summary


def plan_table(rows):
    by = defaultdict(list)
    for r in rows:
        by[r['planner']].append(r)
    hdr = ('| planner | queries | success % | wall time [ms] (median) | planner time [ms] '
           '| length [m] | total turning [rad] | max cost (0-100) | mean cost |\n'
           '|---|---|---|---|---|---|---|---|---|\n')
    lines, summary = [], {}
    for name, rs in sorted(by.items()):
        ok = [r for r in rs if r['success'] == '1']
        wall = [fnum(r['wall_ms']) for r in ok if fnum(r['wall_ms']) is not None]
        s = {
            'n': len(rs), 'success': 100.0 * len(ok) / len(rs),
            'wall': statistics.median(wall) if wall else float('nan'),
            'ptime': mean(fnum(r['planner_ms']) for r in ok),
            'len': mean(fnum(r['length_m']) for r in ok),
            'turn': mean(fnum(r['total_turn_rad']) for r in ok),
            'maxc': mean(fnum(r['max_cost']) for r in ok),
            'meanc': mean(fnum(r['mean_cost']) for r in ok),
        }
        summary[name] = s
        lines.append(f"| {name} | {s['n']} | {s['success']:.0f} | {s['wall']:.1f} | "
                     f"{s['ptime']:.2f} | {s['len']:.2f} | {s['turn']:.2f} | {s['maxc']:.1f} | "
                     f"{s['meanc']:.1f} |")
    return hdr + '\n'.join(lines) + '\n', summary


def plot(summary, keys, title, path):
    try:
        import matplotlib
        matplotlib.use('Agg')
        import matplotlib.pyplot as plt
    except ImportError:
        return False
    labels = list(summary.keys())
    fig, axes = plt.subplots(1, len(keys), figsize=(4 * len(keys), 3.6))
    if len(keys) == 1:
        axes = [axes]
    for ax, (k, name) in zip(axes, keys):
        ax.bar(labels, [summary[lbl][k] for lbl in labels], color='#4C78A8')
        ax.set_title(name, fontsize=10)
        ax.tick_params(axis='x', rotation=30, labelsize=8)
    fig.suptitle(title)
    fig.tight_layout()
    fig.savefig(path, dpi=130)
    plt.close(fig)
    return True


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('inputs', nargs='*', default=[os.path.expanduser('~/amr_benchmarks')],
                    help='CSV files or directories containing them')
    ap.add_argument('--out', default=None, help='output directory (default: first input dir)')
    args, _ = ap.parse_known_args()

    files = []
    for i in args.inputs:
        files += sorted(glob.glob(os.path.join(i, '*.csv'))) if os.path.isdir(i) else [i]
    if not files:
        print('no CSV files found')
        return
    out_dir = args.out or (args.inputs[0] if os.path.isdir(args.inputs[0])
                           else os.path.dirname(os.path.abspath(args.inputs[0])))
    os.makedirs(out_dir, exist_ok=True)

    nav_rows, plan_rows = read_rows(files)
    md = '# AMR navigation benchmark\n\n'
    if plan_rows:
        t, s = plan_table(plan_rows)
        md += '## Global planners (same queries, same costmap)\n\n' + t + '\n'
        if plot(s, [('wall', 'median wall time [ms]'), ('len', 'path length [m]'),
                    ('turn', 'total turning [rad]'), ('meanc', 'mean cost along path')],
                'Global planner comparison', os.path.join(out_dir, 'planners.png')):
            md += '![planners](planners.png)\n\n'
    if nav_rows:
        t, s = nav_table(nav_rows)
        md += '## End-to-end navigation (planner + controller)\n\n' + t + '\n'
        if plot(s, [('success', 'success [%]'), ('time', 'time to goal [s]'),
                    ('clear', 'min obstacle distance [m]'), ('jerk', 'mean |dω/dt|')],
                'Navigation stack comparison', os.path.join(out_dir, 'navigation.png')):
            md += '![navigation](navigation.png)\n'
    with open(os.path.join(out_dir, 'summary.md'), 'w') as f:
        f.write(md)
    print(md)
    print(f'written {os.path.join(out_dir, "summary.md")}')


if __name__ == '__main__':
    main()
