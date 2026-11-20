"""Quantitative SLAM map evaluation against the ground-truth map.

Both maps live in the same frame: the world generator puts the ground truth in the
Gazebo world frame, and SLAM Toolbox anchors its map at the robot's start pose,
which is the world origin. No registration is therefore needed.

Metrics (on occupied cells)
  accuracy    mean / RMS / 95th-percentile distance from each SLAM wall cell to the
              nearest ground-truth wall cell
  precision   share of SLAM wall cells within --tol of a ground-truth wall
  recall      share of ground-truth wall cells within --tol of a SLAM wall
              (only walls the LiDAR can see; interiors of thick boxes are skipped)
  f1          harmonic mean of the two

  ros2 run amr_tools compare_maps --slam ~/maps/warehouse_slam.yaml --out results/map_eval
"""
import argparse
import json
import os

import numpy as np
import yaml


def load_map(yaml_path):
    with open(yaml_path) as f:
        meta = yaml.safe_load(f)
    img_path = meta['image']
    if not os.path.isabs(img_path):
        img_path = os.path.join(os.path.dirname(os.path.abspath(yaml_path)), img_path)
    with open(img_path, 'rb') as f:
        data = f.read()
    # minimal P5 PGM parser (handles comment lines)
    tokens, i = [], 0
    while len(tokens) < 4:
        while data[i:i + 1].isspace():
            i += 1
        if data[i:i + 1] == b'#':
            while data[i:i + 1] not in (b'\n', b''):
                i += 1
            continue
        j = i
        while not data[j:j + 1].isspace():
            j += 1
        tokens.append(data[i:j])
        i = j
    if tokens[0] != b'P5':
        raise ValueError(f'{img_path}: only binary PGM (P5) is supported')
    w, h, maxval = int(tokens[1]), int(tokens[2]), int(tokens[3])
    img = np.frombuffer(data[i + 1:i + 1 + w * h], dtype=np.uint8).reshape(h, w)
    if meta.get('negate', 0):
        p = img.astype(float) / maxval
    else:
        p = (maxval - img.astype(float)) / maxval
    occ = p > float(meta.get('occupied_thresh', 0.65))
    free = p < float(meta.get('free_thresh', 0.25))
    res = float(meta['resolution'])
    ox, oy = float(meta['origin'][0]), float(meta['origin'][1])
    rows, cols = np.nonzero(occ)
    y = oy + (h - 1 - rows + 0.5) * res   # image row 0 is the top (max y)
    x = ox + (cols + 0.5) * res
    return {'pts': np.stack([x, y], axis=1), 'img': img, 'res': res,
            'extent': [ox, ox + w * res, oy, oy + h * res], 'occ': occ, 'free': free}


def boundary_points(m):
    """Occupied cells with a FREE 4-neighbour: the wall faces a LiDAR inside the map can see.

    Excludes the filled interior of thick walls/boxes and faces bordering unknown space
    (e.g. the outside of the outer wall), which SLAM can never observe.
    """
    occ, free = m['occ'], m['free']
    pad = np.pad(free, 1, constant_values=False)
    touches_free = pad[:-2, 1:-1] | pad[2:, 1:-1] | pad[1:-1, :-2] | pad[1:-1, 2:]
    edge = occ & touches_free
    h = occ.shape[0]
    rows, cols = np.nonzero(edge)
    x0, _, y0, _ = m['extent']
    return np.stack([x0 + (cols + 0.5) * m['res'], y0 + (h - 1 - rows + 0.5) * m['res']], axis=1)


def nearest_dist(a, b, chunk=2000):
    out = np.empty(len(a))
    for s in range(0, len(a), chunk):
        d = a[s:s + chunk, None, :] - b[None, :, :]
        out[s:s + chunk] = np.sqrt((d ** 2).sum(-1)).min(axis=1)
    return out


def evaluate(gt, slam, tol):
    gt_pts, slam_pts = boundary_points(gt), boundary_points(slam)
    d_slam = nearest_dist(slam_pts, gt_pts)
    d_gt = nearest_dist(gt_pts, slam_pts)
    precision = float((d_slam <= tol).mean())
    recall = float((d_gt <= tol).mean())
    return {
        'tolerance_m': tol,
        'slam_wall_cells': int(len(slam_pts)), 'gt_wall_cells': int(len(gt_pts)),
        'mean_error_m': round(float(d_slam.mean()), 4),
        'rms_error_m': round(float(np.sqrt((d_slam ** 2).mean())), 4),
        'p95_error_m': round(float(np.percentile(d_slam, 95)), 4),
        'max_error_m': round(float(d_slam.max()), 4),
        'precision': round(precision, 4), 'recall': round(recall, 4),
        'f1': round(2 * precision * recall / (precision + recall), 4)
        if precision + recall > 0 else 0.0,
    }, d_slam, slam_pts


def plot(gt, slam_pts, d_slam, tol, path):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    fig, axes = plt.subplots(1, 2, figsize=(14, 5.6))
    axes[0].imshow(gt['img'], cmap='gray', extent=gt['extent'], vmin=0, vmax=255)
    axes[0].set_title('Ground truth (generated from the world)')
    axes[1].imshow(gt['img'], cmap='gray', extent=gt['extent'], vmin=0, vmax=255, alpha=0.35)
    sc = axes[1].scatter(slam_pts[:, 0], slam_pts[:, 1], c=np.minimum(d_slam, 3 * tol),
                         s=1.5, cmap='turbo', vmin=0, vmax=3 * tol)
    axes[1].set_title('SLAM Toolbox map, coloured by error to ground truth')
    fig.colorbar(sc, ax=axes[1], label='distance to nearest true wall [m]')
    for ax in axes:
        ax.set_xlabel('x [m]')
        ax.set_ylabel('y [m]')
        ax.set_aspect('equal')
    fig.tight_layout()
    fig.savefig(path, dpi=120)
    plt.close(fig)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--slam', required=True, help='SLAM map .yaml')
    ap.add_argument('--gt', default=None, help='ground-truth map .yaml (default: amr_bringup)')
    ap.add_argument('--tol', type=float, default=0.10, help='match tolerance [m]')
    ap.add_argument('--out', default='map_eval', help='output directory')
    args, _ = ap.parse_known_args()

    gt_path = args.gt
    if gt_path is None:
        from ament_index_python.packages import get_package_share_directory
        gt_path = os.path.join(get_package_share_directory('amr_bringup'), 'maps', 'warehouse.yaml')

    gt, slam = load_map(gt_path), load_map(os.path.expanduser(args.slam))
    metrics, d_slam, slam_pts = evaluate(gt, slam, args.tol)
    os.makedirs(args.out, exist_ok=True)
    with open(os.path.join(args.out, 'map_metrics.json'), 'w') as f:
        json.dump(metrics, f, indent=2)
    md = ('| metric | value |\n|---|---|\n' +
          '\n'.join(f'| {k} | {v} |' for k, v in metrics.items()) + '\n')
    with open(os.path.join(args.out, 'map_metrics.md'), 'w') as f:
        f.write(md)
    try:
        plot(gt, slam_pts, d_slam, args.tol, os.path.join(args.out, 'map_comparison.png'))
    except ImportError:
        pass
    print(md)
    print(f'written to {os.path.abspath(args.out)}')


if __name__ == '__main__':
    main()
