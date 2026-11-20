# Benchmarking methodology

All runs use the same world, robot, costmaps, inflation (0.55 m), velocity and acceleration
limits (0.5 m/s, 1.8 rad/s) and goal tolerances (0.25 m, 0.25 rad). Only the planner or the
controller plugin changes.

## 1. Global planners — `planner_benchmark`
NavFn, Smac2D and A* are loaded side by side in `planner_server`. Each start/goal pair in
`benchmark_goals.yaml` (`planner_queries`) is sent to every planner several times, without moving
the robot, against the same global costmap.

| Metric | Meaning |
|---|---|
| success % | a non-empty path was returned |
| planner time [ms] | `planning_time` reported by planner_server, plus wall-clock time including action overhead |
| length [m] | path length |
| total turning [rad] | sum of absolute heading changes (lower = smoother) |
| max / mean cost | global costmap cost along the path (0–100), a proxy for clearance |

## 2. End-to-end navigation — `nav_benchmark`
The robot drives a 5-goal tour that crosses both dynamic-obstacle lanes. The stack is relaunched
for each planner/controller pair.

| Metric | Meaning |
|---|---|
| success % | NavigateToPose succeeded within the timeout |
| time [s], path [m], speed [m/s] | efficiency (simulation time, EKF odometry) |
| min obstacle distance [m] | minimum LiDAR range during the run (sensor centre to nearest return) |
| mean \|dω/dt\| | mean absolute change of the commanded yaw rate (lower = smoother) |
| recoveries | behaviour-tree recoveries triggered |
| final error [m] | distance from the goal at completion (map frame) |

Default comparison matrix (`run_comparison.sh`): `navfn+mppi`, `navfn+dwb`, `smac+mppi`,
`astar+dwb`, `astar+dwa`. Two of these pairs are controlled comparisons:
- `navfn+dwb` vs `astar+dwb`: only the global planner changes.
- `astar+dwb` vs `astar+dwa`: only the local planner changes, between two DWA implementations.

## 3. Mapping accuracy — `compare_maps`
The SLAM map is compared with the generated ground-truth map. Both are in the world frame,
because SLAM Toolbox anchors its map at the spawn pose (0, 0, 0). Only wall faces that border
free space are compared, so wall interiors and unobservable outer faces don't count against
SLAM.

| Metric | Meaning |
|---|---|
| mean / RMS / 95th-percentile error [m] | distance from each SLAM wall cell to the nearest true wall |
| precision | SLAM wall cells within the tolerance (default 0.10 m = 2 cells) of a true wall |
| recall | true wall faces that SLAM recovered |
| F1 | harmonic mean of precision and recall |

## Reproducing
```bash
ros2 run amr_bringup run_comparison.sh 3 ~/amr_benchmarks/run1
ros2 run amr_tools compare_maps --slam ~/maps/warehouse_slam.yaml --out results/map_eval
python3 tools/update_readme_results.py      # writes the tables into README.md
```

## Caveats
- The simulation is single-threaded and timing-sensitive. Report the mean over at least 3 repeats,
  and run with `headless:=true` to reduce CPU noise.
- The dynamic obstacles run on fixed patrol loops, not synchronised with the robot. Encounters
  therefore differ between runs; this tests robustness but adds variance.
- Planner time depends on the CPU, so compare planners on the same machine only.
