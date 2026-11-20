# Custom planners

Both planners are split into a ROS-free core (`astar_core`, `dwa_core`, depending only on
`nav2_costmap_2d::Costmap2D`) and a thin Nav2 plugin wrapper. The cores are covered by 12 gtests.

## A* global planner — `amr_planners::AStarPlanner`

**Graph.** 8-connected grid on the global costmap. A move is allowed if the target cell's cost
is below 253 (the robot centre is outside the inscribed radius). Unknown cells (255) are allowed
when `allow_unknown` is true, which is needed while mapping. Diagonal moves are rejected if
either orthogonal neighbour is blocked (no corner cutting).

**Edge cost.** For a step of length $\ell \in \{1, \sqrt 2\}$ cells into a cell with cost $c$:

$$ g(n') = g(n) + \ell \left(1 + w_c \frac{c}{252}\right) $$

`cost_weight` ($w_c$) trades path length for clearance: $w_c = 0$ gives the geometric shortest
path, larger values keep the robot further from obstacles.

**Heuristic.** Octile distance $h = (d_x + d_y) + (\sqrt 2 - 2)\min(d_x, d_y)$, scaled by
`heuristic_weight`. With weight 1 it is admissible (every step costs at least $\ell$), so A*
returns the optimal path under the cost above. Weights above 1 give weighted A*, which is faster
but no longer optimal.

**Robustness.**
- If the goal is inside an obstacle, the planner uses the nearest free cell within `tolerance`.
- A start inside the inflation zone is allowed, but a start inside a lethal cell is not.
- The search checks Nav2's cancel callback and the `max_planning_time` budget every 1024 expansions.
- Failures raise the matching `nav2_core` exceptions, so the behaviour tree's recoveries react correctly.

**Smoothing.** Gradient descent on the waypoints, with endpoints fixed:

$$ p_i \leftarrow p_i + w_d (q_i - p_i) + w_s (p_{i-1} + p_{i+1} - 2p_i) $$

Here $q_i$ are the original A* waypoints, $w_d$ is the data weight and $w_s$ the smoothing
weight. A move is rejected if it would put the point in a cell above
`max(original cost, cost_tolerance)` or in a lethal cell. Smoothing therefore never pulls the
path into an obstacle.

## DWA local planner — `amr_planners::DWAController`

Dynamic Window Approach (Fox, Burgard & Thrun, *IEEE RAM* 1997) for a unicycle robot.

1. **Dynamic window.** Starting from the measured velocity $(v_0, \omega_0)$, sample the
   velocities reachable within `acc_dt` that also respect the speed limits:
   $$ V_d = [v_0 - a_v \Delta t,\ v_0 + a_v \Delta t] \cap [v_{min}, v_{max}],\qquad
      \Omega_d = [\omega_0 - a_\omega \Delta t,\ \omega_0 + a_\omega \Delta t] \cap [-\omega_{max}, \omega_{max}] $$
   This gives `vx_samples` × `vtheta_samples` candidate pairs.
2. **Rollout.** Integrate each pair exactly along a circular arc over `sim_time`:
   $$ x_{k+1} = x_k + \tfrac{v}{\omega}\left(\sin(\theta_k + \omega\Delta t) - \sin\theta_k\right),\quad
      y_{k+1} = y_k - \tfrac{v}{\omega}\left(\cos(\theta_k + \omega\Delta t) - \cos\theta_k\right) $$
   For $\omega \approx 0$ the motion is a straight line.
3. **Admissibility.** Discard any rollout that passes through a cell of cost 253 or more on the
   local costmap.
4. **Objective.** Each criterion is normalised by its maximum over the admissible set, then the
   weighted sum is minimised:
   $$ J = w_p \hat d_{path} + w_g \hat d_{target} + w_h \hat{|\Delta\theta|} + w_o \hat c_{max} + w_v \frac{v_{max}-v}{v_{max}} $$
   - $d_{path}$: distance from the rollout's end to the reference path
   - $d_{target}$: distance from the rollout's end to the lookahead point on the plan
   - $\Delta\theta$: heading error from the rollout's end towards the lookahead point
   - $c_{max}$: highest costmap cost along the rollout
5. **Supervisor.** Around the DWA search:
   - Rotate in place when the heading error exceeds `rotate_in_place_angle`.
   - Scale the speed limit down linearly inside `approach_dist` of the goal.
   - Align to the goal's yaw within `final_rotate_dist`.
   - If every rollout collides, raise `NoValidControl` so Nav2 runs its recoveries.

The plugin transforms the global plan into the local costmap frame (`odom`), prunes the part
already passed, and keeps only the portion inside the local costmap. It follows Nav2 speed
limits and publishes `/local_plan` and `/dwa_candidates` for RViz.

## Parameters

See `src/amr_bringup/config/nav2_params.yaml`, blocks `GridBased_astar` and `FollowPath_dwa`.
The most useful knobs:

| Parameter | Effect |
|---|---|
| `cost_weight` (A*) | clearance vs path length |
| `heuristic_weight` (A*) | speed vs optimality |
| `w_obstacle` (DWA) | clearance during tracking |
| `w_path` / `w_goal` (DWA) | tight path tracking vs cutting corners towards the lookahead |
| `sim_time` (DWA) | look-ahead horizon; lower it for tight doorways |
| `acc_dt` (DWA) | dynamic-window width; about 2–4 controller periods |
