#!/usr/bin/env bash
# Full Nav2-vs-custom comparison. Each stack is launched headless, benchmarked, and torn down.
#   ros2 run amr_bringup run_comparison.sh [repeats] [output_dir]
set -u
REPEATS=${1:-3}
OUT=${2:-$HOME/amr_benchmarks/$(date +%Y%m%d_%H%M%S)}
mkdir -p "$OUT"
STACKS=("navfn mppi" "navfn dwb" "smac mppi" "astar dwb" "astar dwa")

run_stack () {
  local planner=$1 controller=$2 label="${1}_${2}"
  echo "=============== ${label} ==============="
  ros2 launch amr_bringup bringup.launch.py planner:="$planner" controller:="$controller" \
       headless:=true rviz:=false > "$OUT/${label}_launch.log" 2>&1 &
  local pid=$!
  sleep 25
  ros2 run amr_tools nav_benchmark --label "$label" --repeats "$REPEATS" \
       --output "$OUT/nav_results.csv"
  kill -INT $pid; wait $pid 2>/dev/null
  pkill -f "gz sim" 2>/dev/null; sleep 5
}

# 1) global planners only (all three are loaded side by side in one stack)
ros2 launch amr_bringup bringup.launch.py headless:=true rviz:=false dynamic_obstacles:=false \
     > "$OUT/planner_launch.log" 2>&1 &
PID=$!; sleep 25
ros2 run amr_tools planner_benchmark --repeats 10 --output "$OUT/planner_results.csv"
kill -INT $PID; wait $PID 2>/dev/null; pkill -f "gz sim" 2>/dev/null; sleep 5

# 2) end-to-end, with dynamic obstacles
for s in "${STACKS[@]}"; do run_stack $s; done

ros2 run amr_tools analyze_results "$OUT"
