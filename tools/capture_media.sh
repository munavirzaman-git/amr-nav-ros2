#!/usr/bin/env bash
# Capture REAL simulation media into docs/media/ (v2).
#   - Gazebo (3D, overview camera) on the left, RViz (zoomed, following the robot) on the right
#   - recording starts only once the robot is actually driving
#   DURATION=150 tools/capture_media.sh
# Needs X11 and: sudo apt install ffmpeg xdotool imagemagick wmctrl x11-xserver-utils
set -u

REPO="$(cd "$(dirname "$0")/.." && pwd)"
MEDIA="$REPO/docs/media"
DURATION=${DURATION:-150}
STARTUP=${STARTUP:-35}
mkdir -p "$MEDIA"

for c in ffmpeg xdotool import wmctrl xrandr; do
  command -v "$c" >/dev/null || {
    echo "Missing '$c'. Run: sudo apt install ffmpeg xdotool imagemagick wmctrl x11-xserver-utils"; exit 1; }
done
if [ "${XDG_SESSION_TYPE:-x11}" = "wayland" ]; then
  echo "Wayland session: log out, gear icon -> 'Ubuntu on Xorg', log in and rerun."; exit 1
fi
[ -f "$REPO/install/setup.bash" ] || { echo "Build the workspace first (colcon build)."; exit 1; }

set +u
source /opt/ros/jazzy/setup.bash
source "$REPO/install/setup.bash"
set -u
export GZ_IP=127.0.0.1 QT_ACCESSIBILITY=0 RCUTILS_LOGGING_BUFFERED_STREAM=0

LAUNCH_PID=""; RVIZ_PID=""; FF_PID=""; NAV_PID=""
cleanup() {
  [ -n "$FF_PID" ] && kill -INT "$FF_PID" 2>/dev/null && wait "$FF_PID" 2>/dev/null
  [ -n "$NAV_PID" ] && kill -INT "$NAV_PID" 2>/dev/null
  [ -n "$RVIZ_PID" ] && kill -INT "$RVIZ_PID" 2>/dev/null
  [ -n "$LAUNCH_PID" ] && kill -INT "$LAUNCH_PID" 2>/dev/null
  sleep 4
  pkill -f nav_benchmark 2>/dev/null; pkill -f "gz sim" 2>/dev/null; pkill -f rviz2 2>/dev/null
}
trap cleanup EXIT

find_window() {
  local id=""
  for _ in $(seq 1 40); do
    id=$(xdotool search --onlyvisible --name "$1" 2>/dev/null | head -1)
    [ -n "$id" ] && { echo "$id"; return 0; }
    sleep 1
  done
  return 1
}

place_window() {  # id x y w h
  wmctrl -i -r "$1" -b remove,maximized_vert,maximized_horz 2>/dev/null
  sleep 0.5
  wmctrl -i -r "$1" -e "0,$2,$3,$4,$5" 2>/dev/null
  sleep 1
}

MON=$(xrandr --query | grep -oP ' connected primary \K[0-9]+x[0-9]+\+[0-9]+\+[0-9]+' | head -1)
[ -z "$MON" ] && MON=$(xrandr --query | grep -oP ' connected \K[0-9]+x[0-9]+\+[0-9]+\+[0-9]+' | head -1)
MW=${MON%%x*}; REST=${MON#*x}; MH=${REST%%+*}; REST=${REST#*+}; MX=${REST%%+*}; MY=${REST#*+}
TOP=40
HALF=$((MW / 2)); WH=$((MH - TOP - 20))
echo "Primary monitor: ${MW}x${MH} at ${MX},${MY}"

echo "[1/7] Launching simulation + Nav2 (A* + DWA, moving obstacles). Log: /tmp/capture_launch.log"
ros2 launch amr_bringup bringup.launch.py rviz:=false > /tmp/capture_launch.log 2>&1 &
LAUNCH_PID=$!

RVIZ_CFG=/tmp/amr_demo.rviz
sed -e 's|Target Frame: <Fixed Frame>|Target Frame: base_footprint|' \
    -e 's|^      Scale: 45$|      Scale: 110|' \
    "$(ros2 pkg prefix amr_bringup)/share/amr_bringup/rviz/navigation.rviz" > "$RVIZ_CFG"
rviz2 -d "$RVIZ_CFG" --ros-args -p use_sim_time:=true > /tmp/capture_rviz.log 2>&1 &
RVIZ_PID=$!

echo "      waiting ${STARTUP}s for everything to come up..."
sleep "$STARTUP"
GZ_WIN=$(find_window "Gazebo Sim") || { echo "Gazebo window not found"; exit 1; }
RVIZ_WIN=$(find_window "RViz") || { echo "RViz window not found"; exit 1; }

echo "[2/7] Arranging windows side by side"
place_window "$GZ_WIN" "$MX" $((MY + TOP)) "$HALF" "$WH"
place_window "$RVIZ_WIN" $((MX + HALF)) $((MY + TOP)) "$HALF" "$WH"

echo "[3/7] Gazebo camera: overview of the warehouse"
gz service -s /gui/move_to/pose --reqtype gz.msgs.GUICamera --reptype gz.msgs.Boolean \
  --timeout 3000 --req 'pose: {position: {x: 0, y: -16, z: 14},
  orientation: {w: 0.658, x: -0.259, y: 0.259, z: 0.658}}' >/dev/null 2>&1 \
  || echo "      (could not move the Gazebo camera; using its default view)"
sleep 3

echo "[4/7] Screenshots"
xdotool windowactivate --sync "$GZ_WIN" 2>/dev/null; sleep 1
import -window "$GZ_WIN" "$MEDIA/gazebo_world.png"
xdotool windowactivate --sync "$RVIZ_WIN" 2>/dev/null; sleep 1
import -window "$RVIZ_WIN" "$MEDIA/rviz_overview.png"

echo "[5/7] Starting the navigation tour; waiting until the robot gets its first goal..."
ros2 run amr_tools nav_benchmark --label demo --repeats 1 \
  --output /tmp/capture_demo_run.csv > /tmp/capture_nav.log 2>&1 &
NAV_PID=$!
STARTED=0
for _ in $(seq 1 150); do
  if grep -q -- "-> ne_bay" /tmp/capture_nav.log; then STARTED=1; break; fi
  if ! kill -0 "$NAV_PID" 2>/dev/null; then break; fi
  sleep 1
done
if [ "$STARTED" -ne 1 ]; then
  echo "      Nav2 did not start driving. Last lines of /tmp/capture_nav.log:"
  tail -25 /tmp/capture_nav.log; exit 1
fi

eval "$(xdotool getwindowgeometry --shell "$GZ_WIN")";  GX=$X; GY=$Y; GW=$WIDTH; GH=$HEIGHT
eval "$(xdotool getwindowgeometry --shell "$RVIZ_WIN")"; RX=$X; RY=$Y; RW=$WIDTH; RH=$HEIGHT
X0=$(( GX < RX ? GX : RX )); Y0=$(( GY < RY ? GY : RY ))
X1=$(( GX + GW > RX + RW ? GX + GW : RX + RW )); Y1=$(( GY + GH > RY + RH ? GY + GH : RY + RH ))
W=$(( (X1 - X0) / 2 * 2 )); H=$(( (Y1 - Y0) / 2 * 2 ))

echo "[6/7] Robot is driving - recording ${DURATION}s of ${W}x${H} (don't touch the screen)"
ffmpeg -y -loglevel error -f x11grab -framerate 15 -video_size "${W}x${H}" \
  -i "${DISPLAY}+${X0},${Y0}" -c:v libx264 -preset veryfast -crf 24 -pix_fmt yuv420p \
  "$MEDIA/demo.mp4" &
FF_PID=$!
( sleep $((DURATION / 3)); import -window "$RVIZ_WIN" "$MEDIA/rviz_navigation.png";
  import -window "$GZ_WIN" "$MEDIA/gazebo_navigation.png" ) &
SHOT_PID=$!
sleep "$DURATION"
kill -INT "$FF_PID" 2>/dev/null; wait "$FF_PID" 2>/dev/null; FF_PID=""
wait "$SHOT_PID" 2>/dev/null

echo "[7/7] Making demo.gif"
make_gif() {
  ffmpeg -y -loglevel error -i "$MEDIA/demo.mp4" -vf \
    "setpts=0.33*PTS,fps=10,scale=$1:-1:flags=lanczos,split[a][b];[a]palettegen=max_colors=128[p];[b][p]paletteuse=dither=bayer:bayer_scale=4" \
    -loop 0 "$MEDIA/demo.gif"
}
for width in 1100 900 750 600; do
  make_gif "$width"
  size=$(stat -c %s "$MEDIA/demo.gif")
  [ "$size" -lt 9500000 ] && break
  echo "      ${width}px GIF is $((size / 1000000)) MB, shrinking..."
done

python3 - "$REPO/README.md" <<'PY'
import re, sys
path = sys.argv[1]
s = open(path).read()
section = """## Demo

![demo](docs/media/demo.gif)

*Left: Gazebo Harmonic. Right: RViz following the robot. Custom A\\* global planner (blue path)
and DWA local planner (grey rollouts, green = chosen) with AMCL, while two moving obstacles cross
its route (3x speed). Full-quality video: [docs/media/demo.mp4](docs/media/demo.mp4).*

| Gazebo: warehouse | Gazebo: navigating | RViz: start | RViz: navigating |
|---|---|---|---|
| ![gazebo](docs/media/gazebo_world.png) | ![gazebo nav](docs/media/gazebo_navigation.png) | ![rviz start](docs/media/rviz_overview.png) | ![rviz nav](docs/media/rviz_navigation.png) |

![warehouse layout](docs/media/warehouse_layout.png)
"""
placeholder = "<!-- ## Demo\n![demo](docs/media/demo.gif)\n-->"
if placeholder in s:
    s = s.replace(placeholder, section.rstrip())
elif "## Demo" in s:
    s = re.sub(r"## Demo\n.*?(?=\n## )", section, s, count=1, flags=re.S)
open(path, "w").write(s)
print("      README Demo section updated")
PY

echo
echo "Done. Files in docs/media/:"
ls -lh "$MEDIA" | grep -E "png|mp4|gif"
echo
echo "Check: xdg-open docs/media/demo.gif   then:"
echo "  git add docs/media README.md tools/capture_media.sh && git commit -m 'Demo: Gazebo + RViz while navigating' && git push"
