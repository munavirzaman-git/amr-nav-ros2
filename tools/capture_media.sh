#!/usr/bin/env bash
# Capture REAL simulation media into docs/media/ and add a Demo section to the README.
#   tools/capture_media.sh            # ~90 s of driving
#   DURATION=150 tools/capture_media.sh
# Needs an X11 session (not Wayland) and: sudo apt install ffmpeg xdotool imagemagick
set -u

REPO="$(cd "$(dirname "$0")/.." && pwd)"
MEDIA="$REPO/docs/media"
DURATION=${DURATION:-90}
STARTUP=${STARTUP:-35}
mkdir -p "$MEDIA"

for c in ffmpeg xdotool import; do
  command -v "$c" >/dev/null || { echo "Missing '$c'. Run: sudo apt install ffmpeg xdotool imagemagick"; exit 1; }
done
if [ "${XDG_SESSION_TYPE:-x11}" = "wayland" ]; then
  echo "You are in a Wayland session; window capture only gives black frames there."
  echo "Log out, click the gear icon on the login screen, choose 'Ubuntu on Xorg', log in and rerun."
  exit 1
fi
[ -f "$REPO/install/setup.bash" ] || { echo "Build the workspace first (colcon build)."; exit 1; }

set +u
source /opt/ros/jazzy/setup.bash
source "$REPO/install/setup.bash"
set -u
export GZ_IP=127.0.0.1 QT_ACCESSIBILITY=0

LAUNCH_PID=""
FF_PID=""
cleanup() {
  [ -n "$FF_PID" ] && kill -INT "$FF_PID" 2>/dev/null && wait "$FF_PID" 2>/dev/null
  [ -n "$LAUNCH_PID" ] && kill -INT "$LAUNCH_PID" 2>/dev/null
  sleep 4
  pkill -f "gz sim" 2>/dev/null; pkill -f rviz2 2>/dev/null
}
trap cleanup EXIT

find_window() {
  local id=""
  for _ in $(seq 1 30); do
    id=$(xdotool search --onlyvisible --name "$1" 2>/dev/null | head -1)
    [ -n "$id" ] && { echo "$id"; return 0; }
    sleep 1
  done
  return 1
}

echo "[1/6] Launching simulation + Nav2 (A* + DWA, moving obstacles). Log: /tmp/capture_launch.log"
ros2 launch amr_bringup bringup.launch.py > /tmp/capture_launch.log 2>&1 &
LAUNCH_PID=$!
echo "      waiting ${STARTUP}s for everything to come up..."
sleep "$STARTUP"

GZ_WIN=$(find_window "Gazebo Sim") || { echo "Gazebo window not found"; exit 1; }
RVIZ_WIN=$(find_window "RViz") || { echo "RViz window not found"; exit 1; }

echo "[2/6] Gazebo screenshot"
xdotool windowactivate --sync "$GZ_WIN"; sleep 2
import -window "$GZ_WIN" "$MEDIA/gazebo_world.png"

echo "[3/6] RViz screenshot"
xdotool windowsize --sync "$RVIZ_WIN" 1400 900 2>/dev/null
xdotool windowactivate --sync "$RVIZ_WIN"; sleep 2
import -window "$RVIZ_WIN" "$MEDIA/rviz_overview.png"

eval "$(xdotool getwindowgeometry --shell "$RVIZ_WIN")"
W=$((WIDTH / 2 * 2)); H=$((HEIGHT / 2 * 2))

echo "[4/6] Recording ${DURATION}s of navigation (keep the RViz window visible, don't cover it)"
ffmpeg -y -loglevel error -f x11grab -framerate 15 -video_size "${W}x${H}" \
  -i "${DISPLAY}+${X},${Y}" -c:v libx264 -preset veryfast -crf 23 -pix_fmt yuv420p \
  "$MEDIA/demo.mp4" &
FF_PID=$!
( sleep $((DURATION / 3)); import -window "$RVIZ_WIN" "$MEDIA/rviz_navigation.png" ) &
SHOT_PID=$!

timeout "$DURATION" ros2 run amr_tools nav_benchmark --label demo --repeats 1 \
  --output /tmp/capture_demo_run.csv > /tmp/capture_nav.log 2>&1

kill -INT "$FF_PID" 2>/dev/null; wait "$FF_PID" 2>/dev/null; FF_PID=""
wait "$SHOT_PID" 2>/dev/null

echo "[5/6] Making demo.gif"
make_gif() {
  ffmpeg -y -loglevel error -i "$MEDIA/demo.mp4" -vf \
    "setpts=0.5*PTS,fps=10,scale=$1:-1:flags=lanczos,split[a][b];[a]palettegen=max_colors=128[p];[b][p]paletteuse=dither=bayer:bayer_scale=4" \
    -loop 0 "$MEDIA/demo.gif"
}
for width in 900 750 600 480; do
  make_gif "$width"
  size=$(stat -c %s "$MEDIA/demo.gif")
  [ "$size" -lt 9500000 ] && break
  echo "      ${width}px GIF is $((size / 1000000)) MB, shrinking..."
done

echo "[6/6] Updating README"
python3 - "$REPO/README.md" <<'PY'
import sys
path = sys.argv[1]
s = open(path).read()
old = "<!-- ## Demo\n![demo](docs/media/demo.gif)\n-->"
new = """## Demo

![demo](docs/media/demo.gif)

*Custom A\\* global planner + DWA local planner with AMCL, driving the benchmark tour while two
moving obstacles cross its path (2x speed). Full-quality video: [docs/media/demo.mp4](docs/media/demo.mp4).*

| Gazebo Harmonic | RViz: start | RViz: navigating |
|---|---|---|
| ![gazebo](docs/media/gazebo_world.png) | ![rviz start](docs/media/rviz_overview.png) | ![rviz navigation](docs/media/rviz_navigation.png) |"""
if old in s:
    open(path, 'w').write(s.replace(old, new))
    print("      Demo section added to README.md")
elif "## Demo" in s:
    print("      README already has a Demo section (media files refreshed)")
else:
    print("      Demo placeholder not found; add the images to README.md manually")
PY

echo
echo "Done. Files in docs/media/:"
ls -lh "$MEDIA" | grep -E "png|mp4|gif"
echo
echo "Check them (xdg-open docs/media/demo.gif), then:"
echo "  git add docs/media README.md tools/capture_media.sh && git commit -m 'Add simulation screenshots and demo video' && git push"
