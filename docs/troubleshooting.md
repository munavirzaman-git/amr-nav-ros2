# Troubleshooting

| Symptom | Cause / fix |
|---|---|
| `colcon build` fails with Cython / numpy / IPython "duplicate package" errors | You ran it in a folder containing Python virtualenvs. Build from the workspace root, or add an empty `COLCON_IGNORE` file to each venv. |
| `create`: "Requesting list of world names" then timeout; GUI: "Waited for 10s for a subscriber" | gz-transport discovery is picking the wrong network interface (Docker, VPN, several NICs). `export GZ_IP=127.0.0.1`. `sim.launch.py` sets this by default. |
| Many `QML ... Binding loop` / `TextCaretMoved` warnings | Harmless Qt GUI noise. Hide it with `export QT_ACCESSIBILITY=0`. |
| `diff_drive_controller: Velocity command timed out. Braking.` | Normal while the robot is idle: no command arrived within 0.5 s. |
| `ros2 run amr_tools <x>`: "No executable found" | The source was updated but not rebuilt: `rm -rf build/amr_tools install/amr_tools && colcon build`. |
| No `/scan` | The world must load `gz-sim-sensors-system` with ogre2. Check `gz topic -l`. |
| Robot doesn't move | `ros2 control list_controllers` should show both controllers active. `/diff_drive_controller/cmd_vel` must receive TwistStamped. |
| `map` frame missing | Localization starts 5 s after Gazebo. On slow machines, increase the `TimerAction` periods in `bringup.launch.py`. |
| AMCL particles scattered | Use *2D Pose Estimate*. The default initial pose assumes the spawn pose is (0, 0, 0). |
| Headless run has no LiDAR | Needs `--headless-rendering` (passed by `headless:=true`) and working EGL/GPU drivers. |
| Planner plugin not found | Re-source `install/setup.bash`. Check that `amr_planners` built in Release mode. |
