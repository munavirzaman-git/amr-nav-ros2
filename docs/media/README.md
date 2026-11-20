# Media

Images referenced by the main README. `warehouse_layout.png` is generated; the rest you capture.

| File | How to capture |
|---|---|
| `demo.gif` | Record RViz while the robot avoids a moving obstacle (e.g. `peek`, or OBS → `ffmpeg -i demo.mp4 -vf "fps=12,scale=900:-1" -loop 0 demo.gif`). Aim for under 8 MB. |
| `rviz_navigation.png` | RViz with both costmaps, the A* path and the DWA rollouts visible. |
| `gazebo_world.png` | Gazebo view of the warehouse with the robot and the obstacles. |

When a file is added, uncomment its line in the README's *Demo* section.
