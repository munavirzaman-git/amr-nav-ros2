# Contributing

Issues and pull requests are welcome.

1. Fork and create a branch: `git checkout -b feature/my-change`.
2. Build and test:
   ```bash
   colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
   colcon test --packages-select amr_planners && colcon test-result --verbose
   flake8 src/
   ```
3. Keep algorithm logic in the ROS-free cores (`astar_core`, `dwa_core`) and cover it with gtests;
   keep the Nav2 plugin classes as thin wrappers.
4. If you change planner behaviour, re-run the benchmark and update `results/`
   (see [docs/benchmarking.md](docs/benchmarking.md)).
5. Open a pull request using the template.

C++ follows the ROS 2 style (2-space indent, 100-column lines); Python follows flake8 with a
100-column limit (`setup.cfg`).
