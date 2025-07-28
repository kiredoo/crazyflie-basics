# Cyclic Pursuit ROS2 Workspace

This repository contains a ROS&nbsp;2 workspace that demonstrates a cyclic pursuit controller for multiple Bitcraze Crazyflie micro‑drones using the [Crazyswarm2](https://github.com/IMRCLab/crazyswarm2) stack.

## Repository Layout

- `src/crazyswarm2` – upstream Crazyswarm2 packages providing Crazyflie drivers, examples and simulation tools.
- `src/cyclic_pursuit` – custom package implementing the cyclic pursuit controller and helper utilities.
- `rosbag/` – example rosbag recordings.

## Building

The workspace uses `colcon` like a normal ROS&nbsp;2 workspace. From the repository root run:

```bash
colcon build --symlink-install
```

After the build completes, source the setup script:

```bash
source install/setup.bash
```

## Running the Controller

The cyclic pursuit controller is launched with:

```bash
ros2 launch cyclic_pursuit launch.py
```

Parameters such as drone IDs, desired velocities and controller gains are defined in `src/cyclic_pursuit/config/cyclic.yaml`.

A landing service for all active drones can be triggered via:

```bash
ros2 run cyclic_pursuit landing_client
```

## Utilities

`variable_pose_publisher.py` publishes synthetic pose messages to test the controller without real hardware.

## License

This project is provided under the terms specified in each package's `package.xml` file.
