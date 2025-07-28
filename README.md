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

## Hardware Setup

1. Charge each Crazyflie and verify the battery voltage with `cfclient`.
   Only fly when the voltage is above **3.7&nbsp;V**.
2. Attach a reflective marker to the center of every drone and label them
   with unique IDs (for example `cf1`, `cf123`, ...).
3. Connect the laptop to the `saslab` Wi‑Fi network (password: `saslab`) and
   plug in the Crazyradio.
4. After charging, perform an individual test flight for each Crazyflie.

### Test Flight Procedure

1. Open a terminal in `crazyflie_basics` and run:

   ```bash
   source install/local_setup.bash
   ```

2. Determine the Crazyflie URI with `cfclient` and edit
   `crazyswarm2/crazyflie/config/crazyflies.yaml` so that only the drone being
   tested is enabled. Set its `uri` and `initial_position` fields.
3. Start the server:

   ```bash
   ros2 launch crazyflie launch.py
   ```

   Confirm that the log shows packets being received for the expected ID.
4. In another terminal run the example flight:

   ```bash
   ros2 run crazyflie_examples hello_world
   ```

   The drone should take off, hover and land when a key is pressed.
   Repeat for all Crazyflies.

## Running the Cyclic Pursuit

1. Place the labeled Crazyflies on the test ground facing the positive X
   direction. Use Motive to read their initial coordinates.
2. Enter the coordinates and full URIs in
   `crazyswarm2/crazyflie/config/crazyflies.yaml` under each ID.
3. Edit `cyclic_pursuit/config/cyclic.yaml` and list the Crazyflie names under
   `cfs_active_names` and their numeric IDs under `cfs_active_ids` along with
   the desired parameters.
4. Source the workspace again and launch the server:

   ```bash
   ros2 launch crazyflie launch.py
   ```

5. In another terminal start the controller:

   ```bash
   ros2 launch cyclic_pursuit launch.py
   ```

6. When the formation converges you can land all drones with:

   ```bash
   ros2 run cyclic_pursuit landing
   ```

### Motive Settings

* Enable **NatNet** with **Multicast** transmission.
* Enable **Unlabeled Markers** and **Rigid Bodies**.
* Use the **Z‑Axis** as the up axis.
* Default ports are 1510 (command) and 1511 (data).
* Ensure all assets are disabled and that Motive is in live mode.

## Recording and Analysis

To record data during the flight run in a separate terminal:

```bash
ros2 bag record /<topic1> /<topic2>
```

Press `Ctrl+C` to stop recording. The resulting rosbag can be processed with
the MATLAB script in `matlab_analysis` (example bags are provided in the
repository).

## Debugging

If communication issues occur, toggling the Wi‑Fi connection often resolves the
problem.

## Utilities

`variable_pose_publisher.py` publishes synthetic pose messages to test the controller without real hardware.

## License

This project is provided under the terms specified in each package's `package.xml` file.
