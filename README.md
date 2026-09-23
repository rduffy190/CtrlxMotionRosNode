# ctrlX MOTION ROS 2 Bridge

A ROS 2 (Jazzy) node for position control of Bosch Rexroth **ctrlX MOTION** axes. It connects to the ctrlX Data Layer and exposes each axis as a status topic plus a set of action servers, so any ROS 2 application can power, reset and move axes without speaking the Data Layer directly.

The node can run on a development machine against a ctrlX CORE / COREvirtual over TCP, or be packaged as a snap and installed as an app on the CORE itself.

> This is a supervisory layer, not a real-time controller. The ctrlX motion kernel handles trajectory generation and real-time behaviour; this node issues commands and reports state.

## Features

- **Automatic axis discovery**: browses `motion/axs` on startup, or uses a fixed list from the config file.
- **Per-axis action servers** for power on/off, fault reset and absolute position moves, with cancel support (a cancelled move sends an abort command to the axis).
- **Per-axis status topic** with position, velocity, acceleration, powered and faulted state.
- **Efficient status path**: a single Data Layer subscription covers every axis, regardless of how many there are; values are pushed by the motion app and republished at a steady ROS rate.
- **Typed commands**: move and abort commands are built with the ctrlX MOTION flatbuffer schemas, so payload errors show up at compile time.
- **Snap packaging** for deployment on ctrlX OS.

## Repository layout

| Path | Description |
|---|---|
| `motion_action_cpp/` | The bridge node (C++, `rclcpp` / `rclcpp_action`) |
| `ctrlx_motion_action/` | Action definitions: `PowerOn`, `ResetFault`, `MovePosition` |
| `axis_status_msg/` | Message definition: `AxisStatus` |
| `snap/snapcraft.yaml` | Snap recipe for ctrlX OS |
| `wrapper/run-ctrlx-motion.sh` | Snap entry point: sources the ROS runtime and starts the node |
| `build_snap.sh` | Builds the colcon workspace and packs the snap |

## ROS interfaces

For every axis `<axis>` the node (`ctrlx_motion_node`) creates:

| Kind | Name | Type |
|---|---|---|
| Topic (pub) | `<axis>_status` | `axis_status_msg/msg/AxisStatus` |
| Action | `<axis>_power` | `ctrlx_motion_action/action/PowerOn` |
| Action | `<axis>_reset` | `ctrlx_motion_action/action/ResetFault` |
| Action | `<axis>_move_position` | `ctrlx_motion_action/action/MovePosition` |

### `AxisStatus.msg`

```
bool    powered_on
bool    faulted
float64 act_acc
float64 act_vel
float64 act_pos
```

Position, velocity and acceleration come from the interpolator values (`motion/axs/<axis>/state/values/ipo`). `powered_on` and `faulted` are derived from the PLCopen state (`motion/axs/<axis>/state/opstate/plcopen`):

| PLCopen state | powered_on | faulted |
|---|---|---|
| `ERROR`, `ERROR_STOP` | false | true |
| `DISABLED`, `STOPPED` | false | false |
| `STANDSTILL`, `IN_POSITION`, `IN_MOTION`, `HOMING` | true | false |

Unrecognised states leave the previous flags unchanged. A topic publishes nothing until its axis has delivered at least one value, so you never see a fake "position 0, not faulted" reading at startup.

### Actions

| Action | Goal | Result | Feedback |
|---|---|---|---|
| `PowerOn` | `bool power_on` (true = on, false = off) | `bool powered_on` | `bool powering_on` |
| `ResetFault` | *(empty)* | `bool done` | `bool resetting` |
| `MovePosition` | `float64 pos, vel, acc, dcc` | `bool done` | `float64 dist_to_target` |

Behaviour worth knowing:

- **One command per axis at a time.** A goal sent while another action on the same axis is running is rejected. Different axes are fully independent.
- **Completion** is taken from the motion app's command state (`motion/axs/<axis>/state/cmd-state/<id>`), polled at 5 Hz. The goal succeeds when the command reports `DONE`, and aborts if the command can't be created or its state can't be read.
- **Moves are absolute, unbuffered and use the shortest way.** A move replaces any current motion rather than queuing behind it.
- **Cancelling a move** sends an abort command using the goal's `dcc` as the deceleration, then waits for the abort to finish.
- `dist_to_target` feedback is currently always `-1`, as the motion app doesn't provide it yet.

## Prerequisites

- Ubuntu 24.04 with **ROS 2 Jazzy**
- **ctrlX Data Layer** library, either the `ctrlx-datalayer` Debian package (preferred, found through pkg-config) or an unpacked ctrlX AUTOMATION SDK
- A ctrlX CORE or COREvirtual running the **ctrlX MOTION** app with at least one configured axis
- For snap builds: `snapcraft`, and the ROS 2 base runtime snap installed on the target CORE (the snap consumes it through a `ros-base` content plug)

## Building

```bash
source /opt/ros/jazzy/setup.bash
colcon build
```

If the Data Layer is not installed as a Debian package, point CMake at the SDK instead:

```bash
colcon build --cmake-args -DCTRLX_SDK_DIR=$HOME/ctrlx-automation-sdk
```

## Configuration

Parameters live in `motion_action_cpp/config/ctrlx_motion.yaml` and are installed alongside the package.

| Parameter | Default in YAML | Description |
|---|---|---|
| `axes_root` | `motion/axs` | Data Layer node browsed to discover axes |
| `axes` | `['X']` | Explicit axis list. Leave empty (`[]`) to auto-discover from `axes_root` |
| `status_ros_publish_interval_ms` | `100` | How often every status topic is published |
| `status_publish_interval_ms` | `100` | Data Layer subscription publish interval. Keep ≤ the ROS interval |
| `status_sampling_interval_us` | `50000` | How often the motion app samples each node (microseconds) |

**Connection.** When the node runs inside a snap it connects to the Data Layer over IPC automatically. Outside a snap it uses TCP with the defaults in `motion_action_cpp/include/motion_action_cpp/dl_helper.hpp` (`192.168.1.1`, user/password `boschrexroth`, port 443). For a COREvirtual with port forwarding or different credentials, edit the arguments to `getConnectionString()` in that header and rebuild.

## Running

### On a development machine

```bash
source install/setup.bash
ros2 run motion_action_cpp ctrlx_motion_node --ros-args \
  --params-file install/motion_action_cpp/share/motion_action_cpp/config/ctrlx_motion.yaml
```

### Example usage (axis `X`)

```bash
# Watch status
ros2 topic echo /X_status

# Reset any fault
ros2 action send_goal /X_reset ctrlx_motion_action/action/ResetFault "{}"

# Power on
ros2 action send_goal /X_power ctrlx_motion_action/action/PowerOn "{power_on: true}"

# Move to position 100 with feedback
ros2 action send_goal -f /X_move_position ctrlx_motion_action/action/MovePosition \
  "{pos: 100.0, vel: 50.0, acc: 200.0, dcc: 200.0}"

# Power off
ros2 action send_goal /X_power ctrlx_motion_action/action/PowerOn "{power_on: false}"
```

Units follow the axis configuration in ctrlX MOTION.

## Deploying to ctrlX OS as a snap

```bash
./build_snap.sh
```

The script cleans and rebuilds the colcon workspace against `/opt/ros/jazzy`, then runs `snapcraft pack --build-for=amd64`. Install the resulting `ctrlx-motion-ros-bridge_*.snap` on the CORE through the ctrlX OS web interface (App management, with installation from unknown sources allowed), alongside the ROS 2 base runtime snap.

Inside the snap the node runs as a daemon (restarting after 10 s if it exits), uses IPC to reach the Data Layer, and reads the bundled `ctrlx_motion.yaml`.

> The snap recipe currently targets **amd64** only (e.g. ctrlX CORE X7 or COREvirtual). An arm64 build would need `arm64` added under `platforms` and a matching `--build-for` in `build_snap.sh`.

## Architecture notes

- **Status**: one Data Layer client with one subscription for every axis's IPO values and PLCopen state. The subscription callback decodes changes into a write buffer; a ROS timer copies it to a read buffer and publishes every axis each tick, so idle axes keep a steady topic rate.
- **Actions**: each axis owns its own Data Layer client and three action servers in a shared reentrant callback group. Each accepted goal runs on its own worker thread, and the node spins a `MultiThreadedExecutor`, so a long move never blocks other axes or status publishing.
- **Vendored schemas**: `motion_action_cpp/include/motion_action_cpp/fbs/motion/core/` contains generated flatbuffer headers copied from the ctrlX AUTOMATION SDK (`AxsCmdPosData`, `DynamicLimits`, `AxsCmdAbortData`, `SysCmdReset`, `AxsIpoValues`). If the MOTION app is upgraded and a schema changes, re-copy the matching header from the corresponding SDK release.

## Acknowelegements 
- Claude Readme
- Claude Did some framework stuff

## License

Apache License 2.0. See the `LICENSE` files in the package directories.
