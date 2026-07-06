# golfcart_low_level

ROS 2 package for the golf cart low-level control layer.

It runs two nodes on the Jetson:

- `ros_can_bridge`: supervisor and ROS 2 <-> STM32 CAN bridge.
- `steering_node`: KEYA steering motor driver over extended 29-bit CAN.

Recommended folder layout:

```text
GolfCart/
  golfcart-low-level-control/
    ros2_ws/
      src/
        golfcart_low_level/
```

## CAN Bus

Use InnoMaker USB2CAN as SocketCAN `can0` at 250 kbit/s:

```bash
sudo ip link set can0 up type can bitrate 250000 restart-ms 100
```

The included helper script does the same setup:

```bash
bash ~/GolfCart/golfcart-low-level-control/ros2_ws/src/golfcart_low_level/scripts/setup_can0.sh can0 250000
```

All CAN nodes on the bus must use the same bitrate. The KEYA steering motor uses
250 kbit/s, so the STM32 FDCAN firmware must also be configured for 250 kbit/s.

## Build On Jetson

Install dependencies:

```bash
sudo apt update
sudo apt install -y python3-can can-utils python3-colcon-common-extensions
```

Build:

```bash
source /opt/ros/$ROS_DISTRO/setup.bash
cd ~/GolfCart/golfcart-low-level-control/ros2_ws
colcon build --symlink-install
source install/setup.bash
```

## Run

Bring up CAN first:

```bash
bash src/golfcart_low_level/scripts/setup_can0.sh can0 250000
```

Launch both low-level nodes:

```bash
ros2 launch golfcart_low_level low_level.launch.py
```

Or run them separately:

```bash
ros2 run golfcart_low_level ros_can_bridge
ros2 run golfcart_low_level steering_node
```

## Main Topics

Inputs:

- `/cmd_vel` (`geometry_msgs/msg/Twist`)

Internal bridge-to-steering command:

- `/steering_cmd` (`std_msgs/msg/Float64`, radians)

Status outputs:

- `/e_stop_status`
- `/lowlevel_fault`
- `/speed_status`
- `/brake_status`
- `/brake_current`
- `/steering_angle`
- `/steering_fault`

## Quick Test

Publish a stopped command first:

```bash
ros2 topic pub -r 10 /cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.0}, angular: {z: 0.0}}"
```

Then a slow straight command:

```bash
ros2 topic pub -r 10 /cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.3}, angular: {z: 0.0}}"
```

Watch status:

```bash
ros2 topic echo /lowlevel_fault
ros2 topic echo /e_stop_status
ros2 topic echo /steering_angle
```
