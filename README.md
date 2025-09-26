# clover_payload

ROS package for simulated payload delivery with the **Clover drone** in **Gazebo**.

## Features

- Spawns a custom payload model (e.g., phone, box) under the drone at startup.
- Attaches the payload physically using `gazebo_ros_link_attacher`.
- Provides ROS services:
  - `/release_load` — detach and drop the payload.
  - `/reset_delivery` — reposition and reattach the payload under the drone (even mid-flight).
- Uses Gazebo model state (not TF) for fast and reliable positioning.
- Fully compatible with `clover_simulation`.

## Usage

```bash
rosrun clover_payload payload_manager.py _offset:="[0.0, 0.18, -0.1]"
rosservice call /release_load
rosservice call /reset_delivery
