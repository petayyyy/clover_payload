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

## Install
.bashrc
```bash
export GAZEBO_PLUGIN_PATH=$GAZEBO_PLUGIN_PATH~/catkin_ws/devel/lib
```

## Usage
Команда сброса груза.
```bash
rosservice call /release_load
```
Команда повторной установки груза.
```bash
rosservice call /reset_delivery
```
## Add to Clover simulator
Откройте simulator.launch и модифицируйте его добавив в конце.
```bash
<include file="$(find clover_payload)/launch/payload_phone.launch">
</include>
```
Откройте ваш файл мира и добавте до закрывающего блока `</world>` строку.
```bash
<plugin name="link_attacher" filename="libgazebo_ros_link_attacher.so"/>
``` 
