#!/bin/bash
set -e

file_dir=$(cd "$(dirname "$0")" && pwd)
repo_dir=$(cd "${file_dir}/.." && pwd)
user_name=$(id -un)
uid=$(id -u)
gid=$(id -g)

# X forwarding
xhost +SI:localuser:${user_name} 2>/dev/null || true

GPU_OPT=""
if command -v nvidia-smi >/dev/null 2>&1; then
  GPU_OPT="--gpus all"
fi

# Collect USB/serial device options
DEVICE_OPTS=""
for dev in /dev/ttyUSB* /dev/ttyACM* /dev/video*; do
  [ -e "$dev" ] && DEVICE_OPTS="${DEVICE_OPTS} --device ${dev}"
done

docker run -it --rm --privileged \
  --name hils-bridge \
  --net=host \
  --ipc=host \
  ${GPU_OPT} \
  ${DEVICE_OPTS} \
  --device /dev/dri \
  -e DISPLAY=$DISPLAY \
  -e XAUTHORITY=/home/ubuntu/.Xauthority \
  -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
  -v $HOME/.Xauthority:/home/ubuntu/.Xauthority:ro \
  -v ${repo_dir}/ros2_hils_bridge:/home/ubuntu/colcon_ws/src/ros2_hils_bridge \
  -v /run/dbus/system_bus_socket:/run/dbus/system_bus_socket \
  --user ${uid}:${gid} \
  ${user_name}/ros-jazzy-hils
