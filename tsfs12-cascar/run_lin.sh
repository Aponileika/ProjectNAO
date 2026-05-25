#!/bin/bash

xhost +local:root

docker run -it --rm \
  --net=host \
  -e DISPLAY=$DISPLAY \
  -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
  -e XDG_RUNTIME_DIR=/run/user/$(id -u) \
  -e LIBGL_ALWAYS_SOFTWARE=1 \
  -v /run/user/$(id -u):/run/user/$(id -u) \
  -e PULSE_SERVER=unix:/run/user/$(id -u)/pulse/native \
  -v /run/user/$(id -u)/pulse:/run/user/$(id -u)/pulse \
  -v $(pwd):/cascar_ws \
  --device=/dev/i2c-1 \
  --device=/dev/arduino \
  --device=/dev/rplidar \
  --device /dev/snd \
  --hostname cascar \
  --name cascar cascar:latest
