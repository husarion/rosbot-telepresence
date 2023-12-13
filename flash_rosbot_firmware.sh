#!/bin/bash

echo "stopping all running containers"
docker stop rosbot

echo "flashing the firmware for STM32 microcontroller in ROSbot"
docker run \
--rm -it --privileged \
$(yq .services.rosbot.image compose.yaml) \
ros2 run rosbot_utils flash_firmware