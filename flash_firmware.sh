#!/bin/bash

echo "Stopping rosbot container"
docker stop rosbot

echo "Flashing the firmware for STM32 microcontroller in ROSbot"
docker run --rm -it --privileged \
$(yq .services.rosbot.image compose.yaml) \
ros2 run rosbot_utils flash_firmware