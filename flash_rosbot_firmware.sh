#!/bin/bash

echo "stopping all running containers"
docker stop $(docker ps -q)

echo "flashing the firmware for STM32 microcontroller in ROSbot"
docker run \
--rm -it --privileged \
husarion/rosbot:humble-0.6.1-20230712 \
/flash-firmware.py /root/firmware.bin