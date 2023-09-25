# rosbot-telepresence

Manual ROSbot Driving over the Internet with Real-Time Camera Feed

![ROSbot ROS2 user interface](docs/teleop-rosbot.png)

## Step 1: Connecting ROSbot and laptop over VPN

Ensure that both ROSbot 2R and PRO are linked to the same Husarnet VPN network. If they are not follow these steps:

1. Setup a free account at [app.husarnet.com](https://app.husarnet.com/), create a new Husarnet network, click the **[Add element]** button and copy the code from the **Join Code** tab.
2. Connect your laptop to the [Husarnet network](https://husarnet.com/docs). If you are Ubuntu user, just run:

   ```bash
   curl https://install.husarnet.com/install.sh | sudo bash
   ```

   and connect to the Husarnet network with:

   ```bash
   sudo husarnet join <paste-join-code-here>
   ```

3. Connect your ROSbot to the Husarnet network. Husarnet is already pre-installed so just run:

   ```bash
   sudo husarnet join <paste-join-code-here> rosbot2r
   ```

   > note that `rosbot2r` is a Husarnet hostname that is hardcoded in the [compose.pc.yaml](/rosbot-telepresence/blob/main/compose.pc.yaml) file. If you want a different hostname for your ROSbot remember to change it.
 

## Step 2: Clonning the repo

This repository contains the Docker Compose setup for both PC and ROSbot. You can clone it to both PC and ROSbot, or use the `./sync_with_rosbot.sh` script to clone it to your PC and keep it synchronized with the robot

```bash
git clone https://github.com/husarion/rosbot-telepresence
cd rosbot-telepresence 
export ROSBOT_ADDR=rosbot2r # Replace with your own Husarnet hostname
./sync_with_rosbot.sh ROSBOT_ADDR$
```

## Step 3: Flashing the ROSbot Firmware

SSH to the ROSbot's shell:

```bash
ssh husarion@$ROSBOT_ADDR
```

and execute:

```bash
./flash_rosbot_firmware.sh
```

## Step 4: Choosing the video compression

Edit `.env` file and uncomment one of the available configs:

```bash
# using "compressed" codec from image_transport_plugins
# CODEC=compressed

# using "theora" codec from image_transport_plugins
CODEC=theora
```

## Step 5: Choosing the Network (DDS) Config

Edit `net.env` file and uncomment one of the available configs:

```bash
# =======================================
# Network config options (uncomment one)
# =======================================

# 1. Fast DDS + LAN
RMW_IMPLEMENTATION=rmw_fastrtps_cpp

# 2. Cyclone DDS + LAN
# RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

# 3. Fast DDS + VPN
# RMW_IMPLEMENTATION=rmw_fastrtps_cpp
# FASTRTPS_DEFAULT_PROFILES_FILE=/husarnet-fastdds.xml

# 4. Cyclone DDS + VPN
# RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
# CYCLONEDDS_URI=file:///husarnet-cyclonedds.xml
```

If you choose to use the VPN option, both your ROSbot and laptop must be connected to the same Husarnet network. Follow the guide [here](https://husarion.com/manuals/rosbot/remote-access/).

## Step 6: Launching

### PC

```bash
xhost +local:docker && \
docker compose -f compose.pc.yaml up -d
```

open a teleop interface - if you have ROS 2 installed on your laptop just run:

```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

> **Don't have ROS 2?**
>
> If you don't have ROS 2 natively installed, you can access the `interface` service from `compose.pc.yaml` that has the `teleop_twist_keyboard` package preeinstalled:
> 
> ```
> docker compose -f compose.pc.yaml exec -it interface bash
> ```
> 
> And inside the running container shell execute:
> 
> ```bash
> ros2 run teleop_twist_keyboard teleop_twist_keyboard
> ```

To turn off run:

```bash
docker compose -f compose.pc.yaml down
```

### ROSbot

```bash
docker compose up
```

## Troubleshooting

###  `Packet was not a Theora header` warning

The log from your computer where you launched `compose.pc.yaml` may contain the following message:

```
[image_republisher]: [theora] Packet was not a Theora header
```

Due to an issue in the theora codec, headers are probably sent only at the start. If you've initiated `compose.pc.yaml` following `compose.yaml`, it's essential to restart the image_compressor service.

To do so, execute in the ROSbot's terminal in the `/home/husarion/rosbot-telepresence` folder the following line:

```bash
docker compose restart image_compressor
```

## Usefull tips

**1. Checking a datarate**

To assess the data rate of a video stream being transmitted over the Husarnet VPN (which appears in your OS as the `hnet0` network interface), execute the following:

```bash
husarion@rosbot:~$ ifstat -i hnet0
      wlan0       
 KB/s in  KB/s out
    6.83   2744.66
    1.67   2659.88
    1.02   2748.40
    6.73   2565.20
    1.02   2748.65
    1.18   2749.64
```

**2. Sending uncompressed video frames over the network**

If raw image data is being transmitted over the network, you need to perform some [DDS-tunning](https://docs.ros.org/en/humble/How-To-Guides/DDS-tuning.html) (both on ROSbot and PC):

For configs in LAN:

```bash
sudo sysctl -w net.ipv4.ipfrag_time=3 # 3s
sudo sysctl -w net.ipv4.ipfrag_high_thresh=134217728 # (128 MB)
```

For configs over VPN:

```bash
sudo sysctl -w net.ipv6.ip6frag_time=3 # 3s
sudo sysctl -w net.ipv6.ip6frag_high_thresh=134217728 # (128 MB)
```

**3. Using Logitech F710 gamepad**

Rather than employing the `teleop_twist_keyboard` ROS 2 package, you have the option to use the Logitech F710 gamepad. To utilize it, plug it into your PC's USB port and remove the comment markers from these lines in `compose.pc.yaml`:

```yaml
  # joy2twist:
  #   image: husarion/joy2twist:humble
  #   <<: *net-config
  #   devices:
  #     - /dev/input
  #   volumes:
  #     - ./params/joy2twist.yaml:/params.yaml
  #   command: >
  #     ros2 launch joy2twist gamepad_controller.launch.py
  #       joy2twist_params_file:=/params.yaml
```
