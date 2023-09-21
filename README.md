# rosbot-telepresence

Manual ROSbot Driving over the Internet with Real-Time Camera Feed

![ROSbot ROS2 user interface](docs/teleop-rosbot.png)

## Clonning the repo

This repository contains the Docker Compose setup for both PC and ROSbot. You can clone it to both PC and ROSbot, or use the `./sync_with_rosbot.sh` script to clone it to your PC and keep it synchronized with the robot

```bash
git clone https://github.com/husarion/rosbot-telepresence
cd rosbot-telepresence 
export ROSBOT_ADDR=10.5.10.123 # Replace with your own ROSbot's IP or Husarnet hostname
./sync_with_rosbot.sh $ROSBOT_ADDR
```

## Flashing the ROSbot Firmware

SSH to the ROSbot's shell:

```bash
ssh husarion@$ROSBOT_ADDR
```

and execute:

```bash
./flash_rosbot_firmware.sh
```

## Configuration

### Connecting ROSbot and laptop over VPN

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


### Choosing the video compression

Edit `.env` file and uncomment one of the available configs:

```bash
# using "compressed" codec from image_transport_plugins
# CODEC=compressed

# using "theora" codec from image_transport_plugins
CODEC=theora
```

## Running

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

## Usefull tips

**1. Checking a datarate**

If you want to check the datarate generated by a video stream:

```bash
husarion@rosbot:~$ ifstat -i wlan0
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