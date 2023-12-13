# rosbot-telepresence

Manual ROSbot Driving over the Internet with Real-Time Camera Feed

![ROSbot ROS2 user interface](docs/teleop-rosbot.png)

## Step 1: Connecting ROSbot and laptop over VPN

Ensure that both ROSbot 2R and your laptop linked to the same Husarnet VPN network. If they are not follow these steps:

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

## Step 2: Clonning the repo

This repository contains the Docker Compose setup for both PC and ROSbot. You can clone it to both PC and ROSbot, or use the `./sync_with_rosbot.sh` script to clone it to your PC and keep it synchronized with the robot

```bash
git clone https://github.com/husarion/rosbot-telepresence
cd rosbot-telepresence 
export ROSBOT_HOSTNAME=rosbot2r # Replace with your own Husarnet hostname
./sync_with_rosbot.sh $ROSBOT_HOSTNAME
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

## Step 4: Set the `.env`

Edit `.env` file and write down the ROSbot 2R Husarnet hostname here to let the PC part know how to find the ROSbot 2R.

```bash
ROSBOT_HOSTNAME=rosbot2r
```

## Step 5: Launching

### PC

```bash
xhost +local:docker && \
docker compose -f compose.pc.yaml up -d
```

To control the robot, open a teleop interface by typing the following command in a new terminal:

```bash
docker compose -f compose.pc.yaml run rviz ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

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

Rather than employing the `teleop_twist_keyboard` ROS 2 package, you have the option to use the Logitech F710 gamepad. To utilize it, plug it into your PC's USB port and add these lines to the `compose.pc.yaml`:

```yaml
joy2twist:
   image: husarion/joy2twist:iron
   devices:
      - /dev/input
   volumes:
      - ./params/joy2twist.yaml:/params.yaml
   environment:
   - ROS_DISCOVERY_SERVER=${ROSBOT_HOSTNAME}:8080
   command: >
      ros2 launch joy2twist gamepad_controller.launch.py
         joy2twist_params_file:=/params.yaml
```
