set dotenv-load # to read ROBOT_NAMESPACE from .env file

[private]
default:
    @just --list --unsorted

[private]
alias husarnet := connect-husarnet
[private]
alias flash := flash-firmware
[private]
alias rosbot := start-rosbot
[private]
alias pc := start-pc
[private]
alias teleop := run-teleop
[private]
alias teleop-docker := run-teleop-docker
[private]
alias joy := run-joy

[private]
pre-commit:
    #!/bin/bash
    if ! command -v pre-commit &> /dev/null; then
        pip install pre-commit
        pre-commit install
    fi
    pre-commit run -a

# connect to Husarnet VPN network
connect-husarnet joincode hostname: _run-as-root
    #!/bin/bash
    if ! command -v husarnet > /dev/null; then
        echo "Husarnet is not installed. Installing now..."
        curl https://install.husarnet.com/install.sh | bash
    fi
    husarnet join {{joincode}} {{hostname}}

# flash the proper firmware for STM32 microcontroller in ROSbot 2R / 2 PRO
flash-firmware: _install-yq _run-as-user
    #!/bin/bash
    echo "Stopping all running containers"
    docker ps -q | xargs -r docker stop

    if grep -q "Raspberry Pi" /proc/cpuinfo; then
        echo "Setting up Raspberry Pi GPIO Port"
        gpio_chip=/dev/gpiochip0
        serial_port=/dev/ttyAMA0
    elif grep -q "Intel(R) Atom(TM) x5-Z8350" /proc/cpuinfo; then
        echo "Setting up UpBoard GPIO Port"
        gpio_chip=/dev/gpiochip4
        serial_port=/dev/ttyS4
    fi

    echo "Flashing the firmware for STM32 microcontroller in ROSbot"
    docker run --rm -it \
        --device $gpio_chip \
        --device $serial_port \
        $(yq .services.rosbot.image compose.yaml) \
        ros2 run rosbot_utils flash_firmware

# start containers on ROSbot 2R / 2 PRO
start-rosbot: _run-as-user
    #!/bin/bash
    if [[ "$SBC_NAME" == "RPI4" ]] || [[ "$SBC_NAME" == "UPBOARD" ]] || [[ "$USER" == "husarion" ]]; then
        trap 'docker compose down' SIGINT # Remove containers after CTRL+C
        docker compose pull
        docker compose up
        # if ffmpeg stream is not available in rviz
        # you need to trigger the stream manually by entering
        # astra container and running the following command:
        # /ros_entrypoint.sh && ros2 topic echo /rosbot2r/camera/color/image_raw/ffmpeg
    else
        echo "This command can be run only on ROSbot 2R / 2 PRO."
    fi

# start containers on PC
start-pc: _run-as-user
    #!/bin/bash
    xhost +local:docker
    trap 'docker compose -f compose.pc.yaml down rviz ros2router' SIGINT # Remove containers after CTRL+C
    docker compose -f compose.pc.yaml pull
    docker compose -f compose.pc.yaml up rviz ros2router

# run teleop_twist_keybaord (host)
run-teleop: _run-as-user
    #!/bin/bash
    export FASTRTPS_DEFAULT_PROFILES_FILE=$(pwd)/shm-only.xml
    ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r __ns:=/${ROBOT_NAMESPACE}

# run teleop_twist_keybaord (inside rviz2 container)
run-teleop-docker: _run-as-user
    #!/bin/bash
    docker compose -f compose.pc.yaml exec rviz /bin/bash -c "/ros_entrypoint.sh ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r __ns:=/${ROBOT_NAMESPACE}"

# enable the F710 gemapad (connected to your PC) to control ROSbot
run-joy: _run-as-user
    #!/bin/bash
    trap 'docker compose -f compose.pc.yaml down joy2twist' SIGINT # Remove containers after CTRL+C
    docker compose -f compose.pc.yaml up joy2twist


# copy repo content to remote host with 'rsync' and watch for changes
sync hostname="${ROBOT_NAMESPACE}" password="husarion": _install-rsync _run-as-user
    #!/bin/bash
    if ping -c 1 -W 3 {{hostname}} > /dev/null; then
        sshpass -p {{password}} rsync -vRr --exclude='.git/' --delete ./ husarion@{{hostname}}:/home/husarion/${PWD##*/}
        while inotifywait -r -e modify,create,delete,move ./ ; do
            sshpass -p "{{password}}" rsync --exclude='.git/' -vRr --delete ./ husarion@{{hostname}}:/home/husarion/${PWD##*/}
        done
    else
        echo -e "\e[93mUnable to reach the device or encountering a network issue. Verify the availability of your device in the Husarnet Network at https://app.husarnet.com/.\e[0m"; \
    fi

_run-as-root:
    #!/bin/bash
    if [ "$EUID" -ne 0 ]; then
        echo -e "\e[1;33mPlease re-run as root user to install dependencies\e[0m"
        exit 0
    fi

_run-as-user:
    #!/bin/bash
    if [ "$EUID" -eq 0 ]; then
        echo -e "\e[1;33mPlease re-run as non-root user\e[0m"
        exit 0
    fi

_install-rsync:
    #!/bin/bash
    if ! command -v rsync &> /dev/null || ! command -v sshpass &> /dev/null || ! command -v inotifywait &> /dev/null; then
        if [ "$EUID" -ne 0 ]; then
            echo -e "\e[1;33mPlease run as root to install dependencies\e[0m"
            exit 0
        fi
        apt install -y rsync sshpass inotify-tools
    fi

_install-yq:
    #!/bin/bash
    if ! command -v /usr/bin/yq &> /dev/null; then
        if [ "$EUID" -ne 0 ]; then
            echo -e "\e[1;33mPlease run as root to install dependencies\e[0m"
            exit 0
        fi

        YQ_VERSION=v4.35.1
        ARCH=$(arch)

        if [ "$ARCH" = "x86_64" ]; then
            YQ_ARCH="amd64"
        elif [ "$ARCH" = "aarch64" ]; then
            YQ_ARCH="arm64"
        else
            YQ_ARCH="$ARCH"
        fi

        curl -L https://github.com/mikefarah/yq/releases/download/${YQ_VERSION}/yq_linux_${YQ_ARCH} -o /usr/bin/yq
        chmod +x /usr/bin/yq
        echo "yq installed successfully!"
    fi
