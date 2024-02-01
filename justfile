set dotenv-load # to read ROBOT_NAMESPACE from .env file

[private]
alias husarnet := connect-husarnet
[private]
alias flash := flash-firmware
[private]
alias rosbot := start-rosbot
[private]
alias start := start-rosbot

[private]
default:
  @just --list --unsorted

_install-rsync:
    #!/bin/bash
    if ! command -v rsync &> /dev/null; then \
        if [ "$EUID" -ne 0 ]; then \
            echo -e "\e[93mPlease run as root to install dependencies\e[0m"; \
            exit 1; \
        fi

        sudo apt update && sudo apt install -y rsync
    fi

_install-yq:
    #!/bin/bash
    if ! command -v /usr/bin/yq &> /dev/null; then \
        if [ "$EUID" -ne 0 ]; then \
            echo -e "\e[93mPlease run as root to install dependencies\e[0m"; \
            exit 1; \
        fi

        YQ_VERSION=v4.35.1
        ARCH=$(arch)

        if [ "$ARCH" = "x86_64" ]; then \
            YQ_ARCH="amd64"; \
        elif [ "$ARCH" = "aarch64" ]; then \
            YQ_ARCH="arm64"; \
        else \
            YQ_ARCH="$ARCH"; \
        fi

        curl -L https://github.com/mikefarah/yq/releases/download/${YQ_VERSION}/yq_linux_${YQ_ARCH} -o /usr/bin/yq
        chmod +x /usr/bin/yq
        echo "yq installed successfully!"
    fi

# connect to Husarnet VPN network
connect-husarnet joincode hostname:
    #!/bin/bash
    if [ "$EUID" -ne 0 ]; then \
        echo -e "\e[93mPlease run as root to install Husarnet\e[0m"; \
        exit; \
    fi
    if ! command -v husarnet > /dev/null; then \
        echo "Husarnet is not installed. Installing now..."; \
        curl https://install.husarnet.com/install.sh | sudo bash; \
    fi
    husarnet join {{joincode}} {{hostname}}

# flash the proper firmware for STM32 microcontroller in ROSbot 2R / 2 PRO
flash-firmware: _install-yq
    #!/bin/bash
    echo "Stopping all running containers"
    docker stop $(docker ps -q -a)

    echo "Flashing the firmware for STM32 microcontroller in ROSbot"
    docker run \
        --rm -it --privileged \
        $(yq .services.rosbot.image compose.yaml) \
        ros2 run rosbot_utils flash_firmware

# start containers on ROSbot 2R / 2 PRO
start-rosbot:
    #!/bin/bash
    if [[ $USER == "husarion" ]]; then \
        trap 'docker compose down' SIGINT # Remove containers after CTRL+C
        docker compose pull; \
        docker compose up; \
    else \
        echo "This command can be run only on ROSbot 2R / 2 PRO."; \
    fi


# copy repo content to remote host with 'rsync' and watch for changes
sync hostname="${ROBOT_NAMESPACE}" password="husarion":  _install-rsync
    #!/bin/bash
    sshpass -p "husarion" rsync -vRr --delete ./ husarion@{{hostname}}:/home/husarion/${PWD##*/}
    while inotifywait -r -e modify,create,delete,move ./ ; do
        sshpass -p "{{password}}" rsync -vRr --delete ./ husarion@{{hostname}}:/home/husarion/${PWD##*/}
    done
