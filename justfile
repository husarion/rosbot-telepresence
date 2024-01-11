set dotenv-load

[private]
alias husarnet := connect-husarnet
[private]
alias rosbot := start-rosbot
[private]
alias pc := start-pc
[private]
alias teleop := run-teleop
[private]
alias teleop-docker := run-teleop-docker

default:
  @just --list --unsorted

# connect to Husarnet VPN network
connect-husarnet:
    #!/bin/bash
    if ! command -v husarnet > /dev/null; then \
        echo "Husarnet is not installed. Installing now..."; \
        curl https://install.husarnet.com/install.sh | sudo bash; \
    else \
        echo "Husarnet is already installed."; \
    fi
    husarnet join

# start containers on ROSbot 2R / 2 PRO
start-rosbot:
    #!/bin/bash
    if [[ "{{arch()}}" == "aarch64" ]]; then \
        docker compose up; \
    else \
        echo "This command can be run only on ROSbot 2R / 2 PRO."; \
    fi
    
# start containers on PC
start-pc:
    xhost +local:docker
    docker compose -f compose.pc.yaml up rviz ros2router

# run teleop_twist_keybaord (host)
run-teleop:
    #!/bin/bash
    export FASTRTPS_DEFAULT_PROFILES_FILE=$(pwd)/shm-only.xml
    ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r __ns:=/${ROBOT_NAMESPACE}

# run teleop_twist_keybaord (inside rviz2 container)
run-teleop-docker:
    docker compose -f compose.pc.yaml exec rviz /bin/bash -c "/ros_entrypoint.sh ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r __ns:=/${ROBOT_NAMESPACE}"