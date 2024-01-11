set dotenv-load

default: start-rosbot

# start containers on ROSbot 2R / 2 PRO
start-rosbot:
    docker compose up

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