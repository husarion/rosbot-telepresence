set dotenv-load

run-teleop:
    #!/bin/bash
    export FASTRTPS_DEFAULT_PROFILES_FILE=$(pwd)/shm-only.xml
    ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r __ns:=/${ROBOT_NAMESPACE}

run-teleop-docker:
    docker compose -f compose.pc.yaml exec rviz /bin/bash -c "/ros_entrypoint.sh ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r __ns:=/${ROBOT_NAMESPACE}"