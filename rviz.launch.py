from launch import LaunchDescription
from launch_ros.actions import Node, PushRosNamespace
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import EnvironmentVariable, LaunchConfiguration


def launch_setup(context, *args, **kwargs):
    robot_namespace = LaunchConfiguration("robot_namespace").perform(context)

    tf_remap = []
    if robot_namespace:
        tf_remap.append(("/tf", f"/{robot_namespace}/tf"))
        tf_remap.append(("/tf_static", f"/{robot_namespace}/tf_static"))

    rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        arguments=["-d", "/default.rviz"],
        remappings=tf_remap,
        output="screen",
    )

    decoder = Node(
        package="image_transport",
        executable="republish",
        name="republish",
        arguments=[
            "ffmpeg",
            "in/ffmpeg:=camera/color/image_raw/ffmpeg",
            "raw",
            "out:=camera/color/image_uncompressed",
        ],
        remappings=tf_remap,
        output="screen",
    )

    return [PushRosNamespace(robot_namespace), rviz, decoder]


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "robot_namespace",
                default_value=EnvironmentVariable("ROBOT_NAMESPACE", default_value=""),
                description="Namespace which will appear in front of all topics (including /tf and /tf_static).",
            ),
            OpaqueFunction(function=launch_setup),
        ]
    )
