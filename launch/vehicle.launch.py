"""Launch an independent RC car under a deployment-selected namespace."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    arguments = ("namespace", "device_id", "profile_id", "build_id")
    return LaunchDescription([
        *(DeclareLaunchArgument(name, default_value="") for name in arguments),
        Node(
            package="rc_car_interface",
            executable="vehicle_interface",
            namespace=LaunchConfiguration("namespace"),
            parameters=[{
                name: ParameterValue(LaunchConfiguration(name), value_type=str)
                for name in arguments if name != "namespace"
            }],
            output="screen",
        ),
    ])
