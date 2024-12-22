from launch import LaunchDescription
from launch_ros.actions import Node
from pathlib import Path
from launch_ros.parameter_descriptions import ParameterValue
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Path to the zimo_description package
    zimo_description_dir = get_package_share_directory("zimo_description")

    # Declare launch argument for the URDF model
    model_arg = DeclareLaunchArgument(
        name="model",
        default_value=os.path.join(zimo_description_dir, "urdf", "zimo.urdf.xacro"),
        description="Absolute path to the URDF file"
    )

    # Set GZ_SIM_RESOURCE_PATH for Gazebo resources
    gazebo_resource_path = SetEnvironmentVariable(
        name="GZ_SIM_RESOURCE_PATH",
        value=[str(Path(zimo_description_dir).parent.resolve())]
    )

    # Declare is_ignition as a launch argument
    is_ignition_arg = DeclareLaunchArgument(
        name="is_ignition",
        default_value="true",
        description="Set to true if using Ignition Gazebo"
    )

    # Create a robot description using xacro
    robot_description = ParameterValue(Command([
        "xacro ",
        LaunchConfiguration("model"),
        " is_ignition:=",
        LaunchConfiguration("is_ignition")
        ]),
        value_type=str)

    # Node to publish robot state
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{
            "robot_description": robot_description,
            "use_sim_time": True
        }]
    )

    # Include Gazebo launch file
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(
                get_package_share_directory("ros_gz_sim"),
                "launch"
            ), "/gz_sim.launch.py"]
        ),
        launch_arguments=[
            ("gz_args", "-v 4 -r empty.sdf"),
        ]
    )

    # Node to spawn the robot in Gazebo
    gz_spawn_entity = Node(
        package="ros_gz_sim",
        executable="create",
        output="screen",
        arguments=["-topic", "robot_description", "-name", "zimo"]
    )

    # Bridge between ROS and Gazebo topics
    gz_ros2_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=[
            "/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock]"
        ]
    )

    # Return the complete LaunchDescription
    return LaunchDescription([
        model_arg,
        gazebo_resource_path,
        is_ignition_arg,
        robot_state_publisher,
        gazebo,
        gz_spawn_entity,
        gz_ros2_bridge
    ])
