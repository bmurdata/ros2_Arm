# Launch file for robot to allow for nodes to be run out of one terminal 
# Instead of 3 or more

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
import os
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import Command,LaunchConfiguration

def generate_launch_description():
    model_arg=DeclareLaunchArgument(
        name="model", 
        default_value=os.path.join(get_package_share_directory("arduinobot_description"),"urdf","arduinobot.urdf.xacro"),
        description="Absolute path to the robot URDF file"
        )
    # Convert xacro to URDF model, and run the node for the robot
    robot_description=ParameterValue(Command(['xacro ',LaunchConfiguration("model")]))
    robot_state_publisher=Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{
            "robot_description":robot_description
        }]
    )
    # Node for the Joints of the robot
    joint_state_publisher_gui=Node(
        package="joint_state_publisher_gui",
        executable="joint_state_publisher_gui"
    )
    # Deploy the model for visualization and running in Rviz2
    rviz_node=Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d",os.path.join(get_package_share_directory("arduinobot_description"),"rviz","display.rviz")]
    )

    return LaunchDescription([
        model_arg,
        robot_state_publisher,
        joint_state_publisher_gui,
        rviz_node
    ])