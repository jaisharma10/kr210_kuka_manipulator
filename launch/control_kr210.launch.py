__author__ = 'jaisharm@umd.edu (jai)'
__maintainer__ = 'jaisharm@umd.edu (jai)'

import os
import launch
from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory
from launch.actions import ExecuteProcess
from launch_ros.actions import Node


def generate_launch_description() -> launch.LaunchDescription:
    
    ld = launch.LaunchDescription()

    # Set paths
    package_path = get_package_share_directory('kr210_kuka_manipulator')
    urdf_path = os.path.join(package_path, "urdf", 'kr210_gazebo.urdf')  
    controller_path = os.path.join(package_path, "config", "jtc.yaml")
    robot_description = {"robot_description": urdf_path}
    
    start_gazebo = ExecuteProcess(
        cmd=["gazebo","-s","libgazebo_ros_factory.so",],
        output="screen",
    )
    
    spawn_robot_node = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        arguments=["-entity","kr210_kuka_manipulator","-b","-file", urdf_path,],
    )

    robot_state_publisher_node =Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        arguments=[urdf_path],
    )
    
    controller_manager_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[robot_description, controller_path],
        output={
            "stdout": "screen",
            "stderr": "screen",
        },
    )
    
    joint_state_node = Node(
        package="controller_manager",
        executable="spawner.py",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
    )

    joint_trajectory_node = Node(
        package="controller_manager",
        executable="spawner.py",
        arguments=["joint_trajectory_controller", "-c", "/controller_manager"],
    )
    
    # call actions to launch nodes
    ld.add_action(start_gazebo)
    ld.add_action(spawn_robot_node)
    ld.add_action(robot_state_publisher_node)
    ld.add_action(controller_manager_node)
    ld.add_action(joint_state_node)
    ld.add_action(joint_trajectory_node)

    return ld
