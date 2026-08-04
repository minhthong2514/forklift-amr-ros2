import os
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    dockinglive_node = Node(
        package="car_controller",
        executable="dockinglive",
        name="dockinglive_node",
        output="screen",
    )

    master_brain_node = Node(
        package="car_controller",
        executable="master_brain", 
        name="master_brain_node",
        output="screen",
    )
    
    return LaunchDescription([
        dockinglive_node,
        master_brain_node
    ])