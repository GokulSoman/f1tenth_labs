from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_desription():
    talker_node = Node(executable="talker",
                 package="lab1_pkg",
                 output="screen")
    
    relay_node = Node(executable="relay",
                      package="lab1_pkg",
                      output="screen")
    
    return LaunchDescription(
        [talker_node, relay_node]
    )

