from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    bug_zapper = Node(
        package="bug_zapper",
        executable="BUG_ZAP",
    )

    bug_sim = Node(
        package="bug_zapper",
        executable="BUG_SIM",
    )

    return LaunchDescription([
        bug_zapper,
        bug_sim,
    ])