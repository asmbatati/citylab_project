from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
import os

def generate_launch_description():
    ld = LaunchDescription()

    go_to_pose_action_node = Node(
            package="robot_patrol",
            executable="go_to_pose_action",
            name="go_to_pose_action_node",
            output="screen"
        )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        output='screen',
        name='sim_rviz2',
        arguments=['-d' + os.path.join(get_package_share_directory('robot_patrol'), 'rviz/robot_patrol.rviz')]
    )

    ld.add_action(go_to_pose_action_node)
    ld.add_action(rviz_node)

    return ld