from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():

    # RViz for robot 1
    rviz_tb3_1 = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2_tb3_1',
        arguments=['-d', os.path.join(
            get_package_share_directory('nav2_bringup'),
            'rviz', 'nav2_default_view.rviz'
        )],
        parameters=[{'use_sim_time': True}],
        remappings=[
            ('/map', '/map'),
            ('/tf', '/tf'),
            ('/tf_static', '/tf_static'),
            ('/goal_pose', '/tb3_1/goal_pose'),
            ('/clicked_point', '/tb3_1/clicked_point'),
            ('/initialpose', '/tb3_1/initialpose'),
        ],
        output='screen'
    )

    # RViz for robot 2
    rviz_tb3_2 = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2_tb3_2',
        arguments=['-d', os.path.join(
            get_package_share_directory('nav2_bringup'),
            'rviz', 'nav2_default_view.rviz'
        )],
        parameters=[{'use_sim_time': True}],
        remappings=[
            ('/map', '/map'),
            ('/tf', '/tf'),
            ('/tf_static', '/tf_static'),
            ('/goal_pose', '/tb3_2/goal_pose'),
            ('/clicked_point', '/tb3_2/clicked_point'),
            ('/initialpose', '/tb3_2/initialpose'),
        ],
        output='screen'
    )

    return LaunchDescription([rviz_tb3_1, rviz_tb3_2])
