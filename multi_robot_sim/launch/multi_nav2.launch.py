from launch import LaunchDescription
from launch.actions import GroupAction, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import PushRosNamespace
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    nav2_bringup = get_package_share_directory('nav2_bringup')

    robot1_nav = GroupAction([
        PushRosNamespace('tb3_1'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(nav2_bringup, 'launch', 'bringup_launch.py')
            ),
            launch_arguments={
                'use_namespace': 'True',
                'namespace': 'tb3_1',
                'map': '/path/to/map.yaml',
                'use_sim_time': 'True',
                'params_file': os.path.join(
                    get_package_share_directory('multi_robot_sim'),
                    'config', 'nav2_params_tb3_1.yaml'
                ),
            }.items()
        )
    ])

    robot2_nav = GroupAction([
        PushRosNamespace('tb3_2'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(nav2_bringup, 'launch', 'bringup_launch.py')
            ),
            launch_arguments={
                'use_namespace': 'True',
                'namespace': 'tb3_2',
                'map': '/path/to/map.yaml',
                'use_sim_time': 'True',
                'params_file': os.path.join(
                    get_package_share_directory('multi_robot_sim'),
                    'config', 'nav2_params_tb3_2.yaml'
                ),
            }.items()
        )
    ])

    return LaunchDescription([robot1_nav, robot2_nav])
