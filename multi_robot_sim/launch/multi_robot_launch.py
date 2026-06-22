from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():

    urdf = os.path.join(
    	get_package_share_directory('multi_robot_sim'),
   	'urdf', 'turtlebot3_burger_sim.urdf'
	)

    with open(urdf, 'r') as f:
        robot_desc = f.read()

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('gazebo_ros'),
                'launch', 'gazebo.launch.py'
            )
        )
    )

    rsp_tb3_1 = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        namespace='tb3_1',
        parameters=[{
            'robot_description': robot_desc,
            'use_sim_time': True,
            'frame_prefix': 'tb3_1/'
        }],
        output='screen'
    )

    rsp_tb3_2 = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        namespace='tb3_2',
        parameters=[{
            'robot_description': robot_desc,
            'use_sim_time': True,
            'frame_prefix': 'tb3_2/'
        }],
        output='screen'
    )

    spawn_tb3_1 = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-entity', 'tb3_1',
            '-topic', '/tb3_1/robot_description',
            '-x', '0.0', '-y', '0.0', '-z', '0.01',
            '-robot_namespace', 'tb3_1'
        ],
        output='screen'
    )

    spawn_tb3_2 = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-entity', 'tb3_2',
            '-topic', '/tb3_2/robot_description',
            '-x', '1.5', '-y', '0.0', '-z', '0.01',
            '-robot_namespace', 'tb3_2'
        ],
        output='screen'
    )

    return LaunchDescription([
        gazebo,
        rsp_tb3_1,
        rsp_tb3_2,
        spawn_tb3_1,
        spawn_tb3_2
    ])
