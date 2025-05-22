# campusrover_move/launch/sim_world.launch.py
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # 取得 Gazebo 與兩個套件的 share 目錄
    gazebo_pkg = get_package_share_directory('gazebo_ros')
    desc_pkg   = get_package_share_directory('charger_description')

    world_file = PathJoinSubstitution([
        get_package_share_directory('campusrover_move'),
        'worlds',
        'simple.world'          # ← 你的 world 檔名
    ])

    return LaunchDescription([
        # ① 啟動 Gazebo 並載入 simple.world
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(gazebo_pkg, 'launch', 'gazebo.launch.py')
            ),
            launch_arguments={'world': world_file}.items()
        ),

        # ② 把機器人 URDF 實體化到 Gazebo
        Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            name='spawn_campusrover',
            arguments=[
                '-entity', 'Charger_Chassis_20240829',
                '-file', os.path.join(desc_pkg, 'urdf', 'Charger_Chassis_20240829.urdf'),
                '-x', '0', '-y', '0', '-z', '0.1'
            ]
        ),

        # ③ robot_state_publisher（用 sim time）
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[{
                'robot_description': open(
                    os.path.join(desc_pkg, 'urdf', 'Charger_Chassis_20240829.urdf')
                ).read(),
                'use_sim_time': True
            }]
        ),

        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=[
                '-d', os.path.join(
                    get_package_share_directory('campusrover_move'),
                    'rviz',
                    'sim_planner.rviz'
                )
            ],
            parameters=[{'use_sim_time': True}]
        )
    ])
