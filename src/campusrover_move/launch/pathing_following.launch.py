from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, GroupAction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, EnvironmentVariable
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import PathJoinSubstitution, EnvironmentVariable
import os

def generate_launch_description():
    # === 宣告變數（對應 ROS 1 的 $(arg ...)）===
    debug = LaunchConfiguration('debug')  
    bag_filename = LaunchConfiguration('bag_filename')  
    elevator_path = LaunchConfiguration('elevator_path')  
    global_path = LaunchConfiguration('global_path')
    local_costmap = LaunchConfiguration('local_costmap')
    cmd_vel = LaunchConfiguration('cmd_vel')
    enble_dwa_obstacle_avoidance = LaunchConfiguration('enble_dwa_obstacle_avoidance')
    enble_pullover_mode = LaunchConfiguration('enble_pullover_mode')

    campusrover_move_share = get_package_share_directory('campusrover_move')
    campusrover_description_share = get_package_share_directory('campusrover_description')

    # === 開始回傳整體 Launch Description ===
    return LaunchDescription([
        # === 宣告 launch 參數 ===
        DeclareLaunchArgument('debug', default_value='false'),
        
        DeclareLaunchArgument('bag_filename',
            default_value=PathJoinSubstitution([
                EnvironmentVariable('HOME'),
                'bags/campusrover_bag/20200409_itc_3f_2s.bag'
            ])
        ),
        DeclareLaunchArgument('elevator_path', default_value='/elevator_path'),
        DeclareLaunchArgument('global_path', default_value='/global_path'),
        DeclareLaunchArgument('local_costmap', default_value='/campusrover_local_costmap'),
        DeclareLaunchArgument('cmd_vel', default_value='/input/nav_cmd_vel'),
        DeclareLaunchArgument('enble_dwa_obstacle_avoidance', default_value='false'),
        DeclareLaunchArgument('enble_pullover_mode', default_value='false'),

        # === Debug 模式 Group：rosbag + robot_state_publisher + rviz2 ===
        GroupAction([
            Node(
                package='rosbag2',
                executable='play',
                name='playbag',
                arguments=['--clock', bag_filename],
                condition=IfCondition(debug)  
            ),
            # 修改 robot_state_publisher 以使用一個簡單的機器人描述而非從外部套件讀取
            Node(
                package='robot_state_publisher',
                executable='robot_state_publisher',
                name='robot_state_publisher',
                parameters=[{
                    'robot_description': open(
                        os.path.join(campusrover_description_share, 'urdf', 'campusrover.urdf')
                    ).read(),
                    'use_sim_time': True
                }],
                condition=IfCondition(debug)
            ),
            Node(
                package='rviz2',
                executable='rviz2',
                name='rviz',
                arguments=['-d', os.path.join(campusrover_move_share, 'rviz', 'dp_planner.rviz')],
                condition=IfCondition(debug)
            )
        ]),

        # === 主導航節點：path_following ===
        Node(
            package='campusrover_move',
            executable='path_following',
            name='path_following',
            output='screen',
            remappings=[
                ('elevator_path', elevator_path),
                ('global_path', global_path),
                ('costmap', local_costmap),
                ('cmd_vel', cmd_vel)
            ],
            parameters=[{
                'robot_frame': 'base_link',
                'arriving_range_dis': 0.1,
                'arriving_range_angle': 0.05,
                'max_linear_velocity': 0.5,
                'max_angular_velocity': 1.0,
                'target_point_dis': 0.6,
                'threshold_occupied': 2.0,
                'footprint_max_x': 1.0,
                'footprint_min_x': -0.1,
                'footprint_max_y': 0.1,
                'footprint_min_y': -0.1,
                'speed_pid_k': 0.8,
                'min_angle_of_linear_profile': 0.1,
                'max_angle_of_linear_profile': 0.8,
                'obstacle_range': 0.3,
                'enable_linear_depend_angular': True,
                'enble_costmap_obstacle': True,
                'direction_inverse': False,
                'enble_dwa_obstacle_avoidance': enble_dwa_obstacle_avoidance,
                'enble_pullover_mode': enble_pullover_mode,
                'use_sim_time': True
            }]
        ),

        # === 根據 flag 包含 DWA planner 的子 launch 檔 ===
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(campusrover_move_share, 'launch', 'dwa_planner.launch.py')
            ),
            condition=IfCondition(enble_dwa_obstacle_avoidance)
        ),

        # === 根據 flag 包含 pullover planner 的子 launch 檔 ===
        # IncludeLaunchDescription(
        #     PythonLaunchDescriptionSource(
        #         os.path.join(campusrover_move_share, 'launch', 'pullover_path_planner.launch.py')
        #     ),
        #     condition=IfCondition(enble_pullover_mode)
        # ),
    ])
