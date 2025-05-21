# launch/dwa_planner.launch.py
import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, TextSubstitution
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # Declare launch arguments
    debug_arg = DeclareLaunchArgument(
        'debug', default_value='True',
        description='Enable simulation and RViz'
    )
    elevator_path_arg = DeclareLaunchArgument(
        'elevator_path', default_value='/elevator_path',
        description='Topic for elevator path input'
    )
    global_path_arg = DeclareLaunchArgument(
        'global_path', default_value='/global_path',
        description='Topic for global path input'
    )
    local_costmap_arg = DeclareLaunchArgument(
        'local_costmap', default_value='/campusrover_local_costmap',
        description='Topic for local costmap'
    )
    cmd_vel_arg = DeclareLaunchArgument(
        'cmd_vel', default_value='/input/nav_cmd_vel',
        description='Final cmd_vel output'
    )

    # RViz node group (only if debug == True)
    rviz_group = GroupAction(
        condition=IfCondition(LaunchConfiguration('debug')),
        actions=[
            # Node(
            #     package='robot_state_publisher',
            #     executable='robot_state_publisher',
            #     name='robot_state_publisher',
            #     output='screen'
            # ),
            # ... other simulator nodes, commented out ...
            Node(
                package='rviz2',
                executable='rviz2',
                name='rviz2',
                arguments=[
                    '-d',
                    PathJoinSubstitution([
                        TextSubstitution(text=os.path.join(
                            get_package_share_directory('campusrover_move'),
                            'rviz', 'dwa_planner.rviz'
                        ))
                    ])
                ],
                output='screen'
            ),
        ]
    )

    # Main DWA planner node
    dwa_node = Node(
        package='campusrover_move',
        executable='dwa_planner',
        name='dwa_planner',
        output='screen',
        parameters=[{
            # remappings via topic name
            'elevator_path': LaunchConfiguration('elevator_path'),
            'global_path': LaunchConfiguration('global_path'),
            'costmap': LaunchConfiguration('local_costmap'),
            'cmd_vel': LaunchConfiguration('cmd_vel'),
            # frame & arrival thresholds
            'robot_frame': 'base_link',
            'arriving_range_dis': 0.1,
            'arriving_range_angle': 0.05,
            # acceleration & velocity limits
            'max_linear_acceleration': 0.7,
            'max_angular_acceleration': 0.8,
            'max_linear_velocity': 1.0,
            'min_linear_velocity': 0.0,
            'max_angular_velocity': 3.0,
            'min_angular_velocity': 0.1,
            'target_point_dis': 0.8,
            # obstacle settings
            'threshold_occupied': 2.0,
            'footprint_max_x': 0.8,
            'footprint_min_x': -0.1,
            'footprint_max_y': 0.1,
            'footprint_min_y': -0.1,
            'obstacle_max_dis': 3.0,
            'obstacle_min_dis': 0.3,
            # objective function weights
            'obstable_dis_weight': 5.0,
            'target_heading_weight': 1.0,
            'velocity_weight': 200.0,
            # trajectory parameters
            'trajectory_num': 10,
            'min_angle_of_linear_profile': 0.1,
            'max_angle_of_linear_profile': 0.8,
            # modes & flags
            'enable_linear_depend_angular': True,
            'enble_costmap_obstacle': True,
            'direction_inverse': False
        }],
        remappings=[
            ('odom', 'odom_gt')
        ]
    )

    return LaunchDescription([
        debug_arg,
        elevator_path_arg,
        global_path_arg,
        local_costmap_arg,
        cmd_vel_arg,
        rviz_group,
        dwa_node,
    ])
