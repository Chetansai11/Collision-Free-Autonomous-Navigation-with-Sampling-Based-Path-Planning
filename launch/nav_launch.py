#!/usr/bin/env python3
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    ld = LaunchDescription()

    # Launch parameters
    ld.add_action(DeclareLaunchArgument(
        'obstacles',
        # default_value="5,-3,1.5; 5,0,1.5; 5,3,1.5; 10,10,1.0; -5,8,1.5; -3,-6,1.5"
        # default_value="-2,4,1; -2,1,1; -2,-2,1; -2,-5,1; 5,5,1.3; 5,-4,1.3"
        # default_value="0,6,1.5; 4,4,1.5; 6,0,1.5; 4,-4,1.5; 0,-6,1.5; -4,-4,1.5; -6,0,1.5; -4,4,1.5"
        default_value="0,6,1.2; 3,3,1.2; -3,1,1.2; 3,-1,1.2; -3,-4,1.2"


    ))
    ld.add_action(DeclareLaunchArgument('map_min_x', default_value='-10.0'))
    ld.add_action(DeclareLaunchArgument('map_max_x', default_value='10.0'))
    ld.add_action(DeclareLaunchArgument('map_min_y', default_value='-10.0'))
    ld.add_action(DeclareLaunchArgument('map_max_y', default_value='10.0'))
    ld.add_action(DeclareLaunchArgument('resolution', default_value='0.05'))
    ld.add_action(DeclareLaunchArgument('robot_radius', default_value='0.7'))
    ld.add_action(DeclareLaunchArgument('rviz_config', default_value='nav.rviz'))


    #Simulator node (world frame source)
    sim_node = Node(
        package="sim",
        executable="sim1",
        name="sim1",
        output="screen"
    )

    #Path planner
    planner = Node(
        package='projectz',
        executable='path_planner',
        name='path_planner',
        output='screen',
        parameters=[{
            'obstacles': LaunchConfiguration('obstacles'),
            'map_min_x': LaunchConfiguration('map_min_x'),
            'map_max_x': LaunchConfiguration('map_max_x'),
            'map_min_y': LaunchConfiguration('map_min_y'),
            'map_max_y': LaunchConfiguration('map_max_y'),
            'resolution': LaunchConfiguration('resolution'),
            'robot_radius': LaunchConfiguration('robot_radius'),
            'frame_id': 'world',
            'planner_type': 'rrtstar',    # or 'astar'
            'rrt_max_iters': 3000,
            'rrt_step_size': 0.4,
            'rrt_goal_radius': 0.5,
            'rrt_rewire_radius': 0.8,
            'rrt_goal_sample_rate': 0.10,
        }]
    )


    #Controller (uses /pose as PoseStamped)
    controller = Node(
        package='projectz',
        executable='controller',
        name='controller',
        output='screen',
        parameters=[{
            'odom_topic': '/pose',
            'lookahead': 1.0,
            'max_lin': 2.5,
            'max_ang': 4.0,
            'goal_tolerance': 0.15,
            'frame_id': 'world',
            'linear_k': 1.5,
            'angular_k': 1.5,
        }]
    )


    #RViz
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', LaunchConfiguration('rviz_config')],
        output='screen'
    )

    #Make map = world (identity transform)
    static_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        arguments=["0", "0", "0", "0", "0", "0", "world", "map"],
        name="world_to_map"
    )

    ld.add_action(sim_node)
    ld.add_action(planner)
    ld.add_action(controller)
    ld.add_action(static_tf)
    ld.add_action(rviz)

    return ld
