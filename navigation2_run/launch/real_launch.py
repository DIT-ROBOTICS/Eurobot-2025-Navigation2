# Copyright (c) 2018 Intel Corporation
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""This is all-in-one launch script intended for use by nav2 developers."""

import os
import sys

from ament_index_python.packages import get_package_share_directory # type: ignore
# from launch_ros.substitutions import FindPackageShare # type: ignore

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription # type: ignore
from launch.conditions import IfCondition # type: ignore
from launch.launch_description_sources import PythonLaunchDescriptionSource # type: ignore
from launch.substitutions import LaunchConfiguration, PythonExpression # type: ignore
from launch_ros.actions import Node # type: ignore


def generate_launch_description():
    # Get the launch directory
    pkg_dir = get_package_share_directory('navigation2_run')
    launch_dir = os.path.join(pkg_dir, 'launch')
    ros_domain_id = os.getenv('ROS_DOMAIN_ID')

    if ros_domain_id == '11':
        params_file_name = 'nav2_params_11.yaml'
        print('[INFO] [real_launch] ROS_DOMAIN_ID=11. Use nav2_params_11.yaml')
    elif ros_domain_id == '14':
        params_file_name = 'nav2_params_14.yaml'
        print('[INFO] [real_launch] ROS_DOMAIN_ID=14. Use nav2_params_14.yaml')
    else:
        params_file_name = 'nav2_params_default.yaml'
        print(f'[INFO] [sim_launch] Unrecognized ROS_DOMAIN_ID={ros_domain_id}. Use default params file')

    # Create the launch configuration variables
    namespace = LaunchConfiguration('namespace')
    use_namespace = LaunchConfiguration('use_namespace')
    map_yaml_file = LaunchConfiguration('map')
    use_sim_time = LaunchConfiguration('use_sim_time')
    params_file = LaunchConfiguration('params_file')
    autostart = LaunchConfiguration('autostart')
    use_composition = LaunchConfiguration('use_composition')
    use_respawn = LaunchConfiguration('use_respawn')
    use_odometry_sim = LaunchConfiguration('use_odometry_sim')
    robot_pose_remap = LaunchConfiguration('robot_pose_remap')

    # Launch configuration variables specific to simulation
    rviz_config_file = LaunchConfiguration('rviz_config_file')
    use_rviz = LaunchConfiguration('use_rviz')

    # Declare the launch arguments
    declare_namespace_cmd = DeclareLaunchArgument(
        'namespace',
        default_value='',
        description='Top-level namespace')

    declare_use_namespace_cmd = DeclareLaunchArgument(
        'use_namespace',
        default_value='false',
        description='Whether to apply a namespace to the navigation stack')

    declare_map_yaml_cmd = DeclareLaunchArgument(
        'map',
        default_value=os.path.join(
            pkg_dir, 'maps', 'basic_map.yaml'),
        description='Full path to map file to load')

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation (Gazebo) clock if true')

    declare_params_file_cmd = DeclareLaunchArgument(
        'params_file',
        default_value=os.path.join(pkg_dir, 'params', params_file_name),
        description='Full path to the ROS2 parameters file to use for all launched nodes')

    declare_autostart_cmd = DeclareLaunchArgument(
        'autostart', default_value='true',
        description='Automatically startup the nav2 stack')

    declare_use_composition_cmd = DeclareLaunchArgument(
        'use_composition', default_value='True',
        description='Whether to use composed bringup')

    declare_use_respawn_cmd = DeclareLaunchArgument(
        'use_respawn', default_value='False',
        description='Whether to respawn if a node crashes. Applied when composition is disabled.')

    declare_rviz_config_file_cmd = DeclareLaunchArgument(
        'rviz_config_file',
        default_value=os.path.join(
            pkg_dir, 'rviz', 'rviz_sim.rviz'),
        description='Full path to the RVIZ config file to use')

    declare_use_rviz_cmd = DeclareLaunchArgument(
        'use_rviz',
        default_value='False',
        description='Whether to start RVIZ')
    
    declare_use_odometry_sim_cmd = DeclareLaunchArgument(
        'use_odometry_sim',
        default_value='False',
        description='Whether to start odometry simulation')
    
    declare_robot_pose_remap_cmd = DeclareLaunchArgument(
        'robot_pose_remap',
        default_value='/final_pose_nav',
        description='Remapping for robot pose topic')

    rviz_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(launch_dir, 'rviz_launch.py')),
        condition=IfCondition(use_rviz),
        launch_arguments={'namespace': namespace,
                          'use_namespace': use_namespace,
                          'rviz_config': rviz_config_file}.items())

    bringup_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(launch_dir, 'bringup_launch.py')),
        launch_arguments={'namespace': namespace,
                          'use_namespace': use_namespace,
                          'map': map_yaml_file,
                          'use_sim_time': use_sim_time,
                          'params_file': params_file,
                          'autostart': autostart,
                          'use_composition': use_composition,
                          'use_odometry_sim': use_odometry_sim,
                          'use_respawn': use_respawn,
                          'robot_pose_remap': robot_pose_remap}.items())
    
    final_pose_bridge_cmd = Node(
        package='navigation2_run',
        executable='final_pose_bridge',
        name='final_pose_bridge',
        output='screen',
        parameters=[params_file]
    )

    system_check_cmd = Node(
        package='navigation2_run',
        executable='system_check',
        name='system_check',
        output='screen'
    )

    # Create the launch description and populate
    ld = LaunchDescription()

    # Declare the launch options
    ld.add_action(declare_namespace_cmd)
    ld.add_action(declare_use_namespace_cmd)
    # ld.add_action(declare_slam_cmd)
    ld.add_action(declare_map_yaml_cmd)
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_params_file_cmd)
    ld.add_action(declare_autostart_cmd)
    ld.add_action(declare_use_composition_cmd)

    ld.add_action(declare_rviz_config_file_cmd)
    ld.add_action(declare_use_rviz_cmd)
    ld.add_action(declare_use_odometry_sim_cmd)
    ld.add_action(declare_use_respawn_cmd)
    ld.add_action(declare_robot_pose_remap_cmd)

    # Add any conditioned actions

    # Add the actions to launch all of the navigation nodes
    ld.add_action(rviz_cmd)
    ld.add_action(bringup_cmd)

    # Add the final pose bridge node
    ld.add_action(final_pose_bridge_cmd)

    # Add the system check node
    ld.add_action(system_check_cmd)

    return ld
