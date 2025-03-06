import os

from ament_index_python.packages import get_package_share_directory  # type: ignore

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument  # type: ignore
from launch.substitutions import LaunchConfiguration  # type: ignore
from launch_ros.actions import Node  # type: ignore

def generate_launch_description():
    # Get the param directory
    pkg_dir = get_package_share_directory('navigation2_run')
    param_dir = os.path.join(pkg_dir, 'params')

    params_file = LaunchConfiguration('params_file')

    declare_params_file_cmd = DeclareLaunchArgument(
        'params_file',
        default_value=os.path.join(param_dir, 'script.yaml'),
        description='Full path to the ROS2 parameters file to use for script simulation'
    )
    
    run_script_cmd = Node(
        package='navigation2_run',
        executable='script_sim',
        arguments=[params_file],
        name='ScriptSim',
        output='screen',
    )

    ld = LaunchDescription()

    ld.add_action(declare_params_file_cmd)
    ld.add_action(run_script_cmd)

    return ld
