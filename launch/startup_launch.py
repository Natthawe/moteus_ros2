import os
import launch
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import RegisterEventHandler, EmitEvent, IncludeLaunchDescription, DeclareLaunchArgument
from launch.event_handlers import (OnExecutionComplete, OnProcessExit,OnProcessIO, OnProcessStart, OnShutdown)
from launch.events import Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression

def generate_launch_description():

    # moteus_drive
    launch_moteus_drive_dir = get_package_share_directory('moteus_drive')
    launch_moteus_drive_pkg = os.path.join(launch_moteus_drive_dir, 'launch')

    # moteus_vel
    moteus_vel_dir = get_package_share_directory('moteus_vel')
    launch_moteus_vel_pkg = os.path.join(moteus_vel_dir, 'launch')

    # joy_tester
    joy_tester_dir = get_package_share_directory('joy_tester')
    launch_joy_tester_pkg = os.path.join(joy_tester_dir, 'launch')

    # accel_desel
    accel_desel_dir = get_package_share_directory('accel_desel')
    launch_accel_desel_pkg = os.path.join(accel_desel_dir, 'launch')

    # scurve_twist
    # scurve_twist_dir = get_package_share_directory('scurve_twist')
    # launch_scurve_twist_pkg = os.path.join(scurve_twist_dir, 'launch')    



    return LaunchDescription([

    # moteus_drive       
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(launch_moteus_drive_pkg, 'moteus_node.launch.py'))
        ),

    # moteus_vel        
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(launch_moteus_vel_pkg, 'moteus_vel_node.launch.py'))
        ),

    # joy_tester
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(launch_joy_tester_pkg, 'joy_stick.launch.py'))
        ),

    # accel_desel        
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(launch_accel_desel_pkg, 'accel_desel_node.launch.py'))
        ),

    # scurve_twist
        # IncludeLaunchDescription(
        #     PythonLaunchDescriptionSource(os.path.join(launch_scurve_twist_pkg, 'scurve_twist_launch.py'))
        # ),        


    ])