"""Humble Nav2 + 排他ゲート + Collision Monitor。velocity_smootherは使用しない。"""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.descriptions import ParameterFile
from nav2_common.launch import RewrittenYaml


def generate_launch_description():
    fw = get_package_share_directory('field_world')
    sim = LaunchConfiguration('use_sim_time')
    params = ParameterFile(RewrittenYaml(
        source_file=os.path.join(fw, 'config', 'nav2_params_odom_only.yaml'),
        param_rewrites={'use_sim_time': sim}, convert_types=True), allow_substs=True)
    specs = [('nav2_controller', 'controller_server'), ('nav2_smoother', 'smoother_server'),
             ('nav2_planner', 'planner_server'), ('nav2_behaviors', 'behavior_server'),
             ('nav2_bt_navigator', 'bt_navigator'), ('nav2_waypoint_follower', 'waypoint_follower')]
    nodes = [DeclareLaunchArgument('use_sim_time', default_value='true')]
    for package, name in specs:
        nodes.append(Node(package=package, executable=name, name=name, output='screen',
                          parameters=[params], remappings=[('cmd_vel', '/cmd_vel_nav_stack')]))
    nodes += [
        Node(package='nav2_lifecycle_manager', executable='lifecycle_manager',
             name='lifecycle_manager_navigation', output='screen',
             parameters=[{'use_sim_time': sim, 'autostart': True, 'node_names': [n for _,n in specs]}]),
        Node(package='field_world', executable='command_gate', name='box_command_gate',
             output='screen', parameters=[{'use_sim_time': sim}]),
        Node(package='nav2_collision_monitor', executable='collision_monitor', name='collision_monitor',
             output='screen', parameters=[os.path.join(fw,'config','collision_monitor.yaml'), {'use_sim_time': sim}]),
        Node(package='nav2_lifecycle_manager', executable='lifecycle_manager',
             name='lifecycle_manager_collision', output='screen',
             parameters=[{'use_sim_time': sim, 'autostart': True, 'node_names': ['collision_monitor']}])]
    return LaunchDescription(nodes)