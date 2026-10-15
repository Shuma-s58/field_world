"""field_world + orne-box + Nav2 (自己位置推定なし)

使う orne-box 側の部品は次の3つだけ:
  - orne_box_description の xacro (field_world/urdf/orne_box_sim.urdf.xacro 経由で
    MID360 のシミュレーション用センサを追加)
  - orne_box_bringup/launch/include/robot_localization_ekf.launch.py (odom->base_link)
使わないもの: emcl2, map_server, keepout, joy/teleop, waypoint_manager, 実機用 bringup
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, IncludeLaunchDescription,
                            OpaqueFunction, SetEnvironmentVariable)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (Command, FindExecutable, LaunchConfiguration,
                                  PathJoinSubstitution)
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

from field_world.generate_world import ROBOT_START, generate


def _setup(context, *args, **kwargs):
    seed = LaunchConfiguration('seed').perform(context)
    green = LaunchConfiguration('green_label').perform(context)
    world = LaunchConfiguration('world_out').perform(context)
    g, _ = generate(world, int(seed) if seed else None, green or None)
    print(f'[field_world] green_label={g}  answer: {world[:-6]}.answer.yaml')

    use_sim_time = {'use_sim_time': 'true'}
    inc = lambda path, args=None: IncludeLaunchDescription(  # noqa: E731
        PythonLaunchDescriptionSource(path), launch_arguments=(args or {}).items())

    bringup_inc = os.path.join(get_package_share_directory('orne_box_bringup'),
                               'launch', 'include')
    fw = get_package_share_directory('field_world')

    return [
        # Gazebo Classic + 生成したワールド
        inc(os.path.join(get_package_share_directory('gazebo_ros'),
                         'launch', 'gazebo.launch.py'), {'world': world}),

        # ロボットモデル (VLP-16 + MID360)。xacro は field_world のもの
        Node(package='robot_state_publisher', executable='robot_state_publisher',
             output='screen',
             parameters=[{'robot_description': ParameterValue(
                 Command([FindExecutable(name='xacro'), ' ',
                          os.path.join(fw, 'urdf', 'orne_box_sim.urdf.xacro'),
                          ' camera_x:=', LaunchConfiguration('camera_x'),
                          ' camera_y:=', LaunchConfiguration('camera_y'),
                          ' camera_z:=', LaunchConfiguration('camera_z'),
                          ' camera_pitch:=', LaunchConfiguration('camera_pitch')]),
                 value_type=str), 'use_sim_time': True}]),
        Node(package='joint_state_publisher', executable='joint_state_publisher',
             output='screen', parameters=[{'use_sim_time': True}]),
        Node(package='gazebo_ros', executable='spawn_entity.py', output='screen',
             arguments=['-entity', 'orne_box', '-topic', 'robot_description',
                        '-x', LaunchConfiguration('x'),
                        '-y', LaunchConfiguration('y'),
                        '-z', '0.0',
                        '-Y', LaunchConfiguration('yaw')]),

        # MID360(低い位置)の点群 -> LaserScan -> 自機の点を除去 -> /low_scan
        # (実機と同じ構成: pointcloud_to_laserscan -> laser_filters -> low_scan)
        # base_link は地面の高さにある。床(z≈0)を拾わないよう min_height は 0 より上にする
        Node(package='pointcloud_to_laserscan', executable='pointcloud_to_laserscan_node',
             name='low_pointcloud_to_laserscan',
             remappings=[('cloud_in', 'livox/lidar'), ('scan', 'low_surestar_scan')],
             parameters=[{'use_sim_time': True,
                          'target_frame': 'base_link',
                          'transform_tolerance': 0.1,
                          'min_height': 0.05,    # 床(z≈0)+余裕
                          'max_height': 0.60,
                          'angle_min': -1.5708, 'angle_max': 1.5708,  # 前方180°
                          'angle_increment': 0.0087,
                          'scan_time': 0.1,
                          'range_min': 0.3, 'range_max': 40.0,
                          'use_inf': True, 'inf_epsilon': 1.0}]),
        Node(package='laser_filters', executable='scan_to_scan_filter_chain',
             parameters=[os.path.join(fw, 'config', 'low_scan_filter.yaml')],
             remappings=[('scan', 'low_surestar_scan'), ('scan_filtered', 'low_scan')]),

        # EKF: odom->base_link の TF と /odometry/filtered を出す
        inc(os.path.join(bringup_inc, 'robot_localization_ekf.launch.py'), use_sim_time),

        # 自己位置推定なし: map->odom は「出発位置」で固定(スポーン位置は既知)。
        # これで map 座標 = Gazebo のワールド座標になり、answer.yaml の座標をそのままゴールに使える
        Node(package='tf2_ros', executable='static_transform_publisher',
             arguments=['--x', LaunchConfiguration('x'), '--y', LaunchConfiguration('y'),
                        '--z', '0', '--roll', '0', '--pitch', '0',
                        '--yaw', LaunchConfiguration('yaw'),
                        '--frame-id', 'map', '--child-frame-id', 'odom']),

        # Nav2 (標準の navigation_launch: map_server / AMCL を含まない)
        inc(os.path.join(get_package_share_directory('nav2_bringup'),
                         'launch', 'navigation_launch.py'),
            {'use_sim_time': 'true', 'autostart': 'true',
             'params_file': os.path.join(fw, 'config', 'nav2_params_odom_only.yaml')}),

        Node(package='rviz2', executable='rviz2', output='screen',
             condition=IfCondition(LaunchConfiguration('rviz')),
             arguments=['-d', os.path.join(get_package_share_directory('nav2_bringup'),
                                           'rviz', 'nav2_default_view.rviz')],
             parameters=[{'use_sim_time': True}]),
    ]


def generate_launch_description():
    models = os.path.join(get_package_share_directory('field_world'), 'models')
    prev = os.environ.get('GAZEBO_MODEL_PATH', '')
    return LaunchDescription([
        DeclareLaunchArgument('seed', default_value='', description='空なら毎回ランダム'),
        DeclareLaunchArgument('green_label', default_value='', description='a/b/c/d で固定'),
        DeclareLaunchArgument('world_out', default_value='/tmp/field_random.world'),
        # 緑Boxの上側(+y)3m から、緑Boxの方(-y 向き)を向いて出発
        DeclareLaunchArgument('x', default_value=f'{ROBOT_START[0]:.3f}'),
        DeclareLaunchArgument('y', default_value=f'{ROBOT_START[1]:.3f}'),
        DeclareLaunchArgument('yaw', default_value=f'{ROBOT_START[2]:.4f}'),
        DeclareLaunchArgument('rviz', default_value='true'),
        # カメラ取り付け位置 (base_link 基準 [m])。後方マスト上: camera_x:=-0.12 camera_z:=0.90
        DeclareLaunchArgument('camera_x', default_value='0.40'),
        DeclareLaunchArgument('camera_y', default_value='0.0'),
        DeclareLaunchArgument('camera_z', default_value='0.29'),
        DeclareLaunchArgument('camera_pitch', default_value='0.0'),
        SetEnvironmentVariable('GAZEBO_MODEL_PATH',
                               models + (':' + prev if prev else '')),
        OpaqueFunction(function=_setup),
    ])
