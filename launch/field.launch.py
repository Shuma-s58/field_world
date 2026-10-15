import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, IncludeLaunchDescription,
                            OpaqueFunction, SetEnvironmentVariable)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

from field_world.generate_world import generate


def _setup(context, *args, **kwargs):
    seed = LaunchConfiguration('seed').perform(context)
    green = LaunchConfiguration('green_label').perform(context)
    out = LaunchConfiguration('world_out').perform(context)
    g, _ = generate(out, int(seed) if seed else None, green or None)
    print(f'[field_world] world={out}  green_label={g}  (answer: {out[:-6]}.answer.yaml)')

    gazebo_ros = get_package_share_directory('gazebo_ros')
    return [IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gazebo_ros, 'launch', 'gazebo.launch.py')),
        launch_arguments={'world': out}.items())]


def generate_launch_description():
    models = os.path.join(get_package_share_directory('field_world'), 'models')
    prev = os.environ.get('GAZEBO_MODEL_PATH', '')
    return LaunchDescription([
        DeclareLaunchArgument('seed', default_value='',
                              description='乱数シード。空なら毎回ランダム'),
        DeclareLaunchArgument('green_label', default_value='',
                              description='緑Boxのラベルを固定(a/b/c/d)。空ならランダム'),
        DeclareLaunchArgument('world_out', default_value='/tmp/field_random.world'),
        SetEnvironmentVariable('GAZEBO_MODEL_PATH',
                               models + (':' + prev if prev else '')),
        OpaqueFunction(function=_setup),
    ])
