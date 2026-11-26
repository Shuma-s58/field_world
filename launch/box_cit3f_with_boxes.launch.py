#!/usr/bin/env python3

import os
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
    SetEnvironmentVariable,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from field_world.generate_world import _delivery_box, _pickup_box


def setup_boxes(context):
    green_label = LaunchConfiguration("green_label").perform(context)
    blue_label = LaunchConfiguration("blue_label").perform(context)

    green_x = LaunchConfiguration("green_x").perform(context)
    green_y = LaunchConfiguration("green_y").perform(context)
    green_yaw = LaunchConfiguration("green_yaw").perform(context)

    blue_x = LaunchConfiguration("blue_x").perform(context)
    blue_y = LaunchConfiguration("blue_y").perform(context)
    blue_yaw = LaunchConfiguration("blue_yaw").perform(context)

    # モデル自体は原点に生成し、spawn_entity.pyで配置する
    green_model = _pickup_box(
        "green_box", 0.0, 0.0, 0.0, green_label
    )
    blue_model = _delivery_box(
        "blue_box", 0.0, 0.0, 0.0, blue_label
    )

    green_sdf = "/tmp/field_green_box.sdf"
    blue_sdf = "/tmp/field_blue_box.sdf"

    Path(green_sdf).write_text(
        '<?xml version="1.0"?>\n'
        '<sdf version="1.6">\n'
        f"{green_model}\n"
        "</sdf>\n"
    )

    Path(blue_sdf).write_text(
        '<?xml version="1.0"?>\n'
        '<sdf version="1.6">\n'
        f"{blue_model}\n"
        "</sdf>\n"
    )

    return [
        Node(
            package="gazebo_ros",
            executable="spawn_entity.py",
            arguments=[
                "-entity", "green_box",
                "-file", green_sdf,
                "-x", green_x,
                "-y", green_y,
                "-z", "0.0",
                "-Y", green_yaw,
                "-timeout", "120.0",
            ],
            output="screen",
        ),
        Node(
            package="gazebo_ros",
            executable="spawn_entity.py",
            arguments=[
                "-entity", "blue_box",
                "-file", blue_sdf,
                "-x", blue_x,
                "-y", blue_y,
                "-z", "0.0",
                "-Y", blue_yaw,
                "-timeout", "120.0",
            ],
            output="screen",
        ),
    ]


def generate_launch_description():
    field_share = get_package_share_directory("field_world")
    orne_sim_share = get_package_share_directory("orne_box_simulation")

    box_cit3f_launch = os.path.join(
        orne_sim_share,
        "launch",
        "box_cit3f.launch.py",
    )

    field_models = os.path.join(field_share, "models")
    previous_model_path = os.environ.get("GAZEBO_MODEL_PATH", "")
    gazebo_model_path = (
        field_models + (":" + previous_model_path if previous_model_path else "")
    )

    return LaunchDescription([
        DeclareLaunchArgument("gui", default_value="True"),
        DeclareLaunchArgument("use_sim_time", default_value="True"),
        DeclareLaunchArgument(
            "simulation_lidar",
            default_value="3d",
            choices=["2d", "3d"],
        ),

        DeclareLaunchArgument(
            "green_label",
            default_value="a",
            choices=["a", "b", "c", "d"],
        ),
        DeclareLaunchArgument("green_x", default_value="-1.0"),
        DeclareLaunchArgument("green_y", default_value="0.0"),
        DeclareLaunchArgument("green_yaw", default_value="-1.5708"),

        DeclareLaunchArgument(
            "blue_label",
            default_value="a",
            choices=["a", "b", "c", "d"],
        ),
        DeclareLaunchArgument("blue_x", default_value="2.0"),
        DeclareLaunchArgument("blue_y", default_value="5.0"),
        DeclareLaunchArgument("blue_yaw", default_value="1.5708"),

        # ラベル画像をGazeboから検索できるようにする
        SetEnvironmentVariable(
            "GAZEBO_MODEL_PATH",
            gazebo_model_path,
        ),

        # Tsudanuma world、orne-box、センサなどを起動
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(box_cit3f_launch),
            launch_arguments={
                "gui": LaunchConfiguration("gui"),
                "use_sim_time": LaunchConfiguration("use_sim_time"),
                "simulation_lidar": LaunchConfiguration("simulation_lidar"),
            }.items(),
        ),

        # 同じGazeboへボックスを追加
        OpaqueFunction(function=setup_boxes),
    ])
