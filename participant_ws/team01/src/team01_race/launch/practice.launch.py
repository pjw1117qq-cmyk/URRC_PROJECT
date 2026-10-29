"""Spawn the Team 01 car into an already running official practice world."""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    EmitEvent,
    LogInfo,
    RegisterEventHandler,
)
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _spawn_finished(event, _context):
    if event.returncode != 0:
        reason = (
            "team01 spawn failed. Check that official practice is running, "
            "GZ_PARTITION matches, world_name is correct, and no team01 entity already exists."
        )
        return [LogInfo(msg=f"ERROR: {reason}"), EmitEvent(event=Shutdown(reason=reason))]
    return [LogInfo(msg="team01 spawned successfully in the existing practice world")]


def generate_launch_description():
    share = Path(get_package_share_directory("team01_race"))
    model = str(share / "models" / "team01_car" / "model.sdf")

    world_name = LaunchConfiguration("world_name")
    entity_name = LaunchConfiguration("entity_name")
    x = LaunchConfiguration("x")
    y = LaunchConfiguration("y")
    z = LaunchConfiguration("z")
    yaw = LaunchConfiguration("yaw")

    spawn = Node(
        package="ros_gz_sim",
        executable="create",
        name="team01_practice_spawn",
        output="screen",
        arguments=[
            "--world", world_name,
            "--file", model,
            "--name", entity_name,
            "--x", x,
            "--y", y,
            "--z", z,
            "-Y", yaw,
        ],
    )

    return LaunchDescription([
        DeclareLaunchArgument("world_name", default_value="monza"),
        DeclareLaunchArgument("entity_name", default_value="team01"),
        DeclareLaunchArgument("x", default_value="0.37867"),
        DeclareLaunchArgument("y", default_value="-2.05604"),
        DeclareLaunchArgument("z", default_value="1.24353"),
        DeclareLaunchArgument("yaw", default_value="1.45738"),
        spawn,
        RegisterEventHandler(
            OnProcessExit(target_action=spawn, on_exit=_spawn_finished)
        ),
    ])
