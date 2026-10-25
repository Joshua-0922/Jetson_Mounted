"""One drop episode in Gazebo Harmonic + PX4 v1.15.4 SITL with the Isaac Lab policy.

  ros2 launch drop_policy drop_eval.launch.py world:=/tmp/drop_world.sdf params:=/tmp/ep.yaml wind_x:=0 wind_y:=0

Processes: MicroXRCEAgent, gz sim (server, real time), ros_gz_bridge, PX4 SITL attached to the
pre-spawned x500_drop (airframe 4020_gz_x500_drop), drone_controller (LPF alpha 0.4 = Isaac),
drop_mission_node. The launch ends when drop_mission_node exits (result written).
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, ExecuteProcess, RegisterEventHandler, TimerAction
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

PX4 = "/opt/PX4-Autopilot"


def generate_launch_description():
    share = get_package_share_directory("drop_policy")
    world = LaunchConfiguration("world")
    params = LaunchConfiguration("params")
    wx, wy = LaunchConfiguration("wind_x"), LaunchConfiguration("wind_y")
    res_path = ":".join(["/workspace/gazebo_models", f"{PX4}/Tools/simulation/gz/models"])
    env = f"mkdir -p /tmp/runtime-root && export XDG_RUNTIME_DIR=/tmp/runtime-root && export GZ_SIM_RESOURCE_PATH={res_path} && "

    gz = ExecuteProcess(cmd=["bash", "-c", [env, "gz sim -s -r --headless-rendering ", world]], name="gz_sim", output="log")
    wind = TimerAction(period=4.0, actions=[ExecuteProcess(
        cmd=["bash", "-c", ["gz topic -t /ocean_current -m gz.msgs.Vector3d -p 'x: ", wx, " y: ", wy, " z: 0'"]],
        name="set_wind", output="screen")])
    agent = ExecuteProcess(cmd=["MicroXRCEAgent", "udp4", "-p", "8888"], name="xrce_agent", output="log")
    bridge = Node(package="ros_gz_bridge", executable="parameter_bridge", name="drop_bridge", output="log",
                  parameters=[{"config_file": os.path.join(share, "config", "gz_bridge.yaml")}])
    px4 = TimerAction(period=6.0, actions=[ExecuteProcess(
        cmd=["bash", "-c",
             env + f"rm -f {PX4}/build/px4_sitl_default/rootfs/parameters*.bson; "
             f"cd {PX4}/build/px4_sitl_default/rootfs && PX4_GZ_STANDALONE=1 PX4_GZ_WORLD=drop_world "
             f"PX4_GZ_MODEL_NAME=x500_drop PX4_SIM_MODEL=gz_x500_drop "
             f"{PX4}/build/px4_sitl_default/bin/px4 -d > /tmp/px4_drop.log 2>&1"],
        name="px4_sitl", output="log")])
    ctrl = TimerAction(period=10.0, actions=[Node(package="drone_controller", executable="controller",
                                                    name="drone_controller", output="log",
                                                    parameters=[{"velocity_lpf_alpha": 0.4, "use_sim_time": True}])])
    mission = Node(package="drop_policy", executable="drop_mission", name="drop_mission_node",
                   output="screen", parameters=[params, {"use_sim_time": True}])
    mission_t = TimerAction(period=10.0, actions=[mission])
    return LaunchDescription([
        DeclareLaunchArgument("world"), DeclareLaunchArgument("params"),
        DeclareLaunchArgument("wind_x", default_value="0"), DeclareLaunchArgument("wind_y", default_value="0"),
        agent, gz, wind, bridge, px4, ctrl, mission_t,
        RegisterEventHandler(OnProcessExit(target_action=mission, on_exit=[EmitEvent(event=Shutdown())])),
    ])
