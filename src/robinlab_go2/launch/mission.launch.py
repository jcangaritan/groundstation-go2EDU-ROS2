"""mission.launch.py - Go2 robot stack using the ROS 2 launch library.

Each step is launched only when the previous one is REALLY ready (a "gate" checks a topic, a TF or a node)
instead of sleeping for fixed times. If a critical step is not ready within its maximum wait, everything is
shut down and the following steps are never launched (cruise_teleop is always the last one).

    ros2 launch robinlab_go2 mission.launch.py
    ros2 launch robinlab_go2 mission.launch.py enable_realsense:=false t_fastlio:=120

Order: Livox -> gravity_alignment -> RealSense (optional) -> FAST-LIO -> cmd_vel_bridge -> cruise_teleop
"""
import os
import shlex

from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, ExecuteProcess, LogInfo, OpaqueFunction,
                            RegisterEventHandler, SetEnvironmentVariable, Shutdown, TimerAction)
from launch.event_handlers import OnProcessExit, OnProcessStart
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

CYCLONE_XML = '/root/robinlab_ws/robot-env/cyclonedds.xml'

ARGS = {  # name: (default value, description)
    'gravity_pkg': ('robinlab_tools', 'package that contains gravity_alignment'),
    'go2_pkg': ('robinlab_go2', 'package that contains cmd_vel_bridge and cruise_teleop'),
    'enable_realsense': ('true', 'launch the RealSense camera'),
    'rs_args': ('rgb_camera.color_profile:=640x480x15 depth_module.depth_profile:=640x480x15',
                'arguments passed to rs_launch.py'),
    'livox_launch': ('msg_MID360_launch.py',
                     'launch file of the Livox driver (check: find ~/drivers_ws/install -name "*MID360*")'),
    'livox_imu_topic': ('/livox/imu', 'topic that tells the Livox driver is alive'),
    'rs_topic': ('/camera/camera/color/image_raw', 'topic that tells the RealSense is alive'),
    'odom_topic': ('/Odometry', 'topic that tells FAST-LIO is alive'),
    't_livox': ('30', 'max seconds waiting for the Livox'),
    't_gravity': ('30', 'max seconds waiting for the gravity TF'),
    't_rs': ('40', 'max seconds waiting for the RealSense'),
    't_fastlio': ('90', 'max seconds waiting for FAST-LIO'),
    't_node': ('20', 'max seconds waiting for a node'),
    'settle': ('2.0', 'seconds to pause after each step is ready'),
}


def _gate_topic(topic, timeout):
    # `ros2 topic echo` exits IMMEDIATELY ("Could not determine the type...") if the topic does not exist yet,
    # so we must retry until the maximum wait instead of making a single call.
    script = (f"end=$((SECONDS+{timeout})); while [ $SECONDS -lt $end ]; do "
              f"timeout 5 ros2 topic echo --once {topic} --qos-profile sensor_data >/dev/null 2>&1 && exit 0; "
              "sleep 1; done; exit 1")
    return ExecuteProcess(cmd=['bash', '-c', script], name='gate_topic', output='log')


def _gate_tf(parent, child, timeout):
    script = f"timeout {timeout} ros2 run tf2_ros tf2_echo {parent} {child} 2>/dev/null | grep -m1 -q 'At time'"
    return ExecuteProcess(cmd=['bash', '-c', script], name='gate_tf', output='log')


def _gate_node(node, timeout):
    script = (f"for i in $(seq 1 {timeout}); do ros2 node list 2>/dev/null | grep -qx {node} && exit 0; "
              "sleep 1; done; exit 1")
    return ExecuteProcess(cmd=['bash', '-c', script], name='gate_node', output='log')


def _build(context, *args, **kwargs):
    cfg = {k: LaunchConfiguration(k).perform(context) for k in ARGS}
    settle = max(float(cfg['settle']), 0.1)   # TimerAction needs a period > 0

    # Each step: name, main action, gate and whether it is critical.
    steps = [
        dict(name='Livox', critical=True,
             main=ExecuteProcess(cmd=['ros2', 'launch', 'livox_ros_driver2', cfg['livox_launch']],
                                 name='livox', output='screen', emulate_tty=True),
             gate=_gate_topic(cfg['livox_imu_topic'], cfg['t_livox'])),
        dict(name='Gravity alignment', critical=True,
             main=Node(package=cfg['gravity_pkg'], executable='gravity_alignment', output='screen'),
             gate=_gate_tf('gravity_world', 'camera_init', cfg['t_gravity'])),
    ]
    if cfg['enable_realsense'].lower() in ('true', '1', 'yes'):
        steps.append(dict(
            name='RealSense', critical=False,   # not critical for FAST-LIO: if it fails we only warn
            main=ExecuteProcess(cmd=['ros2', 'launch', 'realsense2_camera', 'rs_launch.py'] + shlex.split(cfg['rs_args']),
                                name='realsense', output='screen', emulate_tty=True),
            gate=_gate_topic(cfg['rs_topic'], cfg['t_rs'])))
    steps += [
        dict(name='FAST-LIO2', critical=True,
             main=ExecuteProcess(cmd=['ros2', 'launch', 'fast_lio', 'mapping.launch.py', 'rviz:=false'],
                                 name='fastlio', output='screen', emulate_tty=True),
             gate=_gate_topic(cfg['odom_topic'], cfg['t_fastlio'])),
        dict(name='cmd_vel_bridge', critical=True,
             main=Node(package=cfg['go2_pkg'], executable='cmd_vel_bridge', output='screen'),
             gate=_gate_node('/cmd_vel_bridge', cfg['t_node'])),
        dict(name='cruise_teleop', critical=True,   # ALWAYS the last one: it is what can move the robot
             main=Node(package=cfg['go2_pkg'], executable='cruise_teleop', output='screen'),
             gate=_gate_node('/cruise_teleop_node', cfg['t_node'])),
    ]

    def proceed(i):
        """Actions after step i is considered good: next step (after a pause) or the final message."""
        if i + 1 < len(steps):
            return [TimerAction(period=settle, actions=[steps[i + 1]['main']])]
        return [LogInfo(msg='[mission] All ready. A = start cruise, B = stop.')]

    def on_gate_exit(i):
        def handler(event, ctx):
            s = steps[i]
            if event.returncode == 0:
                return [LogInfo(msg=f"[mission] OK: {s['name']}")] + proceed(i)
            if s['critical']:
                return [LogInfo(msg=f"[mission] ERROR: {s['name']} is not ready (rc={event.returncode}). "
                                    "Shutting everything down; the following steps will not be launched."),
                        Shutdown(reason=f"{s['name']} not ready")]
            return [LogInfo(msg=f"[mission] WARNING: {s['name']} is not ready; continuing without it.")] + proceed(i)
        return handler

    out = [LogInfo(msg='[mission] WARNING: the robot must be standing and STILL during gravity alignment '
                       'and FAST-LIO start-up.')]
    for i, s in enumerate(steps):
        out.append(RegisterEventHandler(OnProcessStart(target_action=s['main'], on_start=[s['gate']])))
        out.append(RegisterEventHandler(OnProcessExit(target_action=s['gate'], on_exit=on_gate_exit(i))))
    out.append(steps[0]['main'])   # starts the chain
    return out


def generate_launch_description():
    ld = [DeclareLaunchArgument(k, default_value=v, description=d) for k, (v, d) in ARGS.items()]
    # `docker exec` does not inherit what the entrypoint exported: set DDS here if needed.
    if not os.environ.get('CYCLONEDDS_URI') and os.path.exists(CYCLONE_XML):
        ld.append(SetEnvironmentVariable('CYCLONEDDS_URI', 'file://' + CYCLONE_XML))
    ld.append(OpaqueFunction(function=_build))
    return LaunchDescription(ld)
