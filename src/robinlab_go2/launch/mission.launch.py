"""mission.launch.py - Pila del robot Go2 con la librería launch de ROS 2.

Cada paso se lanza cuando el anterior está REALMENTE listo (un "gate" comprueba topic, TF o nodo),
en vez de esperar tiempos fijos. Si un paso crítico no está listo en su tiempo máximo, se hace Shutdown de
todo y no se lanzan los siguientes (cruise_teleop siempre es el último).

    ros2 launch robinlab_go2 mission.launch.py
    ros2 launch robinlab_go2 mission.launch.py enable_realsense:=false t_fastlio:=120

Orden: Livox -> gravity_alignment -> RealSense (opcional) -> FAST-LIO -> cmd_vel_bridge -> cruise_teleop
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

ARGS = {  # nombre: (valor por defecto, descripción)
    'gravity_pkg': ('robinlab_tools', 'paquete de gravity_alignment'),
    'go2_pkg': ('robinlab_go2', 'paquete de cmd_vel_bridge y cruise_teleop'),
    'enable_realsense': ('true', 'lanzar la RealSense'),
    'rs_args': ('rgb_camera.color_profile:=640x480x15 depth_module.depth_profile:=640x480x15',
                'argumentos de rs_launch.py'),
    'livox_imu_topic': ('/livox/imu', 'topic que indica que el Livox está vivo'),
    'rs_topic': ('/camera/camera/color/image_raw', 'topic que indica que la RealSense está viva'),
    'odom_topic': ('/Odometry', 'topic que indica que FAST-LIO está vivo'),
    't_livox': ('30', 's máx. esperando al Livox'),
    't_gravity': ('30', 's máx. esperando al TF de gravedad'),
    't_rs': ('40', 's máx. esperando a la RealSense'),
    't_fastlio': ('90', 's máx. esperando a FAST-LIO'),
    't_node': ('20', 's máx. esperando a un nodo'),
    'settle': ('2.0', 's de pausa tras cada paso listo'),
}


def _gate_topic(topic, timeout):
    return ExecuteProcess(
        cmd=['timeout', timeout, 'ros2', 'topic', 'echo', '--once', topic, '--qos-profile', 'sensor_data'],
        name='gate_topic', output='log')


def _gate_tf(parent, child, timeout):
    script = f"timeout {timeout} ros2 run tf2_ros tf2_echo {parent} {child} 2>/dev/null | grep -m1 -q 'At time'"
    return ExecuteProcess(cmd=['bash', '-c', script], name='gate_tf', output='log')


def _gate_node(node, timeout):
    script = (f"for i in $(seq 1 {timeout}); do ros2 node list 2>/dev/null | grep -qx {node} && exit 0; "
              "sleep 1; done; exit 1")
    return ExecuteProcess(cmd=['bash', '-c', script], name='gate_node', output='log')


def _build(context, *args, **kwargs):
    cfg = {k: LaunchConfiguration(k).perform(context) for k in ARGS}
    settle = max(float(cfg['settle']), 0.1)   # TimerAction necesita un periodo > 0

    # Cada paso: nombre, acción principal, gate (o None) y si es crítico.
    steps = [
        dict(name='Livox', critical=True,
             main=ExecuteProcess(cmd=['ros2', 'launch', 'livox_ros_driver2', 'msg_MID360.launch.py'],
                                 name='livox', output='screen', emulate_tty=True),
             gate=_gate_topic(cfg['livox_imu_topic'], cfg['t_livox'])),
        dict(name='Alineación de gravedad', critical=True,
             main=Node(package=cfg['gravity_pkg'], executable='gravity_alignment', output='screen'),
             gate=_gate_tf('gravity_world', 'camera_init', cfg['t_gravity'])),
    ]
    if cfg['enable_realsense'].lower() in ('true', '1', 'yes'):
        steps.append(dict(
            name='RealSense', critical=False,   # no es crítica para FAST-LIO: si falla, solo avisa
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
        dict(name='cruise_teleop', critical=True,   # SIEMPRE el último: es lo que puede mover el robot
             main=Node(package=cfg['go2_pkg'], executable='cruise_teleop', output='screen'),
             gate=_gate_node('/cruise_teleop_node', cfg['t_node'])),
    ]

    def proceed(i):
        """Acciones tras dar por bueno el paso i: siguiente paso (tras una pausa) o mensaje final."""
        if i + 1 < len(steps):
            return [TimerAction(period=settle, actions=[steps[i + 1]['main']])]
        return [LogInfo(msg='[mission] Todo listo. A = activar crucero, B = parar.')]

    def on_gate_exit(i):
        def handler(event, ctx):
            s = steps[i]
            if event.returncode == 0:
                return [LogInfo(msg=f"[mission] OK: {s['name']}")] + proceed(i)
            if s['critical']:
                return [LogInfo(msg=f"[mission] ERROR: {s['name']} no está listo (rc={event.returncode}). "
                                    "Se detiene todo y no se lanzan los pasos siguientes."),
                        Shutdown(reason=f"{s['name']} no listo")]
            return [LogInfo(msg=f"[mission] AVISO: {s['name']} no está listo; continúo sin esperarlo.")] + proceed(i)
        return handler

    out = [LogInfo(msg='[mission] AVISO: el robot debe estar de pie y QUIETO durante la alineación de '
                       'gravedad y el arranque de FAST-LIO.')]
    for i, s in enumerate(steps):
        out.append(RegisterEventHandler(OnProcessStart(target_action=s['main'], on_start=[s['gate']])))
        out.append(RegisterEventHandler(OnProcessExit(target_action=s['gate'], on_exit=on_gate_exit(i))))
    out.append(steps[0]['main'])   # arranca la cadena
    return out


def generate_launch_description():
    ld = [DeclareLaunchArgument(k, default_value=v, description=d) for k, (v, d) in ARGS.items()]
    # Un `docker exec` no hereda lo que exportó el entrypoint: fijamos DDS aquí si hace falta.
    if not os.environ.get('CYCLONEDDS_URI') and os.path.exists(CYCLONE_XML):
        ld.append(SetEnvironmentVariable('CYCLONEDDS_URI', 'file://' + CYCLONE_XML))
    ld.append(OpaqueFunction(function=_build))
    return LaunchDescription(ld)
