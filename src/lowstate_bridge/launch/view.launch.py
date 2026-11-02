from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    rsp = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution(
            [FindPackageShare('go2_description'), 'launch', 'robot.launch.py'])),
        launch_arguments={'use_sim_time': 'false'}.items())

    bridge = Node(package='lowstate_bridge', executable='joint_bridge',
                  name='go2_joint_bridge')
    rviz = Node(package='rviz2', executable='rviz2')

    return LaunchDescription([rsp, bridge, rviz])
