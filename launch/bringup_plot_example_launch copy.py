from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess

def generate_launch_description():
    cmd = ['MicroXRCEAgent', 'udp', '-p 2018']
    env = {'LD_PRELOAD': "/usr/local/lib/libfastrtps.so.1 /usr/local/lib/libfastcdr.so.1"}

    return LaunchDescription([
        Node(
            package='m5stack_example',
            node_executable='plot_example',
            output='screen'),
        ExecuteProcess(
            cmd=cmd,
            additional_env=env,
            output='screen'
        )
    ])
