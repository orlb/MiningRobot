import launch
import launch_ros.actions

def generate_launch_description():
    return launch.LaunchDescription([
        launch_ros.actions.Node(
            package='aurora_core',
            namespace='aurora1',
            executable='aurora_node_core',
            name='aurora_node_core',
            output='screen',
            emulate_tty=True,
            arguments=[('/tmp/data_exchange/backend.drive')]
        )
    ])
