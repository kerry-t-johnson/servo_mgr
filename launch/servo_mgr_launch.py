import launch
import launch.actions
import launch.substitutions
import launch_ros.actions
import platform

def generate_launch_description():
    return launch.LaunchDescription([
        launch_ros.actions.Node(package='servo_mgr',
                                executable='servo_mgr',
                                output='screen',
                                parameters = [
                                    {'use_mock_pca9685': platform.processor() != 'aarch64'}
                                ],
                                arguments = [
                                    '--ros-args', '--log-level', 'debug'
                                ]),
    ])
