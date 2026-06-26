from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import TimerAction

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='tsp_digital_twin',
            executable='vehicle_publisher',
            name='vehicle_publisher',
            output='screen',
        ),
        Node(
            package='tsp_digital_twin',
            executable='tsp_controller',
            name='tsp_controller',
            output='screen',
        ),
        TimerAction(period=3.0, actions=[
            Node(
                package='tsp_digital_twin',
                executable='signal_controller',
                name='signal_controller',
                output='screen',
            ),
        ]),
        Node(
            package='tsp_digital_twin',
            executable='metrics_logger',
            name='metrics_logger',
            output='screen',
        ),
    ])