from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='safestop_ai',
            executable='health_monitor',
            name='health_monitor',
            output='screen',
            parameters=[
                {'scan_timeout': 0.8},
                {'camera_timeout': 1.0},
                {'imu_timeout': 0.5}
            ]
        ),
        Node(
            package='safestop_ai',
            executable='emergency_planner',
            name='emergency_planner',
            output='screen'
        ),
        Node(
            package='safestop_ai',
            executable='safe_stop_control',
            name='safe_stop_control',
            output='screen',
            parameters=[
                {'max_jerk': 1.0},
                {'max_decel': 1.0}
            ]
        ),
        Node(
            package='safestop_ai',
            executable='v2x_bridge',
            name='v2x_bridge',
            output='screen',
            parameters=[
                {'mqtt_broker': 'test.mosquitto.org'},
                {'vehicle_id': 't4-demo-01'}
            ]
        ),
        Node(
            package='safestop_ai',
            executable='hmi_alert',
            name='hmi_alert',
            output='screen'
        )
    ])
