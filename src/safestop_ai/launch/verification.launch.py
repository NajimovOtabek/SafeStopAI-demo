from launch import LaunchDescription
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    return LaunchDescription([
        # 1. Mock Sensors (Simulates LiDAR, Camera, IMU)
        Node(
            package='safestop_ai',
            executable='mock_sensors',
            name='mock_sensors',
            output='screen',
            parameters=[
                {'fail_lidar': False},
                {'fail_camera': False},
                {'fail_imu': False}
            ]
        ),
        
        # 2. System Under Test (Health Monitor, V2X, HMI)
        Node(
            package='safestop_ai',
            executable='health_monitor',
            name='health_monitor',
            output='screen',
            parameters=[
                {'scan_timeout': 0.8}
            ]
        ),
        Node(
            package='safestop_ai',
            executable='v2x_bridge',
            name='v2x_bridge',
            output='screen'
        ),
        Node(
            package='safestop_ai',
            executable='hmi_alert',
            name='hmi_alert',
            output='screen'
        ),

        # 3. Test Runner (Injects faults and verifies response)
        Node(
            package='safestop_ai',
            executable='test_scenario_runner',
            name='test_scenario_runner',
            output='screen'
        ),

        # 4. Visualization
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', [os.path.join(get_package_share_directory('safestop_ai'), 'rviz', 'safestop.rviz')]],
            output='screen'
        ),

        # 5. Web Bridge (for Investor Dashboard)
        Node(
            package='rosbridge_server',
            executable='rosbridge_websocket',
            name='rosbridge_websocket',
            output='screen'
        )
    ])
