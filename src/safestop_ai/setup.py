from setuptools import setup

package_name = 'safestop_ai'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/safestop.launch.py', 'launch/verification.launch.py']),
        ('share/' + package_name + '/rviz', ['rviz/safestop.rviz']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='user',
    maintainer_email='user@todo.todo',
    description='Vehicle SafeStop AI package for TurtleBot3',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'health_monitor = safestop_ai.health_monitor:main',
            'emergency_planner = safestop_ai.emergency_planner:main',
            'safe_stop_control = safestop_ai.safe_stop_control:main',
            'v2x_bridge = safestop_ai.v2x_bridge:main',
            'hmi_alert = safestop_ai.hmi_alert:main',
            'mock_sensors = safestop_ai.mock_sensors:main',
            'test_scenario_runner = safestop_ai.test_scenario_runner:main',
        ],
    },
)
