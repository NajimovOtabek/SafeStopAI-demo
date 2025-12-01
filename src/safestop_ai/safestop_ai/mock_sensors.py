import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan, Imu, Image
from nav_msgs.msg import Odometry
from std_msgs.msg import String
import time
import random
import json

class MockSensors(Node):
    def __init__(self):
        super().__init__('mock_sensors')
        
        # Parameters to control simulation
        self.declare_parameter('fail_lidar', False)
        self.declare_parameter('fail_camera', False)
        self.declare_parameter('fail_imu', False)
        
        # Publishers
        self.scan_pub = self.create_publisher(LaserScan, '/scan', 10)
        self.imu_pub = self.create_publisher(Imu, '/imu', 10)
        self.cam_pub = self.create_publisher(Image, '/camera/image_raw', 10)
        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)
        
        # Control Subscriber (to trigger faults dynamically)
        self.create_subscription(String, '/mock/control', self.control_callback, 10)

        # Timers
        self.create_timer(0.1, self.publish_scan)   # 10Hz
        self.create_timer(0.03, self.publish_camera) # ~30Hz
        self.create_timer(0.01, self.publish_imu)   # 100Hz
        self.create_timer(0.02, self.publish_odom)  # 50Hz

        self.get_logger().info('Mock Sensors Node Started. Use /mock/control to inject faults.')

    def control_callback(self, msg):
        command = json.loads(msg.data)
        if 'fail_lidar' in command:
            self.set_parameters([rclpy.Parameter('fail_lidar', rclpy.Parameter.Type.BOOL, command['fail_lidar'])])
            self.get_logger().warn(f"LIDAR FAILURE SET TO: {command['fail_lidar']}")
        if 'fail_camera' in command:
            self.set_parameters([rclpy.Parameter('fail_camera', rclpy.Parameter.Type.BOOL, command['fail_camera'])])
        if 'fail_imu' in command:
            self.set_parameters([rclpy.Parameter('fail_imu', rclpy.Parameter.Type.BOOL, command['fail_imu'])])

    def publish_scan(self):
        if self.get_parameter('fail_lidar').value:
            return
        msg = LaserScan()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'base_scan'
        msg.ranges = [random.uniform(0.5, 10.0) for _ in range(360)]
        self.scan_pub.publish(msg)

    def publish_camera(self):
        if self.get_parameter('fail_camera').value:
            return
        msg = Image()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'camera_link'
        # Empty image for mock
        self.cam_pub.publish(msg)

    def publish_imu(self):
        if self.get_parameter('fail_imu').value:
            return
        msg = Imu()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'imu_link'
        self.imu_pub.publish(msg)

    def publish_odom(self):
        # Odom usually doesn't fail in these simple scenarios, but could
        msg = Odometry()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'odom'
        self.odom_pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = MockSensors()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
