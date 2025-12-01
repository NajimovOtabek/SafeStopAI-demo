import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan, Imu, Image
from nav_msgs.msg import Odometry
from std_msgs.msg import String
import json
import time

class HealthMonitor(Node):
    def __init__(self):
        super().__init__('health_monitor')
        
        # Parameters
        self.declare_parameter('scan_timeout', 0.8)
        self.declare_parameter('camera_timeout', 1.0)
        self.declare_parameter('imu_timeout', 0.5)
        
        self.scan_timeout = self.get_parameter('scan_timeout').value
        self.camera_timeout = self.get_parameter('camera_timeout').value
        self.imu_timeout = self.get_parameter('imu_timeout').value

        # State variables (timestamps of last received messages)
        self.last_scan_time = time.time()
        self.last_cam_time = time.time()
        self.last_imu_time = time.time()
        self.last_odom_time = time.time()

        # Subscribers
        self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)
        self.create_subscription(Imu, '/imu', self.imu_callback, 10)
        self.create_subscription(Image, '/camera/image_raw', self.camera_callback, 10)
        self.create_subscription(Odometry, '/odom', self.odom_callback, 10)

        # Publisher
        self.fault_pub = self.create_publisher(String, '/fault_trigger', 10)

        # Monitor Loop
        self.timer = self.create_timer(0.1, self.check_health) # 10Hz check
        self.get_logger().info('Health Monitor Node Started')

    def scan_callback(self, msg):
        self.last_scan_time = time.time()

    def imu_callback(self, msg):
        self.last_imu_time = time.time()

    def camera_callback(self, msg):
        self.last_cam_time = time.time()

    def odom_callback(self, msg):
        self.last_odom_time = time.time()

    def check_health(self):
        current_time = time.time()
        fault_detected = False
        fault_msg = {}

        # Check LiDAR
        if current_time - self.last_scan_time > self.scan_timeout:
            fault_detected = True
            fault_msg = {
                "type": "SENSOR_LOSS_LIDAR",
                "timestamp": current_time,
                "detail": f"No /scan for {current_time - self.last_scan_time:.2f}s"
            }
            self.publish_fault(fault_msg)

        # Check Camera
        if current_time - self.last_cam_time > self.camera_timeout:
            # Only trigger if we haven't just triggered for LiDAR (priority) or trigger multiple?
            # For now, simple independent checks.
            # Note: In real sim, camera might not be publishing if not enabled, so this might spam.
            # We'll assume it should be there.
            pass 
            # Uncomment to enable camera check
            # fault_detected = True
            # fault_msg = {
            #     "type": "SENSOR_LOSS_CAMERA",
            #     "timestamp": current_time,
            #     "detail": f"No /camera/image_raw for {current_time - self.last_cam_time:.2f}s"
            # }
            # self.publish_fault(fault_msg)

    def publish_fault(self, fault_dict):
        msg = String()
        msg.data = json.dumps(fault_dict)
        self.fault_pub.publish(msg)
        self.get_logger().error(f"FAULT TRIGGERED: {fault_dict['type']} - {fault_dict['detail']}")

def main(args=None):
    rclpy.init(args=args)
    node = HealthMonitor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
