import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan, Imu, Image
from nav_msgs.msg import Odometry
from std_msgs.msg import String
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster, StaticTransformBroadcaster
from visualization_msgs.msg import Marker
import math
import random
import json
import time

class MockSensors(Node):
    def __init__(self):
        super().__init__('mock_sensors')
        
        # Parameters
        self.declare_parameter('fail_lidar', False)
        self.declare_parameter('fail_camera', False)
        self.declare_parameter('fail_imu', False)
        
        # Publishers
        self.scan_pub = self.create_publisher(LaserScan, '/scan', 10)
        self.imu_pub = self.create_publisher(Imu, '/imu', 10)
        self.cam_pub = self.create_publisher(Image, '/camera/image_raw', 10)
        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)
        self.marker_pub = self.create_publisher(Marker, '/visualization_marker', 10)
        
        # TF Broadcasters
        self.tf_broadcaster = TransformBroadcaster(self)
        self.static_broadcaster = StaticTransformBroadcaster(self)
        
        # Publish static map -> odom transform
        self.publish_static_tf()

        # Control Subscriber
        self.create_subscription(String, '/mock/control', self.control_callback, 10)

        # Simulation State
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.linear_velocity = 0.2  # m/s
        self.angular_velocity = 0.1 # rad/s
        self.last_time = self.get_clock().now()

        # Timers
        self.create_timer(0.1, self.publish_scan)   # 10Hz
        self.create_timer(0.03, self.publish_camera) # ~30Hz
        self.create_timer(0.01, self.publish_imu)   # 100Hz
        self.create_timer(0.05, self.update_physics_and_odom)  # 20Hz

        self.get_logger().info('Mock Sensors Node Started. Simulating circular motion.')

    def publish_static_tf(self):
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'map'
        t.child_frame_id = 'odom'
        t.transform.translation.x = 0.0
        t.transform.translation.y = 0.0
        t.transform.translation.z = 0.0
        t.transform.rotation.w = 1.0
        self.static_broadcaster.sendTransform(t)

    def control_callback(self, msg):
        command = json.loads(msg.data)
        if 'fail_lidar' in command:
            self.set_parameters([rclpy.Parameter('fail_lidar', rclpy.Parameter.Type.BOOL, command['fail_lidar'])])
            self.get_logger().warn(f"LIDAR FAILURE SET TO: {command['fail_lidar']}")
        if 'fail_camera' in command:
            self.set_parameters([rclpy.Parameter('fail_camera', rclpy.Parameter.Type.BOOL, command['fail_camera'])])
        if 'fail_imu' in command:
            self.set_parameters([rclpy.Parameter('fail_imu', rclpy.Parameter.Type.BOOL, command['fail_imu'])])

    def update_physics_and_odom(self):
        current_time = self.get_clock().now()
        dt = (current_time - self.last_time).nanoseconds / 1e9
        self.last_time = current_time

        # Update pose (Circular motion)
        self.theta += self.angular_velocity * dt
        self.x += self.linear_velocity * math.cos(self.theta) * dt
        self.y += self.linear_velocity * math.sin(self.theta) * dt

        # Quaternion from yaw
        qz = math.sin(self.theta / 2.0)
        qw = math.cos(self.theta / 2.0)

        # 1. Publish Odom Message
        odom = Odometry()
        odom.header.stamp = current_time.to_msg()
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_link'
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.orientation.z = qz
        odom.pose.pose.orientation.w = qw
        self.odom_pub.publish(odom)

        # 2. Publish TF (odom -> base_link)
        t = TransformStamped()
        t.header.stamp = current_time.to_msg()
        t.header.frame_id = 'odom'
        t.child_frame_id = 'base_link'
        t.transform.translation.x = self.x
        t.transform.translation.y = self.y
        t.transform.translation.z = 0.0
        t.transform.rotation.z = qz
        t.transform.rotation.w = qw
        self.tf_broadcaster.sendTransform(t)

        # 3. Publish Robot Marker (for RViz)
        marker = Marker()
        marker.header.frame_id = "base_link"
        marker.header.stamp = current_time.to_msg()
        marker.ns = "robot"
        marker.id = 0
        marker.type = Marker.CUBE
        marker.action = Marker.ADD
        marker.scale.x = 0.4
        marker.scale.y = 0.3
        marker.scale.z = 0.2
        marker.color.a = 1.0

        # Visual Logic: Green = OK, Red = Fault
        if self.get_parameter('fail_lidar').value or \
           self.get_parameter('fail_camera').value or \
           self.get_parameter('fail_imu').value:
            marker.color.r = 1.0
            marker.color.g = 0.0
            marker.color.b = 0.0
        else:
            marker.color.r = 0.0
            marker.color.g = 1.0
            marker.color.b = 0.0

        self.marker_pub.publish(marker)

    def publish_scan(self):
        if self.get_parameter('fail_lidar').value:
            return
        msg = LaserScan()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'base_link' # Attached to robot
        msg.angle_min = 0.0
        msg.angle_max = 6.28
        msg.angle_increment = 0.017 # 1 degree
        msg.range_min = 0.12
        msg.range_max = 3.5
        msg.ranges = [random.uniform(0.5, 3.0) for _ in range(360)]
        self.scan_pub.publish(msg)

    def publish_camera(self):
        if self.get_parameter('fail_camera').value:
            return
        msg = Image()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'base_link'
        self.cam_pub.publish(msg)

    def publish_imu(self):
        if self.get_parameter('fail_imu').value:
            return
        msg = Imu()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'base_link'
        self.imu_pub.publish(msg)

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
