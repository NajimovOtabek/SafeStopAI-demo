import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from visualization_msgs.msg import Marker
import json

class HMIAlert(Node):
    def __init__(self):
        super().__init__('hmi_alert')
        
        # Subscribers
        self.create_subscription(String, '/fault_trigger', self.fault_callback, 10)

        # Publisher for RViz Marker
        self.marker_pub = self.create_publisher(Marker, '/hmi/fault_marker', 10)

        self.get_logger().info('HMI Alert Node Started')

    def fault_callback(self, msg):
        fault_data = json.loads(msg.data)
        self.get_logger().error(f"!!! HMI ALERT: {fault_data['type']} !!!")
        self.get_logger().error(f"Detail: {fault_data['detail']}")
        
        # Publish Visual Marker (Text in RViz)
        marker = Marker()
        marker.header.frame_id = "base_link"
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "hmi"
        marker.id = 0
        marker.type = Marker.TEXT_VIEW_FACING
        marker.action = Marker.ADD
        marker.pose.position.z = 0.5
        marker.scale.z = 0.2
        marker.color.a = 1.0
        marker.color.r = 1.0
        marker.color.g = 0.0
        marker.color.b = 0.0
        marker.text = f"FAULT: {fault_data['type']}"
        
        self.marker_pub.publish(marker)

def main(args=None):
    rclpy.init(args=args)
    node = HMIAlert()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
