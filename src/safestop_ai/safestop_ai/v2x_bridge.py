import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import json
import time
import paho.mqtt.client as mqtt

class V2XBridge(Node):
    def __init__(self):
        super().__init__('v2x_bridge')
        
        # Parameters
        self.declare_parameter('mqtt_broker', 'test.mosquitto.org')
        self.declare_parameter('mqtt_port', 1883)
        self.declare_parameter('vehicle_id', 't4-demo-01')
        
        self.broker = self.get_parameter('mqtt_broker').value
        self.port = self.get_parameter('mqtt_port').value
        self.vehicle_id = self.get_parameter('vehicle_id').value

        # MQTT Client
        self.client = mqtt.Client(self.vehicle_id)
        try:
            self.client.connect(self.broker, self.port, 60)
            self.client.loop_start()
            self.get_logger().info(f"Connected to MQTT Broker: {self.broker}")
        except Exception as e:
            self.get_logger().error(f"Failed to connect to MQTT Broker: {e}")

        # Subscriber
        self.create_subscription(String, '/fault_trigger', self.fault_callback, 10)

    def fault_callback(self, msg):
        fault_data = json.loads(msg.data)
        
        # Construct V2X Alert
        alert = {
            "vehicle_id": self.vehicle_id,
            "ts": time.time(),
            "lat": 37.00001, # Mock GPS
            "lon": 127.00002, # Mock GPS
            "heading_deg": 85.3, # Mock Heading
            "mode": "EMERGENCY_STOP",
            "reason": fault_data.get("type", "UNKNOWN"),
            "est_stop_time_s": 4.5,
            "location_type": "shoulder"
        }
        
        payload = json.dumps(alert)
        topic = "v2x/alerts/safestop"
        
        self.client.publish(topic, payload)
        self.get_logger().info(f"Published V2X Alert to {topic}: {payload}")

    def destroy_node(self):
        self.client.loop_stop()
        self.client.disconnect()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = V2XBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
