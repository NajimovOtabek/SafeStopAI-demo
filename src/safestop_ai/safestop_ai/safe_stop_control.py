import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import String
import json
import time

class SafeStopControl(Node):
    def __init__(self):
        super().__init__('safe_stop_control')
        
        # Parameters
        self.declare_parameter('max_jerk', 1.0) # m/s^3
        self.declare_parameter('max_decel', 1.0) # m/s^2
        
        self.max_jerk = self.get_parameter('max_jerk').value
        self.max_decel = self.get_parameter('max_decel').value

        # State
        self.emergency_mode = False
        self.current_vel = Twist()
        self.target_vel = Twist()
        self.last_update_time = time.time()

        # Subscribers
        self.create_subscription(Twist, '/cmd_vel_nav', self.nav_cmd_callback, 10)
        self.create_subscription(String, '/fault_trigger', self.fault_callback, 10)

        # Publisher
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # Control Loop
        self.timer = self.create_timer(0.05, self.control_loop) # 20Hz
        self.get_logger().info('Safe Stop Control Node Started')

    def nav_cmd_callback(self, msg):
        if not self.emergency_mode:
            self.target_vel = msg

    def fault_callback(self, msg):
        fault_data = json.loads(msg.data)
        self.get_logger().warn(f"SafeStopControl received fault: {fault_data['type']}")
        
        # Engage Emergency Mode immediately
        self.emergency_mode = True
        self.get_logger().error("EMERGENCY MODE ENGAGED: Decelerating to 0.")

    def control_loop(self):
        dt = time.time() - self.last_update_time
        self.last_update_time = time.time()

        if self.emergency_mode:
            # Execute Safe Stop Profile (Decelerate to 0)
            # Simple ramp down for now
            if abs(self.current_vel.linear.x) > 0.01:
                decel = self.max_decel * dt
                if self.current_vel.linear.x > 0:
                    self.current_vel.linear.x = max(0.0, self.current_vel.linear.x - decel)
                else:
                    self.current_vel.linear.x = min(0.0, self.current_vel.linear.x + decel)
            else:
                self.current_vel.linear.x = 0.0
            
            self.current_vel.angular.z = 0.0 # Stabilize yaw
            
        else:
            # Pass through (with potential smoothing if we wanted)
            self.current_vel = self.target_vel

        self.cmd_pub.publish(self.current_vel)

def main(args=None):
    rclpy.init(args=args)
    node = SafeStopControl()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
