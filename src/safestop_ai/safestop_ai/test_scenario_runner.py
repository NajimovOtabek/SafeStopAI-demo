import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import json
import time
import sys

class TestScenarioRunner(Node):
    def __init__(self):
        super().__init__('test_scenario_runner')
        
        # Publishers
        self.control_pub = self.create_publisher(String, '/mock/control', 10)
        
        # Subscribers
        self.create_subscription(String, '/fault_trigger', self.fault_callback, 10)
        
        self.fault_detected = False
        self.start_time = time.time()
        self.test_phase = "INIT" # INIT, RUNNING, FAULT_INJECTED, PASSED, FAILED

        self.create_timer(1.0, self.run_test)
        self.get_logger().info("Test Runner Started. Waiting 5s before fault injection...")

    def fault_callback(self, msg):
        fault_data = json.loads(msg.data)
        self.get_logger().info(f"SUCCESS: Fault detected by system: {fault_data['type']}")
        self.fault_detected = True

    def run_test(self):
        elapsed = time.time() - self.start_time
        
        if self.test_phase == "INIT":
            if elapsed > 5.0:
                self.test_phase = "FAULT_INJECTED"
                self.inject_fault()
                self.fault_injection_time = time.time()

        elif self.test_phase == "FAULT_INJECTED":
            # Check if fault was detected within 2 seconds
            if self.fault_detected:
                self.get_logger().info("TEST PASSED: System detected fault correctly.")
                self.test_phase = "PASSED"
                # Exit with success
                # In a real runner, we'd signal success to a parent process
            elif time.time() - self.fault_injection_time > 3.0:
                self.get_logger().error("TEST FAILED: System did NOT detect fault within 3 seconds.")
                self.test_phase = "FAILED"

    def inject_fault(self):
        self.get_logger().info("Injecting LiDAR Failure...")
        msg = String()
        msg.data = json.dumps({"fail_lidar": True})
        self.control_pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = TestScenarioRunner()
    try:
        # Spin for a fixed time or until pass/fail
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.1)
            if node.test_phase in ["PASSED", "FAILED"]:
                break
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
