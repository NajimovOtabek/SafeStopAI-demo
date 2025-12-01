import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
import json
import math

class EmergencyPlanner(Node):
    def __init__(self):
        super().__init__('emergency_planner')
        
        # Parameters
        # Safe zones: list of [x, y, yaw]
        # For demo, we assume map frame coordinates
        self.safe_zones = [
            [2.0, 0.0, 0.0],  # Shoulder 1
            [5.0, 2.0, 1.57], # Parking Bay
            [-1.0, -1.0, 3.14] # Emergency Lay-by
        ]

        # Subscriber to fault trigger
        self.create_subscription(String, '/fault_trigger', self.fault_callback, 10)

        # Nav2 Action Client
        self._action_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        
        # Current Pose Subscriber (to find nearest safe zone)
        # In a real system, we'd use TF buffer, but for simplicity subscribing to /odom or /amcl_pose
        # Assuming /odom is available and close enough to map frame for this demo, 
        # OR we just use the last known pose if available. 
        # Let's use a placeholder for current pose.
        self.current_pose = None
        self.create_subscription(PoseStamped, '/goal_pose', self.pose_callback, 10) # Using goal_pose as a mock for now or amcl_pose

        self.get_logger().info('Emergency Planner Node Started')

    def pose_callback(self, msg):
        # In reality, use TF to get base_link in map frame
        self.current_pose = msg

    def fault_callback(self, msg):
        fault_data = json.loads(msg.data)
        self.get_logger().warn(f"RECEIVED FAULT: {fault_data['type']}. Initiating Emergency Stop!")
        
        # 1. Cancel current goals (if any) - handled by sending a new goal usually, or explicit cancel
        # self.cancel_current_goal() 

        # 2. Find nearest safe zone
        target_pose = self.find_nearest_safe_zone()

        # 3. Send new goal
        if target_pose:
            self.send_emergency_goal(target_pose)
        else:
            self.get_logger().error("No safe zone found! Executing in-lane stop (not implemented yet).")

    def find_nearest_safe_zone(self):
        if not self.safe_zones:
            return None
        
        if self.current_pose is None:
            self.get_logger().warn("Current pose unknown, defaulting to first safe zone.")
            best_zone = self.safe_zones[0]
        else:
            # Find nearest zone
            curr_x = self.current_pose.pose.position.x
            curr_y = self.current_pose.pose.position.y
            
            min_dist = float('inf')
            best_zone = self.safe_zones[0]
            
            for zone in self.safe_zones:
                dist = math.sqrt((zone[0] - curr_x)**2 + (zone[1] - curr_y)**2)
                if dist < min_dist:
                    min_dist = dist
                    best_zone = zone
            
            self.get_logger().info(f"Nearest safe zone found at {min_dist:.2f}m")

        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = best_zone[0]
        pose.pose.position.y = best_zone[1]
        
        # Convert yaw to quaternion (simplified)
        yaw = best_zone[2]
        pose.pose.orientation.z = math.sin(yaw / 2.0)
        pose.pose.orientation.w = math.cos(yaw / 2.0)
        
        return pose

    def send_emergency_goal(self, pose):
        self._action_client.wait_for_server()
        
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = pose
        goal_msg.behavior_tree = '' # Use default behavior tree

        self.get_logger().info(f"Sending emergency goal to x={pose.pose.position.x}, y={pose.pose.position.y}")
        
        self._send_goal_future = self._action_client.send_goal_async(goal_msg)
        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('Emergency goal rejected :(')
            return

        self.get_logger().info('Emergency goal accepted.')
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        result = future.result().result
        self.get_logger().info('Emergency Stop Maneuver Completed.')

def main(args=None):
    rclpy.init(args=args)
    node = EmergencyPlanner()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
