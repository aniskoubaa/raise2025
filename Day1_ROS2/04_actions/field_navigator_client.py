#!/usr/bin/env python3
"""
Simple Field Navigator Action Client - RAISE 2025
Educational example demonstrating ROS2 action clients.

This client sends navigation goals to the field navigator server.
"""

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from example_interfaces.action import Fibonacci
from std_msgs.msg import String
import sys


class FieldNavigatorClient(Node):
    """
    Simple action client for field navigation.
    Educational example focusing on action client concepts.
    """

    def __init__(self):
        super().__init__('field_navigator_client')
        
        # Create action client
        self._action_client = ActionClient(
            self,
            Fibonacci,
            '/field_navigate'
        )
        
        # Subscribe to robot status
        self.status_sub = self.create_subscription(
            String,
            '/robot_status',
            self.status_callback,
            10
        )
        
        self.get_logger().info("🎮 Field Navigator Client started")
        
        # Wait for action server
        self.wait_for_server()

    def wait_for_server(self):
        """Wait for the action server to be available."""
        self.get_logger().info("⏳ Waiting for field navigator server...")
        while not self._action_client.wait_for_server(timeout_sec=1.0):
            self.get_logger().info("⏳ Still waiting for navigator server...")
        self.get_logger().info("✅ Connected to field navigator!")

    def send_goal(self, distance):
        """Send a navigation goal to the server."""
        # Create goal
        goal_msg = Fibonacci.Goal()
        goal_msg.order = distance  # Using order as distance
        
        self.get_logger().info(f"🎯 Sending goal: Navigate {distance} meters")
        
        # Send goal with feedback callback
        self._send_goal_future = self._action_client.send_goal_async(
            goal_msg,
            feedback_callback=self.feedback_callback
        )
        
        # Add callback for when goal is accepted/rejected
        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        """Handle the server's response to our goal."""
        goal_handle = future.result()
        
        if not goal_handle.accepted:
            self.get_logger().error("❌ Goal rejected by server")
            return
        
        self.get_logger().info("✅ Goal accepted by server")
        
        # Get the result
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def feedback_callback(self, feedback_msg):
        """Handle feedback from the action server."""
        feedback = feedback_msg.feedback
        sequence_length = len(feedback.partial_sequence)
        
        if sequence_length > 0:
            self.get_logger().info(
                f"📊 Progress: Step {sequence_length} | "
                f"Latest value: {feedback.partial_sequence[-1]}"
            )

    def get_result_callback(self, future):
        """Handle the final result from the action server."""
        result = future.result().result
        status = future.result().status
        
        if status == 4:  # SUCCEEDED
            self.get_logger().info(
                f"✅ Navigation completed! "
                f"Total steps: {len(result.sequence)}"
            )
            if result.sequence:
                self.get_logger().info(f"📈 Final sequence: {result.sequence}")
        elif status == 5:  # CANCELED
            self.get_logger().info("🛑 Navigation was cancelled")
        else:
            self.get_logger().error(f"❌ Navigation failed with status: {status}")

    def cancel_goal(self):
        """Cancel the current goal."""
        if hasattr(self, '_send_goal_future'):
            self.get_logger().info("🛑 Attempting to cancel goal...")
            # This is a simplified cancellation for educational purposes
            self.get_logger().info("💡 In a real implementation, you would cancel through the goal handle")

    def status_callback(self, msg):
        """Handle robot status updates."""
        self.get_logger().info(f"🤖 Robot Status: {msg.data}")

    def send_navigation_sequence(self):
        """Send a sequence of navigation goals for demonstration."""
        distances = [3, 5, 2, 4]  # Different distances to navigate
        
        self.get_logger().info("🚜 Starting navigation sequence demo...")
        
        for i, distance in enumerate(distances):
            self.get_logger().info(f"--- Step {i+1}: Navigate {distance}m ---")
            self.send_goal(distance)
            
            # Wait for completion (simple approach for demo)
            import time
            time.sleep(distance + 2)  # Wait for navigation to complete

    def interactive_mode(self):
        """Run in interactive mode for manual testing."""
        self.get_logger().info("🎮 Interactive mode - Commands:")
        self.get_logger().info("  Enter distance (e.g., 5) to navigate")
        self.get_logger().info("  Enter 'c' to cancel current goal")
        self.get_logger().info("  Enter 'q' to quit")
        
        while True:
            try:
                user_input = input("Enter command: ").strip()
                
                if user_input.lower() == 'q':
                    self.get_logger().info("👋 Exiting interactive mode")
                    break
                elif user_input.lower() == 'c':
                    self.cancel_goal()
                else:
                    try:
                        distance = int(user_input)
                        if distance > 0:
                            self.send_goal(distance)
                        else:
                            print("Please enter a positive distance")
                    except ValueError:
                        print("Please enter a valid number or 'c' to cancel, 'q' to quit")
                        
            except KeyboardInterrupt:
                self.get_logger().info("👋 Exiting interactive mode")
                break


def main():
    """Main function."""
    rclpy.init()
    
    client = FieldNavigatorClient()
    
    try:
        # Check command line arguments
        if len(sys.argv) > 1:
            if sys.argv[1] == "sequence":
                client.send_navigation_sequence()
            elif sys.argv[1] == "interactive":
                client.interactive_mode()
            elif sys.argv[1].isdigit():
                distance = int(sys.argv[1])
                client.send_goal(distance)
                # Spin for a while to see the result
                import time
                start_time = time.time()
                while time.time() - start_time < distance + 3:
                    rclpy.spin_once(client, timeout_sec=0.1)
            else:
                print("Usage:")
                print("  python3 field_navigator_client.py sequence")
                print("  python3 field_navigator_client.py interactive")
                print("  python3 field_navigator_client.py <distance>")
                print("  Example: python3 field_navigator_client.py 5")
        else:
            # Default: send a simple goal
            client.send_goal(3)
            
            # Spin for a while to see the result
            import time
            start_time = time.time()
            while time.time() - start_time < 6:
                rclpy.spin_once(client, timeout_sec=0.1)
            
    except KeyboardInterrupt:
        client.get_logger().info("🛑 Client stopped by user")
    finally:
        client.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main() 