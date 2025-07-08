#!/usr/bin/env python3
"""
Simple Field Navigator Action Server - RAISE 2025
Educational example demonstrating ROS2 actions.

This server simulates a robot navigating through agricultural fields.
"""

import rclpy
from rclpy.action import ActionServer
from rclpy.node import Node
from example_interfaces.action import Fibonacci  # We'll use this as placeholder
from std_msgs.msg import String
import time
import math


class FieldNavigatorServer(Node):
    """
    Simple action server for field navigation.
    Educational example focusing on action concepts.
    """

    def __init__(self):
        super().__init__('field_navigator_server')
        
        # Create action server (using Fibonacci as placeholder)
        self._action_server = ActionServer(
            self,
            Fibonacci,
            '/field_navigate',
            self.execute_navigation
        )
        
        # Publisher for robot status
        self.status_pub = self.create_publisher(
            String,
            '/robot_status',
            10
        )
        
        # Robot position simulation
        self.current_x = 0.0
        self.current_y = 0.0
        self.is_moving = False
        
        self.get_logger().info("🤖 Field Navigator Server started")
        self.get_logger().info("📡 Ready to receive navigation goals at: /field_navigate")

    def execute_navigation(self, goal_handle):
        """Execute the navigation goal."""
        self.get_logger().info("🎯 Received navigation goal")
        
        # For educational purposes, we'll use the Fibonacci order as distance
        goal = goal_handle.request
        target_distance = goal.order  # Using order as target distance
        
        # Validate goal
        if target_distance <= 0:
            goal_handle.abort()
            result = Fibonacci.Result()
            result.sequence = []
            self.get_logger().error("❌ Invalid target distance")
            return result
        
        # Start navigation
        self.is_moving = True
        self.get_logger().info(f"🚜 Starting navigation to distance: {target_distance}m")
        
        # Publish status
        status_msg = String()
        status_msg.data = f"NAVIGATING:target_distance={target_distance}"
        self.status_pub.publish(status_msg)
        
        # Simulate movement with feedback
        feedback_msg = Fibonacci.Feedback()
        result = Fibonacci.Result()
        
        current_distance = 0.0
        step = 1.0  # Move 1 meter per step
        
        sequence = []
        
        for i in range(target_distance):
            # Check if goal was cancelled
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                result.sequence = sequence
                self.is_moving = False
                self.get_logger().info("🛑 Navigation cancelled")
                
                # Publish cancelled status
                status_msg.data = f"CANCELLED:distance_traveled={current_distance}"
                self.status_pub.publish(status_msg)
                return result
            
            # Simulate movement (simple sequence for demo)
            if i < 2:
                next_val = i
            else:
                next_val = sequence[i-1] + sequence[i-2]
            
            sequence.append(next_val)
            current_distance = i + 1
            
            # Update robot position simulation
            self.current_x = current_distance * 0.5  # Simple movement pattern
            self.current_y = current_distance * 0.3
            
            # Send feedback
            feedback_msg.partial_sequence = sequence
            goal_handle.publish_feedback(feedback_msg)
            
            # Log progress
            distance_remaining = target_distance - current_distance
            self.get_logger().info(
                f"📍 Position: ({self.current_x:.1f}, {self.current_y:.1f}) | "
                f"Distance: {current_distance}/{target_distance}m | "
                f"Remaining: {distance_remaining}m"
            )
            
            # Simulate time for movement
            time.sleep(0.5)  # Move every 0.5 seconds for demo
        
        # Navigation completed successfully
        goal_handle.succeed()
        result.sequence = sequence
        self.is_moving = False
        
        self.get_logger().info(f"✅ Navigation completed! Reached distance: {target_distance}m")
        
        # Publish completion status
        status_msg.data = f"COMPLETED:final_distance={target_distance}:position=({self.current_x:.1f},{self.current_y:.1f})"
        self.status_pub.publish(status_msg)
        
        return result

    def get_robot_status(self):
        """Get current robot status."""
        status = {
            'position': {'x': self.current_x, 'y': self.current_y},
            'is_moving': self.is_moving
        }
        return status


def main(args=None):
    """Main function to run the field navigator server."""
    rclpy.init(args=args)
    
    # Create and run the action server
    navigator_server = FieldNavigatorServer()
    
    try:
        navigator_server.get_logger().info("🤖 Starting field navigation server...")
        rclpy.spin(navigator_server)
    except KeyboardInterrupt:
        navigator_server.get_logger().info("🛑 Field navigator stopped by user")
    finally:
        navigator_server.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main() 