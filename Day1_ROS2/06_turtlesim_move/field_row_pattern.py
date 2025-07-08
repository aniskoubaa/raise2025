#!/usr/bin/env python3
"""
Simple Field Row Pattern - RAISE 2025
Educational example demonstrating crop row following patterns with turtlesim.

This pattern simulates a robot moving along crop rows in a field.
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
import math
import time


class FieldRowPattern(Node):
    """
    Simple turtlesim controller for field row following pattern.
    Educational example focusing on agricultural movement patterns.
    """

    def __init__(self):
        super().__init__('field_row_pattern')
        
        # Create publisher for turtle velocity commands
        self.velocity_pub = self.create_publisher(
            Twist,
            '/turtle1/cmd_vel',
            10
        )
        
        # Subscribe to turtle position
        self.pose_sub = self.create_subscription(
            Pose,
            '/turtle1/pose',
            self.pose_callback,
            10
        )
        
        # Current turtle position
        self.current_pose = None
        
        # Movement parameters
        self.linear_speed = 2.0      # Forward speed
        self.angular_speed = 1.5     # Rotation speed
        
        # Field parameters
        self.row_length = 6.0        # Length of each crop row
        self.row_spacing = 1.2       # Distance between rows
        self.num_rows = 4            # Number of rows to follow
        
        self.get_logger().info("🚜 Field Row Pattern Controller started")
        self.get_logger().info(f"📏 Field: {self.num_rows} rows, {self.row_length}m long, {self.row_spacing}m apart")
        self.get_logger().info("⏳ Waiting for turtle position...")

    def pose_callback(self, msg):
        """Handle turtle position updates."""
        self.current_pose = msg

    def wait_for_pose(self):
        """Wait until we receive the turtle's position."""
        while self.current_pose is None and rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.1)

    def stop_turtle(self):
        """Stop the turtle by sending zero velocity."""
        vel_msg = Twist()
        vel_msg.linear.x = 0.0
        vel_msg.angular.z = 0.0
        self.velocity_pub.publish(vel_msg)

    def move_forward(self, distance):
        """Move turtle forward by specified distance."""
        if self.current_pose is None:
            self.get_logger().error("No pose data available")
            return
        
        start_x = self.current_pose.x
        start_y = self.current_pose.y
        
        vel_msg = Twist()
        vel_msg.linear.x = self.linear_speed
        vel_msg.angular.z = 0.0
        
        self.get_logger().info(f"🚜 Moving forward {distance:.1f}m")
        
        # Move until we've traveled the desired distance
        while rclpy.ok():
            # Calculate distance traveled
            distance_traveled = math.sqrt(
                (self.current_pose.x - start_x)**2 + 
                (self.current_pose.y - start_y)**2
            )
            
            if distance_traveled >= distance:
                break
            
            # Continue moving forward
            self.velocity_pub.publish(vel_msg)
            rclpy.spin_once(self, timeout_sec=0.1)
        
        # Stop moving
        self.stop_turtle()
        self.get_logger().info(f"✅ Moved {distance_traveled:.1f}m")

    def rotate(self, angle_radians):
        """Rotate turtle by specified angle in radians."""
        if self.current_pose is None:
            self.get_logger().error("No pose data available")
            return
        
        start_angle = self.current_pose.theta
        target_angle = start_angle + angle_radians
        
        # Normalize target angle to [-π, π]
        while target_angle > math.pi:
            target_angle -= 2 * math.pi
        while target_angle < -math.pi:
            target_angle += 2 * math.pi
        
        vel_msg = Twist()
        vel_msg.linear.x = 0.0
        vel_msg.angular.z = self.angular_speed if angle_radians > 0 else -self.angular_speed
        
        angle_degrees = math.degrees(abs(angle_radians))
        self.get_logger().info(f"🔄 Rotating {angle_degrees:.1f}°")
        
        # Rotate until we reach the target angle
        while rclpy.ok():
            current_angle = self.current_pose.theta
            
            # Calculate remaining angle
            angle_diff = target_angle - current_angle
            
            # Normalize angle difference
            while angle_diff > math.pi:
                angle_diff -= 2 * math.pi
            while angle_diff < -math.pi:
                angle_diff += 2 * math.pi
            
            # Check if we're close enough to target
            if abs(angle_diff) < 0.1:  # Within 0.1 radians
                break
            
            # Continue rotating
            self.velocity_pub.publish(vel_msg)
            rclpy.spin_once(self, timeout_sec=0.1)
        
        # Stop rotating
        self.stop_turtle()
        self.get_logger().info("✅ Rotation complete")

    def follow_field_rows(self):
        """Execute the field row following pattern."""
        self.get_logger().info("🌾 Starting field row following pattern...")
        
        for row in range(self.num_rows):
            self.get_logger().info(f"📍 Row {row + 1}/{self.num_rows}")
            
            # Move along the current row
            self.move_forward(self.row_length)
            
            # If not the last row, move to the next row
            if row < self.num_rows - 1:
                self.get_logger().info("🔄 Moving to next row...")
                
                # Turn left (90 degrees)
                self.rotate(math.pi / 2)
                
                # Move to next row
                self.move_forward(self.row_spacing)
                
                # Turn left again to face along the row
                self.rotate(math.pi / 2)
                
                # Small pause between rows
                time.sleep(1.0)
        
        self.get_logger().info("✅ Field row pattern completed!")
        self.get_logger().info("🎯 All crop rows have been traversed")

    def show_pattern_info(self):
        """Display information about the field pattern."""
        self.get_logger().info("📋 FIELD ROW PATTERN INFO:")
        self.get_logger().info(f"   Number of rows: {self.num_rows}")
        self.get_logger().info(f"   Row length: {self.row_length}m")
        self.get_logger().info(f"   Row spacing: {self.row_spacing}m")
        self.get_logger().info(f"   Forward speed: {self.linear_speed} units/sec")
        self.get_logger().info(f"   Rotation speed: {self.angular_speed} rad/sec")
        self.get_logger().info("")
        self.get_logger().info("🌾 AGRICULTURAL APPLICATION:")
        self.get_logger().info("   - Harvesting crops planted in rows")
        self.get_logger().info("   - Systematic crop monitoring")
        self.get_logger().info("   - Precision spraying along crop lines")
        self.get_logger().info("   - Seeding operations")


def main(args=None):
    """Main function to run the field row pattern."""
    rclpy.init(args=args)
    
    # Create the pattern controller
    pattern_controller = FieldRowPattern()
    
    try:
        # Wait for turtle position
        pattern_controller.wait_for_pose()
        
        if pattern_controller.current_pose is None:
            pattern_controller.get_logger().error("❌ Could not get turtle position")
            return
        
        # Show pattern information
        pattern_controller.show_pattern_info()
        
        # Execute the pattern
        pattern_controller.follow_field_rows()
        
        # Keep node alive for a moment to see final position
        time.sleep(2.0)
        
    except KeyboardInterrupt:
        pattern_controller.get_logger().info("🛑 Field row pattern stopped by user")
        pattern_controller.stop_turtle()
    finally:
        pattern_controller.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main() 