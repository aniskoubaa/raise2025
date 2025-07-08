#!/usr/bin/env python3
"""
Simple Spiral Coverage Pattern - RAISE 2025
Educational example demonstrating spiral field coverage with turtlesim.

This pattern simulates efficient area coverage starting from center and spiraling outward.
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
import math
import time


class SpiralCoveragePattern(Node):
    """
    Simple turtlesim controller for spiral coverage pattern.
    Educational example focusing on efficient field coverage.
    """

    def __init__(self):
        super().__init__('spiral_coverage_pattern')
        
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
        self.base_speed = 1.5        # Base movement speed
        self.max_radius = 4.0        # Maximum spiral radius
        self.spiral_increment = 0.1  # How much to increase radius per step
        
        self.get_logger().info("🌀 Spiral Coverage Pattern Controller started")
        self.get_logger().info(f"📏 Coverage: radius up to {self.max_radius} units")
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

    def move_to_center(self):
        """Move turtle to center of the field (approximately)."""
        self.get_logger().info("📍 Moving to field center...")
        
        # Calculate center position (turtlesim is roughly 11x11 units)
        center_x = 5.5
        center_y = 5.5
        
        # Simple approach: move to center
        vel_msg = Twist()
        
        while rclpy.ok():
            if self.current_pose is None:
                rclpy.spin_once(self, timeout_sec=0.1)
                continue
            
            # Calculate distance to center
            dx = center_x - self.current_pose.x
            dy = center_y - self.current_pose.y
            distance = math.sqrt(dx**2 + dy**2)
            
            if distance < 0.2:  # Close enough to center
                break
            
            # Calculate angle to center
            angle_to_center = math.atan2(dy, dx)
            angle_diff = angle_to_center - self.current_pose.theta
            
            # Normalize angle difference
            while angle_diff > math.pi:
                angle_diff -= 2 * math.pi
            while angle_diff < -math.pi:
                angle_diff += 2 * math.pi
            
            # Move toward center
            vel_msg.linear.x = min(self.base_speed, distance)
            vel_msg.angular.z = 2.0 * angle_diff
            
            self.velocity_pub.publish(vel_msg)
            rclpy.spin_once(self, timeout_sec=0.1)
        
        self.stop_turtle()
        self.get_logger().info("✅ Reached field center")

    def execute_spiral_pattern(self):
        """Execute the spiral coverage pattern."""
        self.get_logger().info("🌀 Starting spiral coverage pattern...")
        
        # Start from center
        start_x = self.current_pose.x
        start_y = self.current_pose.y
        
        # Spiral parameters
        radius = 0.2           # Starting radius
        angle = 0.0           # Starting angle
        angle_increment = 0.2  # How much to increase angle per step
        
        step = 0
        
        while radius < self.max_radius and rclpy.ok():
            # Calculate target position on spiral
            target_x = start_x + radius * math.cos(angle)
            target_y = start_y + radius * math.sin(angle)
            
            # Make sure we stay within bounds
            if target_x < 0.5 or target_x > 10.5 or target_y < 0.5 or target_y > 10.5:
                self.get_logger().info("📏 Reached field boundary")
                break
            
            # Calculate movement needed
            dx = target_x - self.current_pose.x
            dy = target_y - self.current_pose.y
            distance = math.sqrt(dx**2 + dy**2)
            
            if distance > 0.1:  # Only move if we need to
                # Calculate angle to target
                angle_to_target = math.atan2(dy, dx)
                angle_diff = angle_to_target - self.current_pose.theta
                
                # Normalize angle difference
                while angle_diff > math.pi:
                    angle_diff -= 2 * math.pi
                while angle_diff < -math.pi:
                    angle_diff += 2 * math.pi
                
                # Move toward target
                vel_msg = Twist()
                vel_msg.linear.x = min(self.base_speed, distance * 3)
                vel_msg.angular.z = 3.0 * angle_diff
                
                self.velocity_pub.publish(vel_msg)
            
            # Update spiral parameters
            angle += angle_increment
            radius += self.spiral_increment
            step += 1
            
            # Log progress occasionally
            if step % 50 == 0:
                self.get_logger().info(f"🌀 Spiral progress: radius={radius:.1f}, step={step}")
            
            # Small delay for smooth movement
            rclpy.spin_once(self, timeout_sec=0.05)
        
        self.stop_turtle()
        self.get_logger().info("✅ Spiral coverage pattern completed!")
        self.get_logger().info(f"🎯 Covered area up to radius {radius:.1f} units")

    def show_pattern_info(self):
        """Display information about the spiral pattern."""
        self.get_logger().info("📋 SPIRAL COVERAGE PATTERN INFO:")
        self.get_logger().info(f"   Maximum radius: {self.max_radius} units")
        self.get_logger().info(f"   Base speed: {self.base_speed} units/sec")
        self.get_logger().info(f"   Spiral increment: {self.spiral_increment} units/step")
        self.get_logger().info("")
        self.get_logger().info("🌀 AGRICULTURAL APPLICATION:")
        self.get_logger().info("   - Efficient field coverage from center outward")
        self.get_logger().info("   - Minimizes travel time to start coverage")
        self.get_logger().info("   - Good for circular or square fields")
        self.get_logger().info("   - Useful for spreading seeds or fertilizer")
        self.get_logger().info("")
        self.get_logger().info("🎯 COVERAGE STRATEGY:")
        self.get_logger().info("   - Starts from field center")
        self.get_logger().info("   - Expands outward in spiral pattern")
        self.get_logger().info("   - Ensures complete area coverage")
        self.get_logger().info("   - Stops at field boundaries")

    def simple_spiral_demo(self):
        """Simple demonstration of spiral movement."""
        self.get_logger().info("🌀 Simple spiral demo (continuous movement)...")
        
        vel_msg = Twist()
        start_time = time.time()
        
        while rclpy.ok() and (time.time() - start_time) < 20.0:  # Run for 20 seconds
            # Create spiral by combining forward motion with rotation
            # The faster we go forward, the wider the spiral
            current_time = time.time() - start_time
            
            # Gradually increase forward speed (creates expanding spiral)
            vel_msg.linear.x = 0.5 + current_time * 0.1
            
            # Constant angular velocity (creates spiral)
            vel_msg.angular.z = 2.0
            
            # Stop if we get too close to boundaries
            if self.current_pose is not None:
                if (self.current_pose.x < 1.0 or self.current_pose.x > 10.0 or
                    self.current_pose.y < 1.0 or self.current_pose.y > 10.0):
                    self.get_logger().info("📏 Approaching boundary, stopping spiral")
                    break
            
            self.velocity_pub.publish(vel_msg)
            rclpy.spin_once(self, timeout_sec=0.1)
        
        self.stop_turtle()
        self.get_logger().info("✅ Simple spiral demo completed!")


def main(args=None):
    """Main function to run the spiral coverage pattern."""
    rclpy.init(args=args)
    
    # Create the pattern controller
    pattern_controller = SpiralCoveragePattern()
    
    try:
        # Wait for turtle position
        pattern_controller.wait_for_pose()
        
        if pattern_controller.current_pose is None:
            pattern_controller.get_logger().error("❌ Could not get turtle position")
            return
        
        # Show pattern information
        pattern_controller.show_pattern_info()
        
        # Move to center first
        pattern_controller.move_to_center()
        
        # Wait a moment
        time.sleep(2.0)
        
        # Execute spiral pattern
        pattern_controller.simple_spiral_demo()
        
        # Keep node alive for a moment
        time.sleep(2.0)
        
    except KeyboardInterrupt:
        pattern_controller.get_logger().info("🛑 Spiral pattern stopped by user")
        pattern_controller.stop_turtle()
    finally:
        pattern_controller.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main() 