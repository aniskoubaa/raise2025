#!/usr/bin/env python3
"""
Simple Rectangle Perimeter Pattern - RAISE 2025
Educational example demonstrating field boundary inspection with turtlesim.

This pattern simulates a robot following the perimeter of a rectangular field.
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
import math
import time


class RectanglePerimeterPattern(Node):
    """
    Simple turtlesim controller for rectangle perimeter pattern.
    Educational example focusing on field boundary inspection.
    """

    def __init__(self):
        super().__init__('rectangle_perimeter_pattern')
        
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
        
        # Rectangle parameters (field boundaries)
        self.field_width = 7.0       # Field width
        self.field_height = 5.0      # Field height
        self.start_x = 2.0          # Bottom-left corner x
        self.start_y = 2.0          # Bottom-left corner y
        
        self.get_logger().info("📐 Rectangle Perimeter Pattern Controller started")
        self.get_logger().info(f"📏 Field: {self.field_width}x{self.field_height} units")
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

    def move_to_start_position(self):
        """Move turtle to the starting position (bottom-left corner)."""
        self.get_logger().info("📍 Moving to field corner...")
        
        vel_msg = Twist()
        
        while rclpy.ok():
            if self.current_pose is None:
                rclpy.spin_once(self, timeout_sec=0.1)
                continue
            
            # Calculate distance to start position
            dx = self.start_x - self.current_pose.x
            dy = self.start_y - self.current_pose.y
            distance = math.sqrt(dx**2 + dy**2)
            
            if distance < 0.2:  # Close enough
                break
            
            # Calculate angle to start position
            angle_to_start = math.atan2(dy, dx)
            angle_diff = angle_to_start - self.current_pose.theta
            
            # Normalize angle difference
            while angle_diff > math.pi:
                angle_diff -= 2 * math.pi
            while angle_diff < -math.pi:
                angle_diff += 2 * math.pi
            
            # Move toward start position
            vel_msg.linear.x = min(self.linear_speed, distance)
            vel_msg.angular.z = 2.0 * angle_diff
            
            self.velocity_pub.publish(vel_msg)
            rclpy.spin_once(self, timeout_sec=0.1)
        
        self.stop_turtle()
        self.get_logger().info("✅ Reached field corner")

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
        
        self.get_logger().info(f"➡️  Moving forward {distance:.1f} units")
        
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
        self.get_logger().info(f"✅ Moved {distance_traveled:.1f} units")

    def rotate_90_degrees(self, clockwise=False):
        """Rotate turtle by 90 degrees."""
        if self.current_pose is None:
            self.get_logger().error("No pose data available")
            return
        
        start_angle = self.current_pose.theta
        target_angle = start_angle + (math.pi / 2 if not clockwise else -math.pi / 2)
        
        # Normalize target angle
        while target_angle > math.pi:
            target_angle -= 2 * math.pi
        while target_angle < -math.pi:
            target_angle += 2 * math.pi
        
        vel_msg = Twist()
        vel_msg.linear.x = 0.0
        vel_msg.angular.z = self.angular_speed if not clockwise else -self.angular_speed
        
        direction = "left" if not clockwise else "right"
        self.get_logger().info(f"🔄 Turning {direction} (90°)")
        
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
            
            # Check if we're close enough
            if abs(angle_diff) < 0.1:
                break
            
            # Continue rotating
            self.velocity_pub.publish(vel_msg)
            rclpy.spin_once(self, timeout_sec=0.1)
        
        self.stop_turtle()
        self.get_logger().info("✅ Turn completed")

    def follow_rectangle_perimeter(self):
        """Execute the rectangle perimeter following pattern."""
        self.get_logger().info("📐 Starting rectangle perimeter inspection...")
        
        # Define the four sides of the rectangle
        sides = [
            ("Bottom side", self.field_width),
            ("Right side", self.field_height),
            ("Top side", self.field_width),
            ("Left side", self.field_height)
        ]
        
        for i, (side_name, distance) in enumerate(sides):
            self.get_logger().info(f"📍 {side_name} ({i+1}/4)")
            
            # Move along the current side
            self.move_forward(distance)
            
            # If not the last side, turn left for the next side
            if i < len(sides) - 1:
                time.sleep(0.5)  # Small pause
                self.rotate_90_degrees(clockwise=False)  # Turn left
                time.sleep(0.5)  # Small pause
        
        self.get_logger().info("✅ Rectangle perimeter inspection completed!")
        self.get_logger().info("🎯 Full field boundary has been traversed")

    def inspect_field_corners(self):
        """Demonstrate corner inspection behavior."""
        self.get_logger().info("🔍 Inspecting field corners...")
        
        # Move to each corner and pause briefly
        corners = [
            ("Bottom-left", self.start_x, self.start_y),
            ("Bottom-right", self.start_x + self.field_width, self.start_y),
            ("Top-right", self.start_x + self.field_width, self.start_y + self.field_height),
            ("Top-left", self.start_x, self.start_y + self.field_height)
        ]
        
        for corner_name, corner_x, corner_y in corners:
            self.get_logger().info(f"🔍 Moving to {corner_name} corner...")
            
            # Move to corner
            vel_msg = Twist()
            
            while rclpy.ok():
                if self.current_pose is None:
                    rclpy.spin_once(self, timeout_sec=0.1)
                    continue
                
                # Calculate distance to corner
                dx = corner_x - self.current_pose.x
                dy = corner_y - self.current_pose.y
                distance = math.sqrt(dx**2 + dy**2)
                
                if distance < 0.3:  # Close enough
                    break
                
                # Calculate angle to corner
                angle_to_corner = math.atan2(dy, dx)
                angle_diff = angle_to_corner - self.current_pose.theta
                
                # Normalize angle difference
                while angle_diff > math.pi:
                    angle_diff -= 2 * math.pi
                while angle_diff < -math.pi:
                    angle_diff += 2 * math.pi
                
                # Move toward corner
                vel_msg.linear.x = min(self.linear_speed, distance)
                vel_msg.angular.z = 2.0 * angle_diff
                
                self.velocity_pub.publish(vel_msg)
                rclpy.spin_once(self, timeout_sec=0.1)
            
            self.stop_turtle()
            self.get_logger().info(f"✅ Inspecting {corner_name} corner")
            time.sleep(2.0)  # Pause for inspection
        
        self.get_logger().info("✅ Corner inspection completed!")

    def show_pattern_info(self):
        """Display information about the rectangle pattern."""
        self.get_logger().info("📋 RECTANGLE PERIMETER PATTERN INFO:")
        self.get_logger().info(f"   Field width: {self.field_width} units")
        self.get_logger().info(f"   Field height: {self.field_height} units")
        self.get_logger().info(f"   Start position: ({self.start_x}, {self.start_y})")
        self.get_logger().info(f"   Forward speed: {self.linear_speed} units/sec")
        self.get_logger().info(f"   Rotation speed: {self.angular_speed} rad/sec")
        self.get_logger().info("")
        self.get_logger().info("📐 AGRICULTURAL APPLICATION:")
        self.get_logger().info("   - Field boundary inspection")
        self.get_logger().info("   - Fence line monitoring")
        self.get_logger().info("   - Perimeter security patrol")
        self.get_logger().info("   - Crop edge assessment")
        self.get_logger().info("")
        self.get_logger().info("🎯 INSPECTION STRATEGY:")
        self.get_logger().info("   - Systematic perimeter following")
        self.get_logger().info("   - Corner-to-corner navigation")
        self.get_logger().info("   - Consistent inspection coverage")
        self.get_logger().info("   - Return to starting position")


def main(args=None):
    """Main function to run the rectangle perimeter pattern."""
    rclpy.init(args=args)
    
    # Create the pattern controller
    pattern_controller = RectanglePerimeterPattern()
    
    try:
        # Wait for turtle position
        pattern_controller.wait_for_pose()
        
        if pattern_controller.current_pose is None:
            pattern_controller.get_logger().error("❌ Could not get turtle position")
            return
        
        # Show pattern information
        pattern_controller.show_pattern_info()
        
        # Move to starting position
        pattern_controller.move_to_start_position()
        
        # Wait a moment
        time.sleep(2.0)
        
        # Execute perimeter pattern
        pattern_controller.follow_rectangle_perimeter()
        
        # Keep node alive for a moment
        time.sleep(2.0)
        
    except KeyboardInterrupt:
        pattern_controller.get_logger().info("🛑 Rectangle perimeter pattern stopped by user")
        pattern_controller.stop_turtle()
    finally:
        pattern_controller.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main() 