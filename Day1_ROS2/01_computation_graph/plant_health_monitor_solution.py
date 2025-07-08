#!/usr/bin/env python3
"""
RAISE 2025 - Plant Health Monitor Node Solution
This is the complete solution for the plant health monitoring exercise.

This file is for instructor reference only.
Students should complete the template file themselves.

Author: RAISE 2025 Team
Date: July 2025
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, String

class PlantHealthMonitor(Node):
    def __init__(self):
        super().__init__('plant_health_monitor')
        
        # Create a subscriber to /soil_moisture
        self.subscription = self.create_subscription(
            Float32,                    # Message type
            '/soil_moisture',           # Topic name
            self.soil_moisture_callback, # Callback function
            10                          # Queue size
        )
        
        # Create a publisher for /plant_health_status
        self.publisher_ = self.create_publisher(
            String,                     # Message type
            '/plant_health_status',     # Topic name
            10                          # Queue size
        )
        
        # Initialize health monitoring parameters
        self.dry_threshold = 30.0      # Below this = too dry
        self.wet_threshold = 70.0      # Above this = too wet
        self.last_reading = 0.0        # Track last moisture reading
        self.reading_count = 0         # Count readings processed
        
        self.get_logger().info('🌿 Plant health monitor started!')
        self.get_logger().info(f'📊 Dry threshold: {self.dry_threshold}%')
        self.get_logger().info(f'📊 Wet threshold: {self.wet_threshold}%')
        self.get_logger().info('🔄 Waiting for soil moisture data...')

    def soil_moisture_callback(self, msg):
        """Process soil moisture data and determine plant health"""
        moisture = msg.data
        self.last_reading = moisture
        self.reading_count += 1
        
        # Implement health logic
        if moisture < self.dry_threshold:
            health_status = "Needs Water"
            emoji = "🔴"
            action = "IRRIGATE"
        elif moisture > self.wet_threshold:
            health_status = "Too Wet"
            emoji = "🔵"
            action = "DRAIN"
        else:
            health_status = "Healthy"
            emoji = "🟢"
            action = "MONITOR"
        
        # Create a String message and publish health status
        health_msg = String()
        health_msg.data = health_status
        self.publisher_.publish(health_msg)
        
        # Log the analysis with additional context
        self.get_logger().info(
            f'{emoji} Reading #{self.reading_count}: {moisture:.1f}% → '
            f'Health: {health_status} → Action: {action}'
        )

def main(args=None):
    rclpy.init(args=args)
    node = PlantHealthMonitor()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('🛑 Shutting down plant health monitor...')
    finally:
        node.destroy_node()
        rclpy.shutdown()
        print("✅ Plant health monitor shut down successfully!")

if __name__ == '__main__':
    main() 