#!/usr/bin/env python3
"""
RAISE 2025 - Agricultural Soil Sensor Node
This node simulates a soil moisture sensor for smart farming.

Author: RAISE 2025 Team
Date: July 2025
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
import random
import time

class SoilSensorNode(Node):
    def __init__(self):
        # Initialize the node with a name
        super().__init__('soil_sensor_node')
        
        # Create a publisher for soil moisture data
        self.publisher_ = self.create_publisher(
            Float32,                    # Message type
            '/soil_moisture',           # Topic name
            10                          # Queue size
        )
        
        # Create a timer to publish data every 2 seconds
        timer_period = 2.0  # seconds
        self.timer = self.create_timer(timer_period, self.publish_soil_moisture)
        
        # Initialize sensor parameters
        self.sensor_location = "Greenhouse_A_Row_1"
        self.sensor_id = "SM001"
        self.min_moisture = 20.0  # Minimum healthy moisture level
        self.max_moisture = 80.0  # Maximum healthy moisture level
        
        # Status tracking
        self.reading_count = 0
        
        self.get_logger().info(f'🌱 Soil sensor node started!')
        self.get_logger().info(f'📍 Location: {self.sensor_location}')
        self.get_logger().info(f'🆔 Sensor ID: {self.sensor_id}')
        self.get_logger().info(f'⏰ Publishing every {timer_period} seconds')

    def publish_soil_moisture(self):
        """Publish soil moisture reading (simulated)"""
        # Simulate soil moisture reading with some realistic variation
        # In a real sensor, this would read from hardware
        base_moisture = 45.0  # Base moisture level
        variation = random.uniform(-25.0, 35.0)  # Random variation
        moisture_level = max(0.0, min(100.0, base_moisture + variation))
        
        # Create message
        msg = Float32()
        msg.data = moisture_level
        
        # Publish the message
        self.publisher_.publish(msg)
        
        # Increment reading count
        self.reading_count += 1
        
        # Log the reading with helpful context
        status = self._get_moisture_status(moisture_level)
        self.get_logger().info(
            f'📊 Reading #{self.reading_count}: {moisture_level:.1f}% - {status}'
        )

    def _get_moisture_status(self, moisture_level):
        """Determine moisture status for logging"""
        if moisture_level < self.min_moisture:
            return "🔴 TOO DRY"
        elif moisture_level > self.max_moisture:
            return "🔵 TOO WET"
        else:
            return "🟢 HEALTHY"

def main(args=None):
    # Initialize ROS2
    rclpy.init(args=args)
    
    # Create and run the node
    node = SoilSensorNode()
    
    try:
        # Keep the node running
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('🛑 Shutting down soil sensor node...')
    finally:
        # Clean shutdown
        node.destroy_node()
        rclpy.shutdown()
        print("✅ Soil sensor node shut down successfully!")

if __name__ == '__main__':
    main() 