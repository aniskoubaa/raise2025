#!/usr/bin/env python3
"""
Simple Soil Sensor Publisher - RAISE 2025
Educational example demonstrating custom messages in ROS2.

This publisher simulates soil sensor data using custom message types.
Note: This is a simplified example using built-in messages since
custom messages require package compilation.
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import json
import time
import random


class SoilSensorPublisher(Node):
    """
    Simple publisher for soil sensor data.
    Educational example focusing on custom message concepts.
    """

    def __init__(self):
        super().__init__('soil_sensor_publisher')
        
        # Create publisher for soil sensor data
        self.publisher = self.create_publisher(
            String,
            '/soil_data',
            10
        )
        
        # Timer to publish data every 5 seconds
        self.timer = self.create_timer(5.0, self.publish_soil_data)
        
        # Sensor locations
        self.sensor_locations = [
            "field_A_zone_1",
            "field_A_zone_2", 
            "field_B_zone_1",
            "field_B_zone_2"
        ]
        
        self.get_logger().info("🌱 Soil Sensor Publisher started")
        self.get_logger().info(f"📊 Publishing data for {len(self.sensor_locations)} sensors")
        self.get_logger().info("📡 Publishing to: /soil_data")

    def generate_soil_data(self, location):
        """Generate realistic soil sensor data."""
        # Simulate different soil conditions for different locations
        if "field_A" in location:
            # Field A has good conditions
            ph = round(random.uniform(6.2, 6.8), 2)
            moisture = round(random.uniform(65, 85), 1)
            temperature = round(random.uniform(20, 25), 1)
        else:
            # Field B needs more attention
            ph = round(random.uniform(5.8, 6.5), 2)
            moisture = round(random.uniform(45, 65), 1)
            temperature = round(random.uniform(18, 23), 1)
        
        return {
            "ph": ph,
            "moisture_percent": moisture,
            "temperature_celsius": temperature,
            "location": location,
            "timestamp": int(time.time())
        }

    def publish_soil_data(self):
        """Publish soil sensor data for all locations."""
        for location in self.sensor_locations:
            # Generate sensor data
            soil_data = self.generate_soil_data(location)
            
            # Create message (using String with JSON for simplicity)
            # In a real implementation, this would be a custom SoilSensor message
            msg = String()
            msg.data = json.dumps(soil_data)
            
            # Publish the message
            self.publisher.publish(msg)
            
            # Log the data
            self.get_logger().info(
                f"📊 {location}: pH={soil_data['ph']}, "
                f"Moisture={soil_data['moisture_percent']}%, "
                f"Temp={soil_data['temperature_celsius']}°C"
            )
    
    def show_message_structure(self):
        """Display the message structure for educational purposes."""
        self.get_logger().info("📋 Custom SoilSensor Message Structure:")
        self.get_logger().info("   float64 ph                    # pH level (0-14)")
        self.get_logger().info("   float64 moisture_percent      # Moisture percentage (0-100)")
        self.get_logger().info("   float64 temperature_celsius   # Temperature in Celsius")
        self.get_logger().info("   string location              # Location identifier")
        self.get_logger().info("   int64 timestamp              # Unix timestamp")


def main(args=None):
    """Main function to run the soil sensor publisher."""
    rclpy.init(args=args)
    
    # Create and run the publisher
    soil_publisher = SoilSensorPublisher()
    
    try:
        soil_publisher.show_message_structure()
        soil_publisher.get_logger().info("🌱 Starting soil sensor data publishing...")
        rclpy.spin(soil_publisher)
    except KeyboardInterrupt:
        soil_publisher.get_logger().info("🛑 Soil sensor publisher stopped by user")
    finally:
        soil_publisher.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main() 