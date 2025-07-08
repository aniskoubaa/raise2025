#!/usr/bin/env python3
"""
RAISE 2025 - Farm Sensor Network
Simulates multiple soil sensors across a farm publishing to different topics.

This demonstrates:
- Multiple publishers in one node
- Different message types (Float32, Temperature)
- Structured topic naming
- Different update rates for different sensor types

Author: RAISE 2025 Team
Date: July 2025
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, String
from sensor_msgs.msg import Temperature
import random
import threading

class FarmSensorNetwork(Node):
    def __init__(self):
        super().__init__('farm_sensor_network')
        
        # Farm configuration
        self.farm_zones = {
            'field_a': ['row_1', 'row_2', 'row_3'],
            'field_b': ['row_1', 'row_2'],
            'greenhouse': ['section_1', 'section_2', 'section_3']
        }
        
        # Create publishers for each sensor location
        self.soil_publishers = {}
        self.temp_publishers = {}
        
        for zone, rows in self.farm_zones.items():
            for row in rows:
                # Soil moisture publisher
                soil_topic = f'/sensors/{zone}/{row}/soil_moisture'
                self.soil_publishers[f'{zone}_{row}'] = self.create_publisher(
                    Float32, soil_topic, 10
                )
                
                # Temperature publisher
                temp_topic = f'/sensors/{zone}/{row}/temperature'
                self.temp_publishers[f'{zone}_{row}'] = self.create_publisher(
                    Temperature, temp_topic, 10
                )
        
        # Central farm status publisher
        self.status_publisher = self.create_publisher(
            String, '/farm_status', 10
        )
        
        # Timers for different update rates
        self.soil_timer = self.create_timer(3.0, self.publish_soil_data)
        self.temp_timer = self.create_timer(5.0, self.publish_temperature_data)
        self.status_timer = self.create_timer(10.0, self.publish_farm_status)
        
        self.reading_count = 0
        
        self.get_logger().info('🌾 Farm sensor network started!')
        self.get_logger().info(f'📊 Monitoring {len(self.soil_publishers)} sensor locations')
        self.get_logger().info('📡 Publishing sensor data on multiple topics:')
        for location in self.soil_publishers.keys():
            zone, row = location.split('_', 1)
            self.get_logger().info(f'   • /sensors/{zone}/{row}/soil_moisture')
            self.get_logger().info(f'   • /sensors/{zone}/{row}/temperature')

    def publish_soil_data(self):
        """Publish soil moisture data from all sensors"""
        for location, publisher in self.soil_publishers.items():
            # Simulate different moisture levels per zone
            zone = location.split('_')[0]
            base_moisture = {'field_a': 40, 'field_b': 55, 'greenhouse': 65}[zone]
            
            # Add some realistic variation
            moisture = max(0, min(100, base_moisture + random.uniform(-20, 20)))
            
            msg = Float32()
            msg.data = moisture
            publisher.publish(msg)
        
        self.reading_count += 1
        self.get_logger().info(f'📊 Soil data update #{self.reading_count} - {len(self.soil_publishers)} sensors')

    def publish_temperature_data(self):
        """Publish temperature data from all sensors"""
        for location, publisher in self.temp_publishers.items():
            # Simulate different temperatures per zone
            zone = location.split('_')[0]
            base_temp = {'field_a': 22, 'field_b': 24, 'greenhouse': 28}[zone]
            
            temperature = base_temp + random.uniform(-3, 3)
            
            msg = Temperature()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = f'sensor_{location}'
            msg.temperature = temperature
            msg.variance = 0.1  # Temperature sensor accuracy
            
            publisher.publish(msg)
        
        self.get_logger().info(f'🌡️ Temperature data published to {len(self.temp_publishers)} sensors')

    def publish_farm_status(self):
        """Publish overall farm status"""
        total_sensors = len(self.soil_publishers) + len(self.temp_publishers)
        
        status_msg = String()
        status_msg.data = f"Farm operational - {total_sensors} sensors active, {self.reading_count} readings completed"
        self.status_publisher.publish(status_msg)
        
        self.get_logger().info(f'📈 Farm status: {total_sensors} sensors active, {self.reading_count} soil readings')

def main(args=None):
    rclpy.init(args=args)
    node = FarmSensorNetwork()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('🛑 Shutting down farm sensor network...')
    finally:
        node.destroy_node()
        rclpy.shutdown()
        print("✅ Farm sensor network shut down successfully!")

if __name__ == '__main__':
    main() 