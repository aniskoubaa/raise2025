#!/usr/bin/env python3
"""
Simple Soil Analysis Service - RAISE 2025
Educational example demonstrating ROS2 service implementation.

This service provides basic soil analysis for agricultural zones.
"""

import rclpy
from rclpy.node import Node
from example_interfaces.srv import SetBool
from std_msgs.msg import String
import random


class SoilAnalysisService(Node):
    """
    Simple service server for soil analysis.
    Educational example with basic functionality.
    """
    
    def __init__(self):
        super().__init__('soil_analysis_service')
        
        # Create the soil analysis service
        self.analysis_srv = self.create_service(
            SetBool,
            '/soil_analysis',
            self.handle_analysis_request
        )
        
        # Publisher for analysis results
        self.results_pub = self.create_publisher(
            String,
            '/soil_analysis_results',
            10
        )
        
        # Available zones
        self.available_zones = ['zone_1', 'zone_2', 'zone_3', 'zone_4']
        
        # Zone types (for educational context)
        self.zone_types = {
            'zone_1': 'Tomato Field',
            'zone_2': 'Wheat Field',
            'zone_3': 'Herb Garden',
            'zone_4': 'Corn Field'
        }
        
        self.get_logger().info("🔬 Simple Soil Analysis Service started")
        self.get_logger().info(f"🌱 Available zones: {self.available_zones}")
        self.get_logger().info("📡 Service ready at: /soil_analysis")
    
    def handle_analysis_request(self, request, response):
        """Handle soil analysis requests."""
        try:
            # Get zone ID from request
            zone_id = request.data
            
            # Check if zone exists
            if zone_id not in self.available_zones:
                response.success = False
                response.message = f"❌ Unknown zone: {zone_id}. Available: {self.available_zones}"
                return response
            
            # Simulate soil analysis
            self.get_logger().info(f"🔬 Analyzing soil for {zone_id} ({self.zone_types[zone_id]})")
            
            # Generate random but realistic soil data
            soil_data = self.generate_soil_data(zone_id)
            
            # Create response
            response.success = True
            response.message = f"✅ Soil analysis completed for {zone_id}"
            
            # Publish results
            self.publish_results(zone_id, soil_data)
            
            return response
            
        except Exception as e:
            self.get_logger().error(f"❌ Error: {str(e)}")
            response.success = False
            response.message = f"Error: {str(e)}"
            return response
    
    def generate_soil_data(self, zone_id):
        """Generate realistic soil data for educational purposes."""
        # Simulate different soil conditions for different zones
        if zone_id == 'zone_1':  # Tomato field
            ph = round(random.uniform(6.0, 6.8), 2)
            moisture = round(random.uniform(60, 80), 1)
            nutrients = "Good"
        elif zone_id == 'zone_2':  # Wheat field
            ph = round(random.uniform(6.5, 7.5), 2)
            moisture = round(random.uniform(50, 70), 1)
            nutrients = "Moderate"
        elif zone_id == 'zone_3':  # Herb garden
            ph = round(random.uniform(6.0, 7.0), 2)
            moisture = round(random.uniform(55, 75), 1)
            nutrients = "Good"
        else:  # Corn field
            ph = round(random.uniform(6.0, 6.8), 2)
            moisture = round(random.uniform(65, 85), 1)
            nutrients = "High"
        
        return {
            'zone_id': zone_id,
            'zone_type': self.zone_types[zone_id],
            'ph': ph,
            'moisture_percent': moisture,
            'nutrients': nutrients,
            'temperature': round(random.uniform(18, 25), 1)
        }
    
    def publish_results(self, zone_id, soil_data):
        """Publish soil analysis results."""
        # Create simple result message
        result_text = (
            f"ZONE: {soil_data['zone_id']} ({soil_data['zone_type']}) | "
            f"pH: {soil_data['ph']} | "
            f"Moisture: {soil_data['moisture_percent']}% | "
            f"Nutrients: {soil_data['nutrients']} | "
            f"Temperature: {soil_data['temperature']}°C"
        )
        
        results_msg = String()
        results_msg.data = result_text
        self.results_pub.publish(results_msg)
        
        self.get_logger().info(f"📊 Published results for {zone_id}: {result_text}")


def main(args=None):
    """Main function to run the soil analysis service."""
    rclpy.init(args=args)
    
    # Create and run the service
    soil_service = SoilAnalysisService()
    
    try:
        soil_service.get_logger().info("🔬 Starting soil analysis service...")
        rclpy.spin(soil_service)
    except KeyboardInterrupt:
        soil_service.get_logger().info("🛑 Service stopped by user")
    finally:
        soil_service.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main() 