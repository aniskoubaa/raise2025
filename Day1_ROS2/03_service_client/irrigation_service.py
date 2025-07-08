#!/usr/bin/env python3
"""
Simple Irrigation Service - RAISE 2025
Educational example demonstrating ROS2 service/client architecture.

This service controls irrigation zones in a farm with basic start/stop functionality.
"""

import rclpy
from rclpy.node import Node
from example_interfaces.srv import SetBool
from std_msgs.msg import String


class IrrigationService(Node):
    """
    Simple service server for controlling irrigation zones.
    Demonstrates basic service concepts without complex logic.
    """
    
    def __init__(self):
        super().__init__('irrigation_service')
        
        # Create the irrigation control service
        self.irrigation_srv = self.create_service(
            SetBool,
            '/irrigation_control',
            self.handle_irrigation_request
        )
        
        # Publisher for status updates
        self.status_pub = self.create_publisher(
            String,
            '/irrigation_status',
            10
        )
        
        # Simple state tracking
        self.active_zones = {}  # Dictionary to track active zones
        
        # Available zones
        self.available_zones = ['zone_1', 'zone_2', 'zone_3', 'zone_4']
        
        self.get_logger().info("🌱 Simple Irrigation Service started")
        self.get_logger().info(f"🚰 Available zones: {self.available_zones}")
        self.get_logger().info("📡 Service ready at: /irrigation_control")
    
    def handle_irrigation_request(self, request, response):
        """
        Handle irrigation control requests.
        Simple version for educational purposes.
        """
        try:
            # Get zone ID from request data
            zone_id = request.data
            
            # Check if zone exists
            if zone_id not in self.available_zones:
                response.success = False
                response.message = f"❌ Unknown zone: {zone_id}. Available: {self.available_zones}"
                return response
            
            # Handle start irrigation
            if request.enable:
                if zone_id in self.active_zones:
                    response.success = False
                    response.message = f"⚠️ Zone {zone_id} is already running"
                else:
                    # Start irrigation
                    self.active_zones[zone_id] = True
                    response.success = True
                    response.message = f"✅ Started irrigation for {zone_id}"
                    
                    # Publish status update
                    status_msg = String()
                    status_msg.data = f"STARTED:{zone_id}"
                    self.status_pub.publish(status_msg)
                    
                    self.get_logger().info(f"🚰 Started irrigation for {zone_id}")
            
            # Handle stop irrigation
            else:
                if zone_id not in self.active_zones:
                    response.success = False
                    response.message = f"⚠️ Zone {zone_id} is not running"
                else:
                    # Stop irrigation
                    del self.active_zones[zone_id]
                    response.success = True
                    response.message = f"🛑 Stopped irrigation for {zone_id}"
                    
                    # Publish status update
                    status_msg = String()
                    status_msg.data = f"STOPPED:{zone_id}"
                    self.status_pub.publish(status_msg)
                    
                    self.get_logger().info(f"🛑 Stopped irrigation for {zone_id}")
            
            return response
            
        except Exception as e:
            self.get_logger().error(f"❌ Error: {str(e)}")
            response.success = False
            response.message = f"Error: {str(e)}"
            return response
    
    def show_active_zones(self):
        """Display currently active irrigation zones."""
        if self.active_zones:
            active_list = list(self.active_zones.keys())
            self.get_logger().info(f"💧 Active zones: {active_list}")
        else:
            self.get_logger().info("💧 No zones currently active")


def main(args=None):
    """Main function to run the irrigation service."""
    rclpy.init(args=args)
    
    # Create and run the irrigation service
    irrigation_service = IrrigationService()
    
    try:
        irrigation_service.get_logger().info("🌾 Starting irrigation service...")
        rclpy.spin(irrigation_service)
    except KeyboardInterrupt:
        irrigation_service.get_logger().info("🛑 Irrigation service stopped by user")
    finally:
        # Cleanup
        irrigation_service.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main() 