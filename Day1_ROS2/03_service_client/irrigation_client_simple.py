#!/usr/bin/env python3
"""
Simple Irrigation Client - RAISE 2025
Educational example demonstrating ROS2 service client.

This client shows how to call services to control irrigation zones.
"""

import rclpy
from rclpy.node import Node
from example_interfaces.srv import SetBool
from std_msgs.msg import String
import sys


class IrrigationClient(Node):
    """
    Simple client for controlling irrigation zones.
    Educational example focusing on basic service calls.
    """
    
    def __init__(self):
        super().__init__('irrigation_client')
        
        # Create client for irrigation control service
        self.irrigation_client = self.create_client(SetBool, '/irrigation_control')
        
        # Subscribe to status updates
        self.status_sub = self.create_subscription(
            String,
            '/irrigation_status',
            self.status_callback,
            10
        )
        
        self.get_logger().info("🌱 Irrigation Client started")
        
        # Wait for service to be available
        self.wait_for_service()
    
    def wait_for_service(self):
        """Wait for the irrigation service to be available."""
        self.get_logger().info("⏳ Waiting for irrigation service...")
        while not self.irrigation_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("⏳ Still waiting for irrigation service...")
        self.get_logger().info("✅ Connected to irrigation service")
    
    def start_irrigation(self, zone_id):
        """Start irrigation for a specific zone."""
        request = SetBool.Request()
        request.data = zone_id
        request.enable = True
        
        self.get_logger().info(f"🚰 Requesting to START irrigation for {zone_id}")
        
        # Call the service
        future = self.irrigation_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        
        if future.result() is not None:
            response = future.result()
            if response.success:
                self.get_logger().info(f"✅ Success: {response.message}")
            else:
                self.get_logger().error(f"❌ Failed: {response.message}")
        else:
            self.get_logger().error("❌ Service call failed")
    
    def stop_irrigation(self, zone_id):
        """Stop irrigation for a specific zone."""
        request = SetBool.Request()
        request.data = zone_id
        request.enable = False
        
        self.get_logger().info(f"🛑 Requesting to STOP irrigation for {zone_id}")
        
        # Call the service
        future = self.irrigation_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        
        if future.result() is not None:
            response = future.result()
            if response.success:
                self.get_logger().info(f"✅ Success: {response.message}")
            else:
                self.get_logger().error(f"❌ Failed: {response.message}")
        else:
            self.get_logger().error("❌ Service call failed")
    
    def status_callback(self, msg):
        """Handle irrigation status updates."""
        self.get_logger().info(f"📊 Status update: {msg.data}")
    
    def run_demo(self):
        """Run a simple demonstration."""
        self.get_logger().info("🌾 Starting irrigation demo...")
        
        # Test zones
        test_zones = ['zone_1', 'zone_2', 'zone_3']
        
        for zone_id in test_zones:
            self.get_logger().info(f"--- Testing {zone_id} ---")
            
            # Start irrigation
            self.start_irrigation(zone_id)
            
            # Wait a bit
            import time
            time.sleep(2)
            
            # Stop irrigation
            self.stop_irrigation(zone_id)
            
            # Wait between tests
            time.sleep(1)
        
        self.get_logger().info("✅ Demo completed!")


def main():
    """Main function."""
    rclpy.init()
    
    client = IrrigationClient()
    
    try:
        # Check command line arguments
        if len(sys.argv) > 1:
            if sys.argv[1] == "demo":
                client.run_demo()
            elif sys.argv[1] == "start" and len(sys.argv) >= 3:
                zone_id = sys.argv[2]
                client.start_irrigation(zone_id)
            elif sys.argv[1] == "stop" and len(sys.argv) >= 3:
                zone_id = sys.argv[2]
                client.stop_irrigation(zone_id)
            else:
                print("Usage:")
                print("  python3 irrigation_client_simple.py demo")
                print("  python3 irrigation_client_simple.py start <zone_id>")
                print("  python3 irrigation_client_simple.py stop <zone_id>")
                print("  Available zones: zone_1, zone_2, zone_3, zone_4")
        else:
            # Default: run demo
            client.run_demo()
            
    except KeyboardInterrupt:
        client.get_logger().info("🛑 Client stopped by user")
    finally:
        client.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main() 