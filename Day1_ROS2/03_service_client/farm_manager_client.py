#!/usr/bin/env python3
"""
Simple Farm Client - RAISE 2025
Educational example demonstrating how to call multiple services in ROS2.

This client shows how to coordinate irrigation and soil analysis services.
"""

import rclpy
from rclpy.node import Node
from example_interfaces.srv import SetBool
from std_msgs.msg import String
import time


class SimpleFarmClient(Node):
    """
    Simple client that demonstrates calling multiple services.
    Shows basic coordination between irrigation and soil analysis.
    """
    
    def __init__(self):
        super().__init__('simple_farm_client')
        
        # Create clients for both services
        self.irrigation_client = self.create_client(SetBool, '/irrigation_control')
        self.soil_client = self.create_client(SetBool, '/soil_analysis')
        
        # Subscribe to results
        self.irrigation_status_sub = self.create_subscription(
            String, '/irrigation_status', self.irrigation_status_callback, 10
        )
        self.soil_results_sub = self.create_subscription(
            String, '/soil_analysis_results', self.soil_results_callback, 10
        )
        
        self.get_logger().info("🚜 Simple Farm Client started")
        
        # Wait for services
        self.wait_for_services()
    
    def wait_for_services(self):
        """Wait for both services to be available."""
        self.get_logger().info("⏳ Waiting for services...")
        
        # Wait for irrigation service
        while not self.irrigation_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("⏳ Waiting for irrigation service...")
        
        # Wait for soil analysis service
        while not self.soil_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("⏳ Waiting for soil analysis service...")
        
        self.get_logger().info("✅ All services connected!")
    
    def analyze_soil(self, zone_id):
        """Request soil analysis for a zone."""
        request = SetBool.Request()
        request.data = zone_id
        request.enable = True
        
        self.get_logger().info(f"🔬 Requesting soil analysis for {zone_id}")
        
        future = self.soil_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        
        if future.result() is not None:
            response = future.result()
            if response.success:
                self.get_logger().info(f"✅ Soil analysis: {response.message}")
                return True
            else:
                self.get_logger().error(f"❌ Soil analysis failed: {response.message}")
                return False
        else:
            self.get_logger().error("❌ Soil analysis service call failed")
            return False
    
    def start_irrigation(self, zone_id):
        """Start irrigation for a zone."""
        request = SetBool.Request()
        request.data = zone_id
        request.enable = True
        
        self.get_logger().info(f"🚰 Starting irrigation for {zone_id}")
        
        future = self.irrigation_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        
        if future.result() is not None:
            response = future.result()
            if response.success:
                self.get_logger().info(f"✅ Irrigation: {response.message}")
                return True
            else:
                self.get_logger().error(f"❌ Irrigation failed: {response.message}")
                return False
        else:
            self.get_logger().error("❌ Irrigation service call failed")
            return False
    
    def stop_irrigation(self, zone_id):
        """Stop irrigation for a zone."""
        request = SetBool.Request()
        request.data = zone_id
        request.enable = False
        
        self.get_logger().info(f"🛑 Stopping irrigation for {zone_id}")
        
        future = self.irrigation_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        
        if future.result() is not None:
            response = future.result()
            if response.success:
                self.get_logger().info(f"✅ Irrigation: {response.message}")
                return True
            else:
                self.get_logger().error(f"❌ Stop irrigation failed: {response.message}")
                return False
        else:
            self.get_logger().error("❌ Irrigation service call failed")
            return False
    
    def irrigation_status_callback(self, msg):
        """Handle irrigation status updates."""
        self.get_logger().info(f"📊 Irrigation Status: {msg.data}")
    
    def soil_results_callback(self, msg):
        """Handle soil analysis results."""
        self.get_logger().info(f"📊 Soil Results: {msg.data}")
    
    def manage_zone(self, zone_id):
        """Demonstrate managing a single zone with both services."""
        self.get_logger().info(f"🌾 Managing {zone_id}")
        
        # Step 1: Analyze soil
        if self.analyze_soil(zone_id):
            time.sleep(2)  # Wait for results
            
            # Step 2: Start irrigation
            if self.start_irrigation(zone_id):
                time.sleep(3)  # Let it run for a moment
                
                # Step 3: Stop irrigation
                self.stop_irrigation(zone_id)
        
        self.get_logger().info(f"✅ Finished managing {zone_id}")
    
    def run_farm_demo(self):
        """Run a demonstration of farm management."""
        self.get_logger().info("🚜 Starting farm management demo...")
        
        # Manage different zones
        zones = ['zone_1', 'zone_2', 'zone_3']
        
        for zone_id in zones:
            self.get_logger().info(f"--- Managing {zone_id} ---")
            self.manage_zone(zone_id)
            time.sleep(2)  # Wait between zones
        
        self.get_logger().info("✅ Farm management demo completed!")


def main():
    """Main function."""
    rclpy.init()
    
    farm_client = SimpleFarmClient()
    
    try:
        import sys
        
        if len(sys.argv) > 1:
            if sys.argv[1] == "demo":
                farm_client.run_farm_demo()
            elif sys.argv[1] == "manage" and len(sys.argv) >= 3:
                zone_id = sys.argv[2]
                farm_client.manage_zone(zone_id)
            else:
                print("Usage:")
                print("  python3 farm_manager_client.py demo")
                print("  python3 farm_manager_client.py manage <zone_id>")
                print("  Available zones: zone_1, zone_2, zone_3, zone_4")
        else:
            # Default: run demo
            farm_client.run_farm_demo()
            
    except KeyboardInterrupt:
        farm_client.get_logger().info("🛑 Farm client stopped by user")
    finally:
        farm_client.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main() 