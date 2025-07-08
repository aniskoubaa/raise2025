#!/usr/bin/env python3
"""
RAISE 2025 - Plant Health Monitor Node Template
TODO: Complete this node to monitor plant health based on soil moisture

Your task:
1. Create a subscriber to /soil_moisture topic
2. Create a publisher for /plant_health_status topic
3. Implement logic to determine plant health based on moisture levels
4. Publish health status messages

Author: Your Name Here
Date: July 2025
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, String

class PlantHealthMonitor(Node):
    def __init__(self):
        super().__init__('plant_health_monitor')
        
        # TODO: Create a subscriber to /soil_moisture
        # Hint: Use self.create_subscription(message_type, topic_name, callback, queue_size)
        # Your code here:
        
        
        # TODO: Create a publisher for /plant_health_status
        # Hint: Use self.create_publisher(message_type, topic_name, queue_size)
        # Your code here:
        
        
        # Initialize health monitoring parameters
        self.dry_threshold = 30.0      # Below this = too dry
        self.wet_threshold = 70.0      # Above this = too wet
        self.last_reading = 0.0        # Track last moisture reading
        
        self.get_logger().info('🌿 Plant health monitor started!')
        self.get_logger().info(f'📊 Dry threshold: {self.dry_threshold}%')
        self.get_logger().info(f'📊 Wet threshold: {self.wet_threshold}%')

    def soil_moisture_callback(self, msg):
        """Process soil moisture data and determine plant health"""
        moisture = msg.data
        self.last_reading = moisture
        
        # TODO: Implement health logic
        # If moisture < 30%: health_status = "Needs Water"
        # If moisture > 70%: health_status = "Too Wet"
        # Otherwise: health_status = "Healthy"
        
        # Your code here:
        
        
        # TODO: Create a String message and publish health status
        # Hint: 
        # health_msg = String()
        # health_msg.data = health_status
        # self.publisher_.publish(health_msg)
        
        # Your code here:
        
        
        # Log the analysis (this is already done for you)
        self.get_logger().info(f'💧 Moisture: {moisture:.1f}% → Health: {health_status}')

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

# INSTRUCTIONS FOR STUDENTS:
# 1. Replace the TODO comments with actual code
# 2. Test your node by running the soil sensor node first
# 3. Run your plant health monitor node
# 4. Check the output using: ros2 topic echo /plant_health_status
# 5. Verify the health status changes as moisture levels change

# EXPECTED BEHAVIOR:
# - When soil moisture < 30%: Publishes "Needs Water"
# - When soil moisture > 70%: Publishes "Too Wet"  
# - When soil moisture 30-70%: Publishes "Healthy" 