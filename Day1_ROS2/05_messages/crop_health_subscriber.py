#!/usr/bin/env python3
"""
Simple Crop Health Subscriber - RAISE 2025
Educational example demonstrating custom messages in ROS2.

This subscriber listens to crop health data and analyzes it.
Note: This is a simplified example using built-in messages since
custom messages require package compilation.
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import json
import time
import random


class CropHealthSubscriber(Node):
    """
    Simple subscriber for crop health data.
    Educational example focusing on custom message concepts.
    """

    def __init__(self):
        super().__init__('crop_health_subscriber')
        
        # Create subscriber for crop health data
        self.subscription = self.create_subscription(
            String,
            '/crop_health',
            self.crop_health_callback,
            10
        )
        
        # Create publisher for crop health data (for demo purposes)
        self.publisher = self.create_publisher(
            String,
            '/crop_health',
            10
        )
        
        # Timer to publish demo data every 8 seconds
        self.timer = self.create_timer(8.0, self.publish_demo_data)
        
        # Storage for health data
        self.crop_data = {}
        
        # Crop types and their typical health ranges
        self.crop_types = {
            "tomato": {"min_health": 70, "max_health": 95},
            "wheat": {"min_health": 60, "max_health": 85},
            "corn": {"min_health": 65, "max_health": 90},
            "herbs": {"min_health": 75, "max_health": 98}
        }
        
        self.get_logger().info("🌾 Crop Health Subscriber started")
        self.get_logger().info("📡 Listening to: /crop_health")
        self.get_logger().info("📊 Will analyze crop health data")

    def crop_health_callback(self, msg):
        """Handle received crop health data."""
        try:
            # Parse the JSON data
            crop_data = json.loads(msg.data)
            
            # Extract key information
            crop_type = crop_data.get("crop_type", "unknown")
            location = crop_data.get("location", "unknown")
            health_score = crop_data.get("health_score", 0)
            growth_stage = crop_data.get("growth_stage", "unknown")
            disease_detected = crop_data.get("disease_detected", False)
            
            # Store data
            self.crop_data[location] = crop_data
            
            # Analyze health
            self.analyze_crop_health(crop_data)
            
            # Log received data
            self.get_logger().info(
                f"🌱 Received: {crop_type} at {location} | "
                f"Health: {health_score}/100 | "
                f"Stage: {growth_stage} | "
                f"Disease: {'Yes' if disease_detected else 'No'}"
            )
            
        except json.JSONDecodeError as e:
            self.get_logger().error(f"❌ Error parsing crop health data: {e}")
        except Exception as e:
            self.get_logger().error(f"❌ Error processing crop health: {e}")

    def analyze_crop_health(self, crop_data):
        """Analyze crop health and provide recommendations."""
        crop_type = crop_data.get("crop_type", "unknown")
        health_score = crop_data.get("health_score", 0)
        disease_detected = crop_data.get("disease_detected", False)
        location = crop_data.get("location", "unknown")
        
        # Get expected health range for this crop type
        if crop_type in self.crop_types:
            min_health = self.crop_types[crop_type]["min_health"]
            max_health = self.crop_types[crop_type]["max_health"]
        else:
            min_health = 60
            max_health = 85
        
        # Analysis and recommendations
        if disease_detected:
            self.get_logger().warn(f"🚨 ALERT: Disease detected in {crop_type} at {location}")
            self.get_logger().warn(f"💊 Recommendation: Apply disease treatment immediately")
        
        if health_score < min_health:
            self.get_logger().warn(f"⚠️  LOW HEALTH: {crop_type} at {location} (Score: {health_score})")
            self.get_logger().warn(f"🔧 Recommendation: Increase nutrition and irrigation")
        elif health_score > max_health:
            self.get_logger().info(f"✅ EXCELLENT: {crop_type} at {location} (Score: {health_score})")
        else:
            self.get_logger().info(f"✅ HEALTHY: {crop_type} at {location} (Score: {health_score})")

    def generate_crop_health_data(self):
        """Generate demo crop health data."""
        crop_types = list(self.crop_types.keys())
        locations = ["field_A_section_1", "field_A_section_2", "field_B_section_1", "greenhouse_1"]
        growth_stages = ["seedling", "vegetative", "flowering", "mature"]
        diseases = ["", "leaf_spot", "powdery_mildew", "root_rot"]
        
        crop_type = random.choice(crop_types)
        location = random.choice(locations)
        growth_stage = random.choice(growth_stages)
        
        # Generate health score based on crop type
        health_range = self.crop_types[crop_type]
        health_score = round(random.uniform(
            health_range["min_health"] - 20,
            health_range["max_health"] + 5
        ), 1)
        
        # Disease probability
        disease_detected = random.random() < 0.2  # 20% chance of disease
        disease_type = random.choice(diseases[1:]) if disease_detected else ""
        
        return {
            "crop_type": crop_type,
            "growth_stage": growth_stage,
            "health_score": max(0, min(100, health_score)),  # Clamp to 0-100
            "disease_detected": disease_detected,
            "disease_type": disease_type,
            "location": location,
            "timestamp": int(time.time())
        }

    def publish_demo_data(self):
        """Publish demo crop health data."""
        # Generate demo data
        crop_data = self.generate_crop_health_data()
        
        # Create message
        msg = String()
        msg.data = json.dumps(crop_data)
        
        # Publish the message
        self.publisher.publish(msg)
        
        self.get_logger().info(f"📤 Published demo data for {crop_data['crop_type']} at {crop_data['location']}")

    def show_message_structure(self):
        """Display the message structure for educational purposes."""
        self.get_logger().info("📋 Custom CropHealth Message Structure:")
        self.get_logger().info("   string crop_type              # e.g., 'tomato', 'wheat', 'corn'")
        self.get_logger().info("   string growth_stage           # e.g., 'seedling', 'flowering', 'mature'")
        self.get_logger().info("   float64 health_score          # Health score (0-100)")
        self.get_logger().info("   bool disease_detected         # True if disease found")
        self.get_logger().info("   string disease_type           # Disease name if detected")
        self.get_logger().info("   string location              # Field location identifier")
        self.get_logger().info("   int64 timestamp              # Unix timestamp")

    def get_summary(self):
        """Get summary of all crop health data."""
        if not self.crop_data:
            self.get_logger().info("📊 No crop health data received yet")
            return
        
        total_crops = len(self.crop_data)
        healthy_crops = sum(1 for data in self.crop_data.values() if data.get("health_score", 0) >= 70)
        diseased_crops = sum(1 for data in self.crop_data.values() if data.get("disease_detected", False))
        
        self.get_logger().info(f"📊 CROP HEALTH SUMMARY:")
        self.get_logger().info(f"   Total monitored: {total_crops}")
        self.get_logger().info(f"   Healthy (≥70): {healthy_crops}")
        self.get_logger().info(f"   With disease: {diseased_crops}")


def main(args=None):
    """Main function to run the crop health subscriber."""
    rclpy.init(args=args)
    
    # Create and run the subscriber
    crop_subscriber = CropHealthSubscriber()
    
    try:
        crop_subscriber.show_message_structure()
        crop_subscriber.get_logger().info("🌾 Starting crop health monitoring...")
        rclpy.spin(crop_subscriber)
    except KeyboardInterrupt:
        crop_subscriber.get_logger().info("🛑 Crop health subscriber stopped by user")
        crop_subscriber.get_summary()
    finally:
        crop_subscriber.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main() 