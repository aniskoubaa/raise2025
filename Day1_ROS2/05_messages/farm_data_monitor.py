#!/usr/bin/env python3
"""
Simple Farm Data Monitor - RAISE 2025
Educational example demonstrating multiple custom message types

This monitor subscribes to multiple data types and provides a dashboard view.
Note: This is a simplified example using built-in messages since
custom messages require package compilation.
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import json
import time
import random


class FarmDataMonitor(Node):
    """
    Farm data monitor for multiple custom message types.
    Educational example showing message coordination.
    """

    def __init__(self):
        super().__init__('farm_data_monitor')
        
        # Create subscribers for different data types
        self.soil_sub = self.create_subscription(
            String, '/soil_data', self.soil_data_callback, 10
        )
        self.crop_sub = self.create_subscription(
            String, '/crop_health', self.crop_health_callback, 10
        )
        self.robot_sub = self.create_subscription(
            String, '/robot_status', self.robot_status_callback, 10
        )
        
        # Create publishers for demo data
        self.robot_pub = self.create_publisher(String, '/robot_status', 10)
        
        # Timer for robot status updates
        self.robot_timer = self.create_timer(6.0, self.publish_robot_status)
        
        # Timer for periodic dashboard updates
        self.dashboard_timer = self.create_timer(15.0, self.show_dashboard)
        
        # Data storage
        self.soil_data = {}
        self.crop_data = {}
        self.robot_data = {}
        
        # Farm alerts
        self.alerts = []
        
        self.get_logger().info("📊 Farm Data Monitor started")
        self.get_logger().info("📡 Monitoring: /soil_data, /crop_health, /robot_status")
        self.get_logger().info("🖥️  Dashboard updates every 15 seconds")

    def soil_data_callback(self, msg):
        """Handle soil sensor data."""
        try:
            data = json.loads(msg.data)
            location = data.get("location", "unknown")
            self.soil_data[location] = data
            
            # Check for alerts
            self.check_soil_alerts(data)
            
        except json.JSONDecodeError as e:
            self.get_logger().error(f"❌ Error parsing soil data: {e}")

    def crop_health_callback(self, msg):
        """Handle crop health data."""
        try:
            data = json.loads(msg.data)
            location = data.get("location", "unknown")
            self.crop_data[location] = data
            
            # Check for alerts
            self.check_crop_alerts(data)
            
        except json.JSONDecodeError as e:
            self.get_logger().error(f"❌ Error parsing crop health data: {e}")

    def robot_status_callback(self, msg):
        """Handle robot status data."""
        try:
            data = json.loads(msg.data)
            robot_id = data.get("robot_id", "unknown")
            self.robot_data[robot_id] = data
            
            # Check for alerts
            self.check_robot_alerts(data)
            
        except json.JSONDecodeError as e:
            self.get_logger().error(f"❌ Error parsing robot status: {e}")

    def check_soil_alerts(self, data):
        """Check soil data for alert conditions."""
        location = data.get("location", "unknown")
        ph = data.get("ph", 7.0)
        moisture = data.get("moisture_percent", 50)
        
        # pH alerts
        if ph < 5.5:
            self.add_alert(f"LOW pH: {location} (pH: {ph})")
        elif ph > 8.0:
            self.add_alert(f"HIGH pH: {location} (pH: {ph})")
        
        # Moisture alerts
        if moisture < 40:
            self.add_alert(f"LOW MOISTURE: {location} ({moisture}%)")
        elif moisture > 90:
            self.add_alert(f"HIGH MOISTURE: {location} ({moisture}%)")

    def check_crop_alerts(self, data):
        """Check crop health data for alert conditions."""
        location = data.get("location", "unknown")
        health_score = data.get("health_score", 100)
        disease_detected = data.get("disease_detected", False)
        
        # Health alerts
        if health_score < 60:
            self.add_alert(f"LOW HEALTH: {location} (Score: {health_score})")
        
        # Disease alerts
        if disease_detected:
            disease_type = data.get("disease_type", "unknown")
            self.add_alert(f"DISEASE: {disease_type} at {location}")

    def check_robot_alerts(self, data):
        """Check robot status for alert conditions."""
        robot_id = data.get("robot_id", "unknown")
        battery = data.get("battery_percentage", 100)
        emergency_stop = data.get("emergency_stop", False)
        
        # Battery alerts
        if battery < 20:
            self.add_alert(f"LOW BATTERY: {robot_id} ({battery}%)")
        
        # Emergency stop alerts
        if emergency_stop:
            self.add_alert(f"EMERGENCY STOP: {robot_id}")

    def add_alert(self, message):
        """Add an alert to the alert list."""
        current_time = int(time.time())
        alert = {
            "message": message,
            "timestamp": current_time,
            "time_str": time.strftime("%H:%M:%S", time.localtime(current_time))
        }
        
        # Avoid duplicate alerts
        if not any(a["message"] == message for a in self.alerts):
            self.alerts.append(alert)
            self.get_logger().warn(f"🚨 ALERT: {message}")
        
        # Keep only last 10 alerts
        if len(self.alerts) > 10:
            self.alerts = self.alerts[-10:]

    def generate_robot_status(self):
        """Generate demo robot status data."""
        robot_ids = ["robot_01", "robot_02", "robot_03"]
        tasks = ["idle", "navigating", "harvesting", "charging", "maintenance"]
        
        robot_id = random.choice(robot_ids)
        current_task = random.choice(tasks)
        
        # Battery level simulation
        if current_task == "charging":
            battery = round(random.uniform(70, 100), 1)
        else:
            battery = round(random.uniform(15, 95), 1)
        
        # Position simulation
        position_x = round(random.uniform(0, 100), 1)
        position_y = round(random.uniform(0, 50), 1)
        
        # Alerts simulation
        alerts = []
        if battery < 30:
            alerts.append("low_battery")
        if random.random() < 0.1:  # 10% chance of other alerts
            alerts.append(random.choice(["sensor_error", "gps_issue", "maintenance_due"]))
        
        return {
            "robot_id": robot_id,
            "current_task": current_task,
            "battery_percentage": battery,
            "position_x": position_x,
            "position_y": position_y,
            "alerts": alerts,
            "emergency_stop": random.random() < 0.02,  # 2% chance
            "timestamp": int(time.time())
        }

    def publish_robot_status(self):
        """Publish demo robot status data."""
        robot_data = self.generate_robot_status()
        
        msg = String()
        msg.data = json.dumps(robot_data)
        
        self.robot_pub.publish(msg)
        
        self.get_logger().info(f"📤 Published robot status: {robot_data['robot_id']} - {robot_data['current_task']}")

    def show_dashboard(self):
        """Display farm dashboard with all data."""
        self.get_logger().info("=" * 60)
        self.get_logger().info("🚜 FARM DASHBOARD")
        self.get_logger().info("=" * 60)
        
        # Soil sensor summary
        if self.soil_data:
            self.get_logger().info(f"🌱 SOIL SENSORS ({len(self.soil_data)} locations):")
            for location, data in self.soil_data.items():
                self.get_logger().info(
                    f"   {location}: pH={data.get('ph', 'N/A')}, "
                    f"Moisture={data.get('moisture_percent', 'N/A')}%, "
                    f"Temp={data.get('temperature_celsius', 'N/A')}°C"
                )
        else:
            self.get_logger().info("🌱 SOIL SENSORS: No data received")
        
        # Crop health summary
        if self.crop_data:
            self.get_logger().info(f"🌾 CROP HEALTH ({len(self.crop_data)} locations):")
            for location, data in self.crop_data.items():
                disease_status = "Disease!" if data.get('disease_detected', False) else "Healthy"
                self.get_logger().info(
                    f"   {location}: {data.get('crop_type', 'N/A')} - "
                    f"Health={data.get('health_score', 'N/A')}/100, "
                    f"Status={disease_status}"
                )
        else:
            self.get_logger().info("🌾 CROP HEALTH: No data received")
        
        # Robot status summary
        if self.robot_data:
            self.get_logger().info(f"🤖 ROBOTS ({len(self.robot_data)} active):")
            for robot_id, data in self.robot_data.items():
                self.get_logger().info(
                    f"   {robot_id}: {data.get('current_task', 'N/A')} - "
                    f"Battery={data.get('battery_percentage', 'N/A')}%, "
                    f"Position=({data.get('position_x', 'N/A')}, {data.get('position_y', 'N/A')})"
                )
        else:
            self.get_logger().info("🤖 ROBOTS: No data received")
        
        # Recent alerts
        if self.alerts:
            self.get_logger().info(f"🚨 RECENT ALERTS ({len(self.alerts)}):")
            for alert in self.alerts[-5:]:  # Show last 5 alerts
                self.get_logger().info(f"   [{alert['time_str']}] {alert['message']}")
        else:
            self.get_logger().info("🚨 ALERTS: None")
        
        self.get_logger().info("=" * 60)

    def show_message_structures(self):
        """Display all custom message structures."""
        self.get_logger().info("📋 CUSTOM MESSAGE STRUCTURES:")
        self.get_logger().info("")
        
        self.get_logger().info("1. SoilSensor.msg:")
        self.get_logger().info("   float64 ph")
        self.get_logger().info("   float64 moisture_percent")
        self.get_logger().info("   float64 temperature_celsius")
        self.get_logger().info("   string location")
        self.get_logger().info("   int64 timestamp")
        self.get_logger().info("")
        
        self.get_logger().info("2. CropHealth.msg:")
        self.get_logger().info("   string crop_type")
        self.get_logger().info("   string growth_stage")
        self.get_logger().info("   float64 health_score")
        self.get_logger().info("   bool disease_detected")
        self.get_logger().info("   string disease_type")
        self.get_logger().info("   string location")
        self.get_logger().info("   int64 timestamp")
        self.get_logger().info("")
        
        self.get_logger().info("3. RobotStatus.msg:")
        self.get_logger().info("   string robot_id")
        self.get_logger().info("   string current_task")
        self.get_logger().info("   float64 battery_percentage")
        self.get_logger().info("   float64 position_x")
        self.get_logger().info("   float64 position_y")
        self.get_logger().info("   string[] alerts")
        self.get_logger().info("   bool emergency_stop")
        self.get_logger().info("   int64 timestamp")


def main(args=None):
    """Main function to run the farm data monitor."""
    rclpy.init(args=args)
    
    # Create and run the monitor
    farm_monitor = FarmDataMonitor()
    
    try:
        farm_monitor.show_message_structures()
        farm_monitor.get_logger().info("📊 Starting farm data monitoring...")
        rclpy.spin(farm_monitor)
    except KeyboardInterrupt:
        farm_monitor.get_logger().info("🛑 Farm data monitor stopped by user")
        farm_monitor.show_dashboard()
    finally:
        farm_monitor.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main() 