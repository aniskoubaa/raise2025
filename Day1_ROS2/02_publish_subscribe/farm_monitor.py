#!/usr/bin/env python3
"""
RAISE 2025 - Farm Monitor
Subscribes to all sensor data and provides farm-wide analysis.

This demonstrates:
- Multiple subscribers in one node
- Dynamic callback creation
- Data aggregation and analysis
- Real-time monitoring and alerting

Author: RAISE 2025 Team
Date: July 2025
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, String
from sensor_msgs.msg import Temperature
import time
import statistics

class FarmMonitor(Node):
    def __init__(self):
        super().__init__('farm_monitor')
        
        # Data storage
        self.soil_data = {}
        self.temperature_data = {}
        self.last_update = {}
        
        # Thresholds for alerts
        self.soil_low_threshold = 25.0
        self.soil_high_threshold = 75.0
        self.temp_low_threshold = 10.0
        self.temp_high_threshold = 35.0
        
        # Create subscribers for all sensor locations
        self.create_farm_subscribers()
        
        # Farm status subscriber
        self.create_subscription(String, '/farm_status', self.status_callback, 10)
        
        # Analysis publisher
        self.analysis_publisher = self.create_publisher(String, '/farm_analysis', 10)
        
        # Analysis timer
        self.analysis_timer = self.create_timer(15.0, self.analyze_farm_data)
        
        self.get_logger().info('🖥️ Farm monitor started!')
        self.get_logger().info('📡 Subscribing to all sensor data...')
        self.get_logger().info(f'🚨 Alert thresholds: Soil [{self.soil_low_threshold}-{self.soil_high_threshold}%], Temp [{self.temp_low_threshold}-{self.temp_high_threshold}°C]')

    def create_farm_subscribers(self):
        """Create subscribers for all farm sensor locations"""
        farm_zones = {
            'field_a': ['row_1', 'row_2', 'row_3'],
            'field_b': ['row_1', 'row_2'],
            'greenhouse': ['section_1', 'section_2', 'section_3']
        }
        
        for zone, rows in farm_zones.items():
            for row in rows:
                location = f'{zone}_{row}'
                
                # Soil moisture subscriber
                soil_topic = f'/sensors/{zone}/{row}/soil_moisture'
                self.create_subscription(
                    Float32, 
                    soil_topic, 
                    lambda msg, loc=location: self.soil_callback(msg, loc), 
                    10
                )
                
                # Temperature subscriber
                temp_topic = f'/sensors/{zone}/{row}/temperature'
                self.create_subscription(
                    Temperature, 
                    temp_topic, 
                    lambda msg, loc=location: self.temp_callback(msg, loc), 
                    10
                )
        
        self.get_logger().info(f'📊 Created subscribers for {len(farm_zones)} zones')

    def soil_callback(self, msg, location):
        """Process soil moisture data"""
        self.soil_data[location] = msg.data
        self.last_update[location] = time.time()
        
        # Immediate alert for critical conditions
        if msg.data < self.soil_low_threshold:
            self.get_logger().warn(f'🚨 CRITICAL: Very low soil moisture at {location}: {msg.data:.1f}%')
            self.publish_alert(f"LOW_SOIL_MOISTURE|{location}|{msg.data:.1f}")
        elif msg.data > self.soil_high_threshold:
            self.get_logger().warn(f'⚠️ WARNING: High soil moisture at {location}: {msg.data:.1f}%')
            self.publish_alert(f"HIGH_SOIL_MOISTURE|{location}|{msg.data:.1f}")
        else:
            self.get_logger().debug(f'📊 Soil moisture at {location}: {msg.data:.1f}% - NORMAL')

    def temp_callback(self, msg, location):
        """Process temperature data"""
        self.temperature_data[location] = msg.temperature
        
        # Temperature alerts
        if msg.temperature > self.temp_high_threshold:
            self.get_logger().warn(f'🌡️ HIGH TEMP: {location}: {msg.temperature:.1f}°C')
            self.publish_alert(f"HIGH_TEMPERATURE|{location}|{msg.temperature:.1f}")
        elif msg.temperature < self.temp_low_threshold:
            self.get_logger().warn(f'🧊 LOW TEMP: {location}: {msg.temperature:.1f}°C')
            self.publish_alert(f"LOW_TEMPERATURE|{location}|{msg.temperature:.1f}")
        else:
            self.get_logger().debug(f'🌡️ Temperature at {location}: {msg.temperature:.1f}°C - NORMAL')

    def status_callback(self, msg):
        """Process farm status updates"""
        self.get_logger().info(f'📊 Farm Status: {msg.data}')

    def publish_alert(self, alert_data):
        """Publish immediate alerts"""
        alert_msg = String()
        alert_msg.data = f"ALERT|{alert_data}|{time.time()}"
        self.analysis_publisher.publish(alert_msg)

    def analyze_farm_data(self):
        """Analyze all farm data and publish comprehensive insights"""
        if not self.soil_data:
            self.get_logger().warn('⚠️ No soil data available for analysis')
            return
        
        # Calculate soil moisture statistics
        soil_values = list(self.soil_data.values())
        avg_soil = statistics.mean(soil_values)
        min_soil = min(soil_values)
        max_soil = max(soil_values)
        std_soil = statistics.stdev(soil_values) if len(soil_values) > 1 else 0
        
        # Calculate temperature statistics
        temp_values = list(self.temperature_data.values())
        avg_temp = statistics.mean(temp_values) if temp_values else 0
        min_temp = min(temp_values) if temp_values else 0
        max_temp = max(temp_values) if temp_values else 0
        
        # Generate detailed analysis
        analysis_lines = [
            f"=== FARM ANALYSIS REPORT ===",
            f"Timestamp: {time.strftime('%Y-%m-%d %H:%M:%S')}",
            f"Soil Moisture: Avg={avg_soil:.1f}%, Min={min_soil:.1f}%, Max={max_soil:.1f}%, StdDev={std_soil:.1f}%",
            f"Temperature: Avg={avg_temp:.1f}°C, Min={min_temp:.1f}°C, Max={max_temp:.1f}°C",
            f"Sensors: {len(self.soil_data)} soil, {len(self.temperature_data)} temperature"
        ]
        
        # Add zone-specific analysis
        zone_analysis = self.analyze_by_zone()
        analysis_lines.extend(zone_analysis)
        
        # Add recommendations
        recommendations = self.generate_recommendations(avg_soil, avg_temp, std_soil)
        analysis_lines.extend(recommendations)
        
        # Publish comprehensive analysis
        analysis_text = " | ".join(analysis_lines)
        analysis_msg = String()
        analysis_msg.data = analysis_text
        self.analysis_publisher.publish(analysis_msg)
        
        # Log summary
        self.get_logger().info(f'📈 Analysis: Soil {avg_soil:.1f}±{std_soil:.1f}%, Temp {avg_temp:.1f}°C, {len(recommendations)} recommendations')

    def analyze_by_zone(self):
        """Analyze data by farm zones"""
        zones = {}
        
        # Group data by zone
        for location, moisture in self.soil_data.items():
            zone = location.split('_')[0]
            if zone not in zones:
                zones[zone] = {'soil': [], 'temp': []}
            zones[zone]['soil'].append(moisture)
            
            if location in self.temperature_data:
                zones[zone]['temp'].append(self.temperature_data[location])
        
        # Calculate zone statistics
        zone_reports = []
        for zone, data in zones.items():
            soil_avg = statistics.mean(data['soil']) if data['soil'] else 0
            temp_avg = statistics.mean(data['temp']) if data['temp'] else 0
            zone_reports.append(f"{zone.upper()}: Soil {soil_avg:.1f}%, Temp {temp_avg:.1f}°C")
        
        return zone_reports

    def generate_recommendations(self, avg_soil, avg_temp, std_soil):
        """Generate actionable recommendations"""
        recommendations = []
        
        # Soil moisture recommendations
        if avg_soil < 30:
            recommendations.append("REC: Increase overall irrigation")
        elif avg_soil > 70:
            recommendations.append("REC: Reduce irrigation frequency")
        
        # Temperature recommendations
        if avg_temp > 30:
            recommendations.append("REC: Increase ventilation/cooling")
        elif avg_temp < 15:
            recommendations.append("REC: Increase heating")
        
        # Variability recommendations
        if std_soil > 20:
            recommendations.append("REC: Check irrigation system uniformity")
        
        # Data quality recommendations
        if len(self.soil_data) < 5:
            recommendations.append("REC: Check sensor connectivity")
        
        return recommendations if recommendations else ["STATUS: All systems operating normally"]

def main(args=None):
    rclpy.init(args=args)
    node = FarmMonitor()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('🛑 Shutting down farm monitor...')
    finally:
        node.destroy_node()
        rclpy.shutdown()
        print("✅ Farm monitor shut down successfully!")

if __name__ == '__main__':
    main() 