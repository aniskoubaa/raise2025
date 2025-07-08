# Module 2: Advanced Publish/Subscribe Patterns

## 🎯 Learning Objectives

By the end of this module, you will understand:
- ✅ Advanced publisher/subscriber communication patterns
- ✅ Multiple publishers and subscribers
- ✅ Quality of Service (QoS) settings
- ✅ Message filtering and processing
- ✅ Multi-robot coordination using topics
- ✅ Real-time data processing for agriculture

**Duration:** 45 minutes  
**Difficulty:** Intermediate  
**Prerequisites:** Completed Module 1 (Computation Graph)

## 🌾 Agricultural Robotics Context

In modern smart farming, multiple robots and sensors work together:

```
Smart Farm Network Architecture
┌─────────────────────────────────────────────────────────────────┐
│                    Farm Management System                        │
├─────────────────────────────────────────────────────────────────┤
│                                                                 │
│  ┌─────────────┐  ┌─────────────┐  ┌─────────────┐  ┌─────────────┐ │
│  │   Robot A   │  │   Robot B   │  │   Robot C   │  │   Robot D   │ │
│  │ (Monitoring)│  │ (Watering)  │  │ (Weeding)   │  │ (Harvesting)│ │
│  └─────────────┘  └─────────────┘  └─────────────┘  └─────────────┘ │
│         │                │                │                │        │
│         ▼                ▼                ▼                ▼        │
│  ┌─────────────────────────────────────────────────────────────────┐ │
│  │                   Shared Topics                                 │ │
│  │  /crop_health   /soil_data   /weather   /robot_status          │ │
│  └─────────────────────────────────────────────────────────────────┘ │
│                                 │                                   │
│                                 ▼                                   │
│  ┌─────────────────────────────────────────────────────────────────┐ │
│  │              Central Farm Controller                            │ │
│  │        (Coordinates all robots and sensors)                     │ │
│  └─────────────────────────────────────────────────────────────────┘ │
│                                                                 │
└─────────────────────────────────────────────────────────────────┘
```

## 📚 Core Concepts

### 1. Multiple Publishers/Subscribers

In real farms, you'll have:
- **Multiple sensors** publishing to the same topic
- **Multiple robots** subscribing to the same data
- **Broadcast communication** for coordination
- **Data fusion** from multiple sources

### 2. Quality of Service (QoS)

Different data types need different reliability:
- **Sensor data**: Can lose some readings (best effort)
- **Robot commands**: Must be reliable (reliable)
- **Emergency signals**: Must be immediate (real-time)

### 3. Message Types

Agricultural systems use various message formats:
- **Sensor readings**: Float32, sensor_msgs/Temperature
- **Robot positions**: geometry_msgs/Point, nav_msgs/Odometry
- **Images**: sensor_msgs/Image, sensor_msgs/CompressedImage
- **Commands**: geometry_msgs/Twist, std_msgs/String

## 🚀 Hands-On Exercise 1: Multi-Sensor Farm Network

Let's create a farm with multiple soil sensors and a central monitoring system.

### Step 1: Create a Multi-Sensor Publisher

Create `farm_sensor_network.py`:

```python
#!/usr/bin/env python3
"""
RAISE 2025 - Farm Sensor Network
Simulates multiple soil sensors across a farm publishing to different topics.
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

    def publish_soil_data(self):
        """Publish soil moisture data from all sensors"""
        for location, publisher in self.soil_publishers.items():
            # Simulate different moisture levels per zone
            zone = location.split('_')[0]
            base_moisture = {'field_a': 40, 'field_b': 55, 'greenhouse': 65}[zone]
            
            moisture = max(0, min(100, base_moisture + random.uniform(-20, 20)))
            
            msg = Float32()
            msg.data = moisture
            publisher.publish(msg)
        
        self.reading_count += 1
        self.get_logger().info(f'📊 Soil data update #{self.reading_count}')

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

    def publish_farm_status(self):
        """Publish overall farm status"""
        status_msg = String()
        status_msg.data = f"Farm operational - {len(self.soil_publishers)} sensors active"
        self.status_publisher.publish(status_msg)

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

if __name__ == '__main__':
    main()
```

### Step 2: Create a Farm Monitor

Create `farm_monitor.py`:

```python
#!/usr/bin/env python3
"""
RAISE 2025 - Farm Monitor
Subscribes to all sensor data and provides farm-wide analysis.
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, String
from sensor_msgs.msg import Temperature
import time

class FarmMonitor(Node):
    def __init__(self):
        super().__init__('farm_monitor')
        
        # Data storage
        self.soil_data = {}
        self.temperature_data = {}
        self.last_update = {}
        
        # Subscribe to all sensor topics
        self.create_subscription(Float32, '/sensors/field_a/row_1/soil_moisture', 
                               lambda msg: self.soil_callback(msg, 'field_a_row_1'), 10)
        self.create_subscription(Float32, '/sensors/field_a/row_2/soil_moisture', 
                               lambda msg: self.soil_callback(msg, 'field_a_row_2'), 10)
        self.create_subscription(Float32, '/sensors/field_a/row_3/soil_moisture', 
                               lambda msg: self.soil_callback(msg, 'field_a_row_3'), 10)
        
        self.create_subscription(Temperature, '/sensors/field_a/row_1/temperature', 
                               lambda msg: self.temp_callback(msg, 'field_a_row_1'), 10)
        self.create_subscription(Temperature, '/sensors/field_a/row_2/temperature', 
                               lambda msg: self.temp_callback(msg, 'field_a_row_2'), 10)
        
        # Farm status subscriber
        self.create_subscription(String, '/farm_status', self.status_callback, 10)
        
        # Analysis publisher
        self.analysis_publisher = self.create_publisher(String, '/farm_analysis', 10)
        
        # Analysis timer
        self.analysis_timer = self.create_timer(15.0, self.analyze_farm_data)
        
        self.get_logger().info('🖥️ Farm monitor started!')
        self.get_logger().info('📡 Listening to all sensor data...')

    def soil_callback(self, msg, location):
        """Process soil moisture data"""
        self.soil_data[location] = msg.data
        self.last_update[location] = time.time()
        
        # Immediate alert for critical conditions
        if msg.data < 20:
            self.get_logger().warn(f'🚨 CRITICAL: Very low soil moisture at {location}: {msg.data:.1f}%')
        elif msg.data > 80:
            self.get_logger().warn(f'⚠️ WARNING: High soil moisture at {location}: {msg.data:.1f}%')

    def temp_callback(self, msg, location):
        """Process temperature data"""
        self.temperature_data[location] = msg.temperature
        
        # Temperature alerts
        if msg.temperature > 35:
            self.get_logger().warn(f'🌡️ HIGH TEMP: {location}: {msg.temperature:.1f}°C')
        elif msg.temperature < 10:
            self.get_logger().warn(f'🧊 LOW TEMP: {location}: {msg.temperature:.1f}°C')

    def status_callback(self, msg):
        """Process farm status updates"""
        self.get_logger().info(f'📊 Farm Status: {msg.data}')

    def analyze_farm_data(self):
        """Analyze all farm data and publish insights"""
        if not self.soil_data:
            return
        
        # Calculate averages
        avg_soil = sum(self.soil_data.values()) / len(self.soil_data)
        avg_temp = sum(self.temperature_data.values()) / len(self.temperature_data) if self.temperature_data else 0
        
        # Generate analysis
        analysis = f"Farm Analysis - Avg Soil: {avg_soil:.1f}%, Avg Temp: {avg_temp:.1f}°C"
        
        # Add recommendations
        if avg_soil < 30:
            analysis += " | RECOMMENDATION: Increase irrigation"
        elif avg_soil > 70:
            analysis += " | RECOMMENDATION: Reduce irrigation"
        else:
            analysis += " | STATUS: Optimal conditions"
        
        # Publish analysis
        msg = String()
        msg.data = analysis
        self.analysis_publisher.publish(msg)
        
        self.get_logger().info(f'📈 {analysis}')

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

if __name__ == '__main__':
    main()
```

### Step 3: Run the Farm Network

```bash
# Terminal 1: Start the sensor network
python3 farm_sensor_network.py

# Terminal 2: Start the farm monitor
python3 farm_monitor.py

# Terminal 3: Monitor specific data
ros2 topic echo /farm_analysis

# Terminal 4: See all topics
ros2 topic list
```

## 🔧 Quality of Service (QoS) Profiles

Different agricultural data needs different QoS settings:

### Example: QoS Configuration

```python
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy

# For sensor data (can lose some readings)
sensor_qos = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    durability=DurabilityPolicy.VOLATILE,
    history=HistoryPolicy.KEEP_LAST,
    depth=10
)

# For robot commands (must be reliable)
command_qos = QoSProfile(
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
    history=HistoryPolicy.KEEP_LAST,
    depth=5
)

# Create publishers with specific QoS
self.sensor_pub = self.create_publisher(Float32, '/sensors/data', sensor_qos)
self.command_pub = self.create_publisher(String, '/robot/commands', command_qos)
```

## 🤖 Exercise 2: Robot Coordination System

Create a system where multiple robots coordinate their actions based on sensor data.

### Step 1: Create Robot Coordinator

Create `robot_coordinator.py`:

```python
#!/usr/bin/env python3
"""
RAISE 2025 - Robot Coordinator
Coordinates multiple robots based on sensor data and farm conditions.
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Float32
from geometry_msgs.msg import Point
import json

class RobotCoordinator(Node):
    def __init__(self):
        super().__init__('robot_coordinator')
        
        # Robot fleet status
        self.robots = {
            'irrigation_bot': {'status': 'idle', 'location': 'field_a_row_1'},
            'monitoring_bot': {'status': 'idle', 'location': 'field_a_row_2'},
            'weeding_bot': {'status': 'idle', 'location': 'field_b_row_1'}
        }
        
        # Task queue
        self.task_queue = []
        
        # Subscribers for sensor data
        self.create_subscription(Float32, '/sensors/field_a/row_1/soil_moisture',
                               lambda msg: self.soil_callback(msg, 'field_a_row_1'), 10)
        self.create_subscription(Float32, '/sensors/field_a/row_2/soil_moisture',
                               lambda msg: self.soil_callback(msg, 'field_a_row_2'), 10)
        
        # Subscribers for robot status
        self.create_subscription(String, '/robots/irrigation_bot/status', 
                               lambda msg: self.robot_status_callback(msg, 'irrigation_bot'), 10)
        self.create_subscription(String, '/robots/monitoring_bot/status',
                               lambda msg: self.robot_status_callback(msg, 'monitoring_bot'), 10)
        
        # Publishers for robot commands
        self.command_publishers = {}
        for robot in self.robots.keys():
            self.command_publishers[robot] = self.create_publisher(
                String, f'/robots/{robot}/commands', 10
            )
        
        # Coordination timer
        self.coordination_timer = self.create_timer(5.0, self.coordinate_robots)
        
        self.get_logger().info('🤖 Robot coordinator started!')
        self.get_logger().info(f'👥 Managing {len(self.robots)} robots')

    def soil_callback(self, msg, location):
        """Process soil data and create tasks"""
        moisture = msg.data
        
        # Create irrigation task if needed
        if moisture < 25:
            task = {
                'type': 'irrigation',
                'location': location,
                'priority': 'high',
                'moisture_level': moisture
            }
            self.task_queue.append(task)
            self.get_logger().info(f'📝 Added irrigation task for {location} (moisture: {moisture:.1f}%)')
        
        # Create monitoring task if readings are unusual
        elif moisture > 85:
            task = {
                'type': 'monitoring',
                'location': location,
                'priority': 'medium',
                'moisture_level': moisture
            }
            self.task_queue.append(task)
            self.get_logger().info(f'📝 Added monitoring task for {location} (high moisture: {moisture:.1f}%)')

    def robot_status_callback(self, msg, robot_name):
        """Update robot status"""
        self.robots[robot_name]['status'] = msg.data
        self.get_logger().info(f'🤖 {robot_name} status: {msg.data}')

    def coordinate_robots(self):
        """Coordinate robot activities based on tasks and robot availability"""
        if not self.task_queue:
            return
        
        # Sort tasks by priority
        self.task_queue.sort(key=lambda x: {'high': 1, 'medium': 2, 'low': 3}[x['priority']])
        
        # Assign tasks to available robots
        for task in self.task_queue[:]:  # Create copy to iterate
            suitable_robot = self.find_suitable_robot(task)
            
            if suitable_robot:
                self.assign_task(suitable_robot, task)
                self.task_queue.remove(task)

    def find_suitable_robot(self, task):
        """Find the best robot for a task"""
        for robot_name, robot_info in self.robots.items():
            if robot_info['status'] == 'idle':
                # Check if robot is suitable for task type
                if task['type'] == 'irrigation' and 'irrigation' in robot_name:
                    return robot_name
                elif task['type'] == 'monitoring' and 'monitoring' in robot_name:
                    return robot_name
        return None

    def assign_task(self, robot_name, task):
        """Assign a task to a robot"""
        command = {
            'action': task['type'],
            'target_location': task['location'],
            'priority': task['priority']
        }
        
        # Send command to robot
        msg = String()
        msg.data = json.dumps(command)
        self.command_publishers[robot_name].publish(msg)
        
        # Update robot status
        self.robots[robot_name]['status'] = 'busy'
        
        self.get_logger().info(f'📤 Assigned {task["type"]} task to {robot_name} at {task["location"]}')

def main(args=None):
    rclpy.init(args=args)
    node = RobotCoordinator()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('🛑 Shutting down robot coordinator...')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## 🌐 Advanced Message Patterns

### 1. Message Filtering

```python
def advanced_message_filter(self, msg):
    """Filter messages based on multiple criteria"""
    # Only process messages with specific conditions
    if hasattr(msg, 'temperature') and msg.temperature > 30:
        # Only process high temperature readings
        self.process_high_temp_data(msg)
    
    # Time-based filtering
    current_time = self.get_clock().now()
    if (current_time - self.last_processed).nanoseconds > 5e9:  # 5 seconds
        self.process_periodic_data(msg)
        self.last_processed = current_time
```

### 2. Data Aggregation

```python
def aggregate_sensor_data(self):
    """Aggregate data from multiple sensors"""
    if len(self.sensor_readings) < 3:
        return  # Wait for more data
    
    # Calculate statistics
    values = list(self.sensor_readings.values())
    avg_value = sum(values) / len(values)
    max_value = max(values)
    min_value = min(values)
    
    # Publish aggregated data
    summary = f"Sensors: {len(values)}, Avg: {avg_value:.1f}, Range: {min_value:.1f}-{max_value:.1f}"
    self.publish_summary(summary)
```

## 🔍 Debugging Multi-Node Systems

### Essential Commands

```bash
# Monitor all topics at once
ros2 topic list | xargs -I {} ros2 topic echo {} --once

# Check message rates
ros2 topic hz /sensors/field_a/row_1/soil_moisture

# Monitor node performance
ros2 node info /farm_monitor

# Visualize the complete system
ros2 run rqt_graph rqt_graph

# Check system performance
ros2 run plotjuggler plotjuggler
```

### Performance Monitoring

```python
def performance_monitor(self):
    """Monitor system performance"""
    import psutil
    
    # Check CPU usage
    cpu_percent = psutil.cpu_percent()
    
    # Check memory usage
    memory = psutil.virtual_memory()
    
    # Check message rates
    if hasattr(self, 'message_count'):
        rate = self.message_count / self.uptime
        self.get_logger().info(f'📊 Performance: CPU {cpu_percent}%, Memory {memory.percent}%, Rate {rate:.1f} msg/s')
```

## 🌱 Agricultural Applications

### Real-World Scenarios

**Precision Irrigation:**
```python
# Multiple soil sensors → Irrigation controller
'/sensors/field_a/row_1/soil_moisture' → Irrigation Controller
'/sensors/field_a/row_2/soil_moisture' → Irrigation Controller
'/sensors/field_a/row_3/soil_moisture' → Irrigation Controller

# Controller publishes targeted irrigation commands
'/irrigation/field_a/row_1/activate' ← Irrigation Controller
'/irrigation/field_a/row_2/activate' ← Irrigation Controller
```

**Crop Health Monitoring:**
```python
# Multiple cameras → Disease detection AI
'/cameras/field_a/row_1/image' → Disease Detector
'/cameras/field_a/row_2/image' → Disease Detector

# AI publishes health assessments
'/crop_health/field_a/row_1/status' ← Disease Detector
'/crop_health/field_a/row_2/status' ← Disease Detector
```

## 🎯 Key Takeaways

1. **Scalability**: Pub/sub scales naturally with multiple publishers/subscribers
2. **Decoupling**: Producers and consumers don't need to know about each other
3. **QoS**: Different data types need different quality guarantees
4. **Coordination**: Central coordinators can manage distributed systems
5. **Monitoring**: System-wide monitoring is essential for farm operations

## 📝 Module 2 Assessment

**Conceptual Questions:**
1. How does pub/sub enable multi-robot coordination?
2. When would you use RELIABLE vs BEST_EFFORT QoS?
3. How can you aggregate data from multiple sensors?

**Practical Task:**
Create a system with 3 temperature sensors and 1 monitor that publishes alerts when any sensor exceeds 35°C.

---

**🎉 Congratulations!** You've mastered advanced publish/subscribe patterns and can now coordinate multiple robots and sensors in agricultural systems.

**Next:** [Module 3: Service/Client](../03_service_client/) - Learn synchronous communication for robot control.

## 📚 Additional Resources

- [ROS2 QoS Documentation](https://docs.ros.org/en/humble/Concepts/About-Quality-of-Service-Settings.html)
- [ROS2 Performance Tuning](https://docs.ros.org/en/humble/Tutorials/Performance-Tuning.html)
- [Agricultural ROS2 Packages](https://github.com/ros-agriculture)

---

**Questions? Need Help?**
- Review the code examples above
- Check the troubleshooting section
- Ask your instructor for clarification 