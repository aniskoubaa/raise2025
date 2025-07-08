# Module 1: ROS2 Computation Graph

## 🎯 Learning Objectives

By the end of this module, you will understand:
- ✅ What a ROS2 computation graph is
- ✅ The role of nodes, topics, services, and actions
- ✅ How to inspect and visualize the computation graph
- ✅ How to create your first ROS2 node
- ✅ Basic ROS2 command-line tools

**Duration:** 45 minutes  
**Difficulty:** Beginner  
**Prerequisites:** Basic Python knowledge

## 🤖 What is a Computation Graph?

Imagine you're building a smart greenhouse robot. This robot needs to:
1. **See** plants using cameras
2. **Analyze** plant health using AI
3. **Move** between crop rows
4. **Water** plants when needed
5. **Report** status to farmers

Instead of writing one giant program, ROS2 lets you create separate **nodes** for each task. These nodes communicate with each other through the **computation graph**.

```
Agricultural Robot Computation Graph
┌─────────────────────────────────────────────────────────────┐
│                                                             │
│  ┌─────────────┐    /plant_images    ┌─────────────┐       │
│  │   Camera    │ ──────────────────→ │  AI Vision  │       │
│  │    Node     │                     │    Node     │       │
│  └─────────────┘                     └─────────────┘       │
│                                             │               │
│                                             │ /health_status │
│                                             ▼               │
│  ┌─────────────┐    /move_commands    ┌─────────────┐       │
│  │ Navigation  │ ←─────────────────── │  Decision   │       │
│  │    Node     │                     │    Node     │       │
│  └─────────────┘                     └─────────────┘       │
│                                             │               │
│                                             │ /water_cmd    │
│                                             ▼               │
│  ┌─────────────┐                     ┌─────────────┐       │
│  │  Reporting  │                     │  Irrigation │       │
│  │    Node     │                     │    Node     │       │
│  └─────────────┘                     └─────────────┘       │
│                                                             │
└─────────────────────────────────────────────────────────────┘
```

## 📚 Core Concepts

### 1. Nodes
**Nodes** are individual programs that perform specific tasks.

**Examples in Agriculture:**
- `camera_node`: Captures images of plants
- `disease_detector`: Analyzes plant health
- `irrigation_controller`: Manages water systems
- `gps_navigator`: Handles robot movement

### 2. Topics
**Topics** are named channels for sending messages between nodes.

**Examples:**
- `/plant_images`: Camera sends images
- `/soil_moisture`: Sensor sends moisture data
- `/cmd_vel`: Navigation commands for robot movement
- `/crop_health`: AI analysis results

### 3. Services
**Services** are for request-response communication.

**Examples:**
- `/take_photo`: Request camera to capture image
- `/analyze_plant`: Request AI analysis of specific plant
- `/start_irrigation`: Request to start watering

### 4. Actions
**Actions** are for long-running tasks that provide feedback.

**Examples:**
- `/navigate_to_plant`: Move robot to specific location
- `/complete_field_scan`: Scan entire field (takes time)
- `/harvest_crop`: Autonomous harvesting task

## 🚀 Hands-On Exercise 1: Exploring the Graph

Let's start by exploring a simple ROS2 system using turtlesim (a robot turtle simulator).

### Step 1: Launch Turtlesim
```bash
# Terminal 1: Start the turtle simulator
ros2 run turtlesim turtlesim_node
```

You should see a blue window with a turtle in the center.

### Step 2: Control the Turtle
```bash
# Terminal 2: Start turtle teleop (keyboard control)
ros2 run turtlesim turtle_teleop_key
```

Try moving the turtle using arrow keys!

### Step 3: Inspect the Computation Graph
```bash
# Terminal 3: See what nodes are running
ros2 node list
```

**Expected Output:**
```
/turtlesim
/teleop_turtle
```

### Step 4: Explore Topics
```bash
# See all topics
ros2 topic list

# See topic details
ros2 topic info /turtle1/cmd_vel

# Watch messages being sent
ros2 topic echo /turtle1/cmd_vel
```

**Try This:** Move the turtle and watch the velocity commands in Terminal 3!

### Step 5: Explore Services
```bash
# See all services
ros2 service list

# Get service details
ros2 service type /turtle1/set_pen
```

**Try This:** Change the turtle's pen color:
```bash
ros2 service call /turtle1/set_pen turtlesim/srv/SetPen "{r: 255, g: 0, b: 0, width: 3, off: 0}"
```

## 🛠️ Hands-On Exercise 2: Create Your First Node

Let's create a simple agricultural sensor node that publishes soil moisture data.

### Step 1: Create the Node File
Create a new file `soil_sensor_node.py`:

```python
#!/usr/bin/env python3
"""
RAISE 2025 - Agricultural Soil Sensor Node
This node simulates a soil moisture sensor for smart farming.
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
import random
import time

class SoilSensorNode(Node):
    def __init__(self):
        # Initialize the node with a name
        super().__init__('soil_sensor_node')
        
        # Create a publisher for soil moisture data
        self.publisher_ = self.create_publisher(
            Float32,                    # Message type
            '/soil_moisture',           # Topic name
            10                          # Queue size
        )
        
        # Create a timer to publish data every 2 seconds
        timer_period = 2.0  # seconds
        self.timer = self.create_timer(timer_period, self.publish_soil_moisture)
        
        # Initialize sensor parameters
        self.sensor_location = "Greenhouse_A_Row_1"
        self.get_logger().info(f'Soil sensor node started for {self.sensor_location}')

    def publish_soil_moisture(self):
        """Publish soil moisture reading (simulated)"""
        # Simulate soil moisture reading (0-100%)
        moisture_level = random.uniform(20.0, 80.0)
        
        # Create message
        msg = Float32()
        msg.data = moisture_level
        
        # Publish the message
        self.publisher_.publish(msg)
        
        # Log the reading
        self.get_logger().info(f'Soil moisture: {moisture_level:.1f}%')

def main(args=None):
    # Initialize ROS2
    rclpy.init(args=args)
    
    # Create and run the node
    node = SoilSensorNode()
    
    try:
        # Keep the node running
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # Clean shutdown
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Step 2: Make the File Executable
```bash
chmod +x soil_sensor_node.py
```

### Step 3: Run Your Node
```bash
# Terminal 1: Run your soil sensor node
python3 soil_sensor_node.py
```

### Step 4: Monitor the Data
```bash
# Terminal 2: Watch the soil moisture data
ros2 topic echo /soil_moisture
```

### Step 5: Inspect Your Node
```bash
# Terminal 3: Check your node is running
ros2 node list

# Get detailed info about your node
ros2 node info /soil_sensor_node
```

## 🔍 Understanding the Code

Let's break down the soil sensor node:

### 1. Node Initialization
```python
super().__init__('soil_sensor_node')
```
- Creates a node named 'soil_sensor_node'
- This name appears in `ros2 node list`

### 2. Publisher Creation
```python
self.publisher_ = self.create_publisher(Float32, '/soil_moisture', 10)
```
- **Float32**: Message type for decimal numbers
- **'/soil_moisture'**: Topic name (global namespace)
- **10**: Queue size (how many messages to buffer)

### 3. Timer Setup
```python
self.timer = self.create_timer(timer_period, self.publish_soil_moisture)
```
- Calls `publish_soil_moisture()` every 2 seconds
- This creates a regular data stream

### 4. Message Publishing
```python
msg = Float32()
msg.data = moisture_level
self.publisher_.publish(msg)
```
- Creates a message, sets data, and publishes it

## 🌱 Agricultural Applications

### Real-World Examples

**Precision Agriculture:** A farm might have dozens of soil sensors across different fields:
```
Field_A_Row_1 → /field_a/row_1/soil_moisture
Field_A_Row_2 → /field_a/row_2/soil_moisture
Field_B_Row_1 → /field_b/row_1/soil_moisture
```

**Multi-Sensor Integration:** Each sensor node could publish multiple topics:
```
/sensors/soil_moisture
/sensors/temperature
/sensors/humidity
/sensors/ph_level
/sensors/nutrient_levels
```

## 🛠️ Exercise 3: Create a Plant Health Monitor

**Your Task:** Create a node that subscribes to soil moisture and determines plant health.

### Starter Code Template
```python
#!/usr/bin/env python3
"""
RAISE 2025 - Plant Health Monitor Node
TODO: Complete this node to monitor plant health based on soil moisture
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, String

class PlantHealthMonitor(Node):
    def __init__(self):
        super().__init__('plant_health_monitor')
        
        # TODO: Create a subscriber to /soil_moisture
        # Hint: Use self.create_subscription()
        
        # TODO: Create a publisher for /plant_health_status
        # Hint: Use String message type
        
        self.get_logger().info('Plant health monitor started')

    def soil_moisture_callback(self, msg):
        """Process soil moisture data and determine plant health"""
        moisture = msg.data
        
        # TODO: Implement health logic
        # If moisture < 30%: "Needs Water"
        # If moisture > 70%: "Too Wet"
        # Otherwise: "Healthy"
        
        # TODO: Publish health status
        pass

def main(args=None):
    rclpy.init(args=args)
    node = PlantHealthMonitor()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Test Your Solution
1. Run the soil sensor node
2. Run your plant health monitor node
3. Check the health status: `ros2 topic echo /plant_health_status`

## 🔧 ROS2 Command-Line Tools

### Essential Commands
```bash
# Node management
ros2 node list                          # List all nodes
ros2 node info /node_name              # Get node details
ros2 node kill /node_name              # Stop a node

# Topic management
ros2 topic list                        # List all topics
ros2 topic info /topic_name            # Get topic details
ros2 topic echo /topic_name            # Monitor messages
ros2 topic hz /topic_name              # Check message frequency
ros2 topic pub /topic_name msg_type "data"  # Publish manually

# Service management
ros2 service list                      # List all services
ros2 service type /service_name        # Get service type
ros2 service call /service_name type "request"  # Call service

# Graph visualization
ros2 run rqt_graph rqt_graph          # Visual graph viewer
```

## 📊 Debugging Tips

### Common Issues and Solutions

**1. "No module named 'rclpy'"**
```bash
# Solution: Install ROS2 Python packages
sudo apt install python3-rclpy
```

**2. "ros2 command not found"**
```bash
# Solution: Source ROS2 environment
source /opt/ros/humble/setup.bash
```

**3. Node not appearing in list**
```bash
# Check if node is actually running
ps aux | grep python3
# Check for error messages in terminal
```

**4. No messages on topic**
```bash
# Check if publisher is actually publishing
ros2 topic hz /topic_name
# Check topic connections
ros2 topic info /topic_name
```

## 🎯 Key Takeaways

1. **Nodes** are independent programs that perform specific tasks
2. **Topics** enable asynchronous communication between nodes
3. **Services** provide request-response communication
4. **Actions** handle long-running tasks with feedback
5. **ROS2 tools** help inspect and debug the system

## 📝 Module 1 Assessment

**Quick Quiz Questions:**
1. What is the purpose of a ROS2 node?
2. How do nodes communicate asynchronously?
3. What ROS2 command lists all running nodes?
4. What's the difference between a topic and a service?

**Practical Task:**
Create a node that publishes temperature data and another that subscribes to it.

---

**🎉 Congratulations!** You've completed Module 1. You now understand the basics of ROS2 computation graphs and have created your first agricultural sensor node.

**Next:** [Module 2: Publish/Subscribe](../02_publish_subscribe/) - Learn advanced publisher/subscriber patterns for robot communication.

## 📚 Additional Resources

- [ROS2 Documentation](https://docs.ros.org/en/humble/)
- [ROS2 Tutorials](https://docs.ros.org/en/humble/Tutorials.html)
- [Agricultural Robotics with ROS2](https://github.com/ros-agriculture)

---

**Questions? Need Help?**
- Ask your instructor during the lab session
- Check the troubleshooting section above
- Collaborate with your fellow students! 