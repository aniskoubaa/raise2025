# Module 1 Quick Start Guide

This guide will help you get up and running with the computation graph exercises quickly.

## 🚀 Exercise 1: Exploring ROS2 with Turtlesim

### Step 1: Start turtlesim
```bash
# Terminal 1
ros2 run turtlesim turtlesim_node
```

### Step 2: Control the turtle
```bash
# Terminal 2
ros2 run turtlesim turtle_teleop_key
```

### Step 3: Explore the system
```bash
# Terminal 3
ros2 node list
ros2 topic list
ros2 topic echo /turtle1/cmd_vel
```

---

## 🌱 Exercise 2: Soil Sensor Node

### Step 1: Navigate to the module directory
```bash
cd RAISE2025/Day1_ROS2/01_computation_graph/
```

### Step 2: Run the soil sensor node
```bash
# Terminal 1
python3 soil_sensor_node.py
```

### Step 3: Monitor the data
```bash
# Terminal 2
ros2 topic echo /soil_moisture
```

### Step 4: Inspect the system
```bash
# Terminal 3
ros2 node list
ros2 node info /soil_sensor_node
ros2 topic info /soil_moisture
```

---

## 🌿 Exercise 3: Plant Health Monitor (Your Task)

### Step 1: Edit the template file
```bash
# Use your favorite editor to complete the template
nano plant_health_monitor_template.py
# or
code plant_health_monitor_template.py
```

### Step 2: Run the soil sensor (if not already running)
```bash
# Terminal 1
python3 soil_sensor_node.py
```

### Step 3: Run your plant health monitor
```bash
# Terminal 2
python3 plant_health_monitor_template.py
```

### Step 4: Check the health status
```bash
# Terminal 3
ros2 topic echo /plant_health_status
```

### Step 5: Verify the system
```bash
# Terminal 4
ros2 node list
ros2 topic list
ros2 run rqt_graph rqt_graph
```

---

## 🔧 Troubleshooting

### Common Issues

**"ros2 command not found"**
```bash
source /opt/ros/humble/setup.bash
```

**"Permission denied"**
```bash
chmod +x *.py
```

**"No module named 'rclpy'"**
```bash
sudo apt install python3-rclpy
```

**Node doesn't appear in `ros2 node list`**
- Check if the node is running without errors
- Look for error messages in the terminal

### Quick Tests

```bash
# Test ROS2 installation
ros2 --version

# Test Python ROS2 packages
python3 -c "import rclpy; print('ROS2 Python OK')"

# Check if nodes are running
ros2 node list

# Check if topics are active
ros2 topic list
```

---

## 📊 Expected Output

### Soil Sensor Node
```
[INFO] [1690123456.789] [soil_sensor_node]: 🌱 Soil sensor node started!
[INFO] [1690123456.790] [soil_sensor_node]: 📍 Location: Greenhouse_A_Row_1
[INFO] [1690123456.791] [soil_sensor_node]: 🆔 Sensor ID: SM001
[INFO] [1690123458.792] [soil_sensor_node]: 📊 Reading #1: 45.2% - 🟢 HEALTHY
[INFO] [1690123460.793] [soil_sensor_node]: 📊 Reading #2: 28.7% - 🔴 TOO DRY
```

### Plant Health Monitor
```
[INFO] [1690123465.123] [plant_health_monitor]: 🌿 Plant health monitor started!
[INFO] [1690123465.124] [plant_health_monitor]: 📊 Dry threshold: 30.0%
[INFO] [1690123465.125] [plant_health_monitor]: 📊 Wet threshold: 70.0%
[INFO] [1690123467.126] [plant_health_monitor]: 💧 Moisture: 45.2% → Health: Healthy
[INFO] [1690123469.127] [plant_health_monitor]: 💧 Moisture: 28.7% → Health: Needs Water
```

---

## 🎯 Success Criteria

You've successfully completed Module 1 when you can:
- [ ] Run turtlesim and control it with keyboard
- [ ] Explain what nodes and topics are
- [ ] Run the soil sensor node and see moisture data
- [ ] Complete the plant health monitor template
- [ ] See health status messages being published
- [ ] Use basic ROS2 command-line tools

---

## 📚 What's Next?

Once you've completed these exercises, you're ready for:
- [Module 2: Publish/Subscribe](../02_publish_subscribe/) - Advanced communication patterns
- More complex node interactions
- Custom message types
- Multi-node systems

**Great job! You're on your way to mastering ROS2! 🚀** 