# Module 2 Quick Start Guide

This guide will help you run the advanced publish/subscribe exercises with multi-sensor farm networks.

## 🌾 Exercise 1: Multi-Sensor Farm Network

### Step 1: Navigate to the module directory
```bash
cd RAISE2025/Day1_ROS2/02_publish_subscribe/
```

### Step 2: Run the farm sensor network
```bash
# Terminal 1 - Start the sensor network
python3 farm_sensor_network.py
```

### Step 3: Run the farm monitor
```bash
# Terminal 2 - Start the farm monitor
python3 farm_monitor.py
```

### Step 4: Explore the topic structure
```bash
# Terminal 3 - See all topics
ros2 topic list

# Check specific sensor data
ros2 topic echo /sensors/field_a/row_1/soil_moisture

# Check farm analysis
ros2 topic echo /farm_analysis
```

### Step 5: Visualize the network
```bash
# Terminal 4 - Visualize the computation graph
ros2 run rqt_graph rqt_graph
```

---

## 🔍 Understanding the System

### Topic Structure
The farm sensor network creates a hierarchical topic structure:
```
/sensors/field_a/row_1/soil_moisture
/sensors/field_a/row_1/temperature
/sensors/field_a/row_2/soil_moisture
/sensors/field_a/row_2/temperature
...
/sensors/greenhouse/section_1/soil_moisture
/sensors/greenhouse/section_1/temperature
```

### Data Flow
1. **Sensor Network** → Publishes sensor data to multiple topics
2. **Farm Monitor** → Subscribes to all sensor topics
3. **Analysis** → Aggregates data and publishes insights

---

## 📊 Monitoring Commands

### Check Topic Activity
```bash
# List all active topics
ros2 topic list

# Check message frequency
ros2 topic hz /sensors/field_a/row_1/soil_moisture

# Monitor specific topics
ros2 topic echo /farm_status
ros2 topic echo /farm_analysis
```

### Node Information
```bash
# Check running nodes
ros2 node list

# Get detailed node info
ros2 node info /farm_sensor_network
ros2 node info /farm_monitor
```

### System Performance
```bash
# Check topic connections
ros2 topic info /sensors/field_a/row_1/soil_moisture

# Monitor all topics at once
ros2 topic list | head -10 | xargs -I {} ros2 topic echo {} --once
```

---

## 🚨 Expected Alerts

The farm monitor will generate alerts when:
- **Low Soil Moisture** (< 25%): `🚨 CRITICAL: Very low soil moisture`
- **High Soil Moisture** (> 75%): `⚠️ WARNING: High soil moisture`
- **High Temperature** (> 35°C): `🌡️ HIGH TEMP`
- **Low Temperature** (< 10°C): `🧊 LOW TEMP`

---

## 🔧 Troubleshooting

### Common Issues

**No sensor data appearing:**
```bash
# Check if sensor network is running
ros2 node list | grep farm_sensor_network

# Check if topics are being published
ros2 topic hz /sensors/field_a/row_1/soil_moisture
```

**Monitor not receiving data:**
```bash
# Check if monitor is running
ros2 node list | grep farm_monitor

# Check subscriber connections
ros2 topic info /sensors/field_a/row_1/soil_moisture
```

**ROS2 performance issues:**
```bash
# Check system resources
htop

# Restart ROS2 daemon
ros2 daemon stop
ros2 daemon start
```

---

## 📈 Expected Output

### Farm Sensor Network
```
[INFO] [1690123456.789] [farm_sensor_network]: 🌾 Farm sensor network started!
[INFO] [1690123456.790] [farm_sensor_network]: 📊 Monitoring 8 sensor locations
[INFO] [1690123456.791] [farm_sensor_network]: 📡 Publishing sensor data on multiple topics:
[INFO] [1690123456.792] [farm_sensor_network]:    • /sensors/field_a/row_1/soil_moisture
[INFO] [1690123456.793] [farm_sensor_network]:    • /sensors/field_a/row_1/temperature
[INFO] [1690123459.794] [farm_sensor_network]: 📊 Soil data update #1 - 8 sensors
[INFO] [1690123464.795] [farm_sensor_network]: 🌡️ Temperature data published to 8 sensors
```

### Farm Monitor
```
[INFO] [1690123465.123] [farm_monitor]: 🖥️ Farm monitor started!
[INFO] [1690123465.124] [farm_monitor]: 📡 Subscribing to all sensor data...
[INFO] [1690123465.125] [farm_monitor]: 🚨 Alert thresholds: Soil [25.0-75.0%], Temp [10.0-35.0°C]
[INFO] [1690123465.126] [farm_monitor]: 📊 Created subscribers for 3 zones
[INFO] [1690123467.127] [farm_monitor]: 📈 Analysis: Soil 45.2±12.3%, Temp 24.1°C, 1 recommendations
```

---

## 🎯 Learning Objectives Check

After completing this module, you should be able to:
- [ ] Create multiple publishers in one node
- [ ] Create multiple subscribers in one node  
- [ ] Understand hierarchical topic naming
- [ ] Aggregate data from multiple sources
- [ ] Generate real-time alerts and analysis
- [ ] Visualize complex ROS2 systems

---

## 🚀 Advanced Exercises

### Exercise A: Add More Sensors
Try adding humidity sensors:
1. Modify `farm_sensor_network.py` to publish humidity data
2. Update `farm_monitor.py` to subscribe to humidity topics
3. Add humidity-based alerts and analysis

### Exercise B: QoS Experiments
Experiment with different QoS settings:
1. Change QoS to BEST_EFFORT for sensor data
2. Use RELIABLE for critical alerts
3. Compare performance and reliability

### Exercise C: Data Logging
Add data logging functionality:
1. Save sensor readings to files
2. Create periodic reports
3. Implement data retention policies

---

## 📚 What's Next?

Once you've completed these exercises, you're ready for:
- [Module 3: Service/Client](../03_service_client/) - Synchronous communication
- [Module 4: Actions](../04_actions/) - Long-running tasks
- [Module 5: Custom Messages](../05_messages/) - Custom data types

**Excellent work! You're mastering advanced ROS2 communication patterns! 🌟** 