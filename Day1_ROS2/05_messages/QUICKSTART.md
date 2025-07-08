# Module 5 Quickstart Guide: Custom Messages

## Overview
This guide shows you how to work with custom messages in ROS2 using agricultural sensor data examples. You'll learn how to structure complex data and use it in publishers and subscribers.

## Prerequisites
- ROS2 installed and sourced
- Python 3.7+
- Basic understanding of ROS2 concepts from previous modules

## What are Custom Messages?
Custom messages let you create structured data types for your specific needs:
- **Built-in messages**: Good for simple data (`String`, `Float64`)
- **Custom messages**: Perfect for complex, structured data

## Quick Start (5-minute demo)

### Terminal 1: Start Soil Sensor Publisher
```bash
cd RAISE2025/Day1_ROS2/05_messages
python3 soil_sensor_publisher.py
```

**Expected Output:**
```
🌱 Soil Sensor Publisher started
📊 Publishing data for 4 sensors
📡 Publishing to: /soil_data
📋 Custom SoilSensor Message Structure:
   float64 ph                    # pH level (0-14)
   float64 moisture_percent      # Moisture percentage (0-100)
   float64 temperature_celsius   # Temperature in Celsius
   string location              # Location identifier
   int64 timestamp              # Unix timestamp
🌱 Starting soil sensor data publishing...
📊 field_A_zone_1: pH=6.45, Moisture=72.3%, Temp=22.1°C
📊 field_A_zone_2: pH=6.62, Moisture=78.9%, Temp=23.4°C
```

### Terminal 2: Start Crop Health Monitor
```bash
cd RAISE2025/Day1_ROS2/05_messages
python3 crop_health_subscriber.py
```

**Expected Output:**
```
🌾 Crop Health Subscriber started
📡 Listening to: /crop_health
📊 Will analyze crop health data
📋 Custom CropHealth Message Structure:
   string crop_type              # e.g., 'tomato', 'wheat', 'corn'
   string growth_stage           # e.g., 'seedling', 'flowering', 'mature'
   float64 health_score          # Health score (0-100)
   bool disease_detected         # True if disease found
   string disease_type           # Disease name if detected
   string location              # Field location identifier
   int64 timestamp              # Unix timestamp
🌾 Starting crop health monitoring...
```

### Terminal 3: Start Farm Data Monitor
```bash
cd RAISE2025/Day1_ROS2/05_messages
python3 farm_data_monitor.py
```

**Expected Output:**
```
📊 Farm Data Monitor started
📡 Monitoring: /soil_data, /crop_health, /robot_status
🖥️  Dashboard updates every 15 seconds
📋 CUSTOM MESSAGE STRUCTURES:

1. SoilSensor.msg:
   float64 ph
   float64 moisture_percent
   float64 temperature_celsius
   string location
   int64 timestamp

📊 Starting farm data monitoring...
```

## Understanding Custom Messages

### Message Files
Custom messages are defined in `.msg` files:

**SoilSensor.msg:**
```
float64 ph                    # pH level (0-14)
float64 moisture_percent      # Moisture percentage (0-100)
float64 temperature_celsius   # Temperature in Celsius
string location              # Location identifier
int64 timestamp              # Unix timestamp
```

**CropHealth.msg:**
```
string crop_type              # e.g., "tomato", "wheat", "corn"
string growth_stage           # e.g., "seedling", "flowering", "mature"
float64 health_score          # Health score (0-100)
bool disease_detected         # True if disease found
string disease_type           # Disease name if detected
string location              # Field location identifier
int64 timestamp              # Unix timestamp
```

### Using Custom Messages in Code
```python
# In a real implementation with compiled custom messages:
from your_package.msg import SoilSensor

# Create message instance
soil_msg = SoilSensor()
soil_msg.ph = 6.8
soil_msg.moisture_percent = 75.2
soil_msg.temperature_celsius = 22.1
soil_msg.location = "field_A_zone_3"
soil_msg.timestamp = int(time.time())

# Publish the message
publisher.publish(soil_msg)
```

## Detailed Examples

### Example 1: Monitor Soil Data
Watch the soil sensor data being published:
```bash
# In a new terminal, monitor the soil data topic
ros2 topic echo /soil_data
```

You'll see JSON-formatted soil sensor data:
```
data: '{"ph": 6.45, "moisture_percent": 72.3, "temperature_celsius": 22.1, "location": "field_A_zone_1", "timestamp": 1640995200}'
```

### Example 2: Monitor Crop Health
Watch the crop health analysis in real-time:
```bash
# In a new terminal, monitor crop health
ros2 topic echo /crop_health
```

### Example 3: Monitor Robot Status
Watch robot status updates:
```bash
# In a new terminal, monitor robot status
ros2 topic echo /robot_status
```

## Understanding the Output

### Soil Sensor Data
- **pH**: Soil acidity/alkalinity (6.0-7.0 is typically good)
- **Moisture**: Soil water content percentage
- **Temperature**: Soil temperature in Celsius
- **Location**: Which field zone the sensor is in

### Crop Health Data
- **Crop Type**: What crop is being monitored
- **Growth Stage**: Current development stage
- **Health Score**: Overall health rating (0-100)
- **Disease Detection**: Whether diseases were found

### Robot Status Data
- **Robot ID**: Which robot is reporting
- **Current Task**: What the robot is doing
- **Battery**: Power level percentage
- **Position**: Location coordinates in the field

## Farm Dashboard
The farm data monitor provides a comprehensive dashboard every 15 seconds:

```
============================================================
🚜 FARM DASHBOARD
============================================================
🌱 SOIL SENSORS (4 locations):
   field_A_zone_1: pH=6.45, Moisture=72.3%, Temp=22.1°C
   field_A_zone_2: pH=6.62, Moisture=78.9%, Temp=23.4°C
🌾 CROP HEALTH (2 locations):
   field_A_section_1: tomato - Health=85/100, Status=Healthy
🤖 ROBOTS (2 active):
   robot_01: harvesting - Battery=67.8%, Position=(45.2, 23.1)
🚨 RECENT ALERTS (1):
   [14:23:15] LOW MOISTURE: field_B_zone_1 (35%)
============================================================
```

## Message Design Best Practices

### 1. Use Clear Field Names
```
# Good
temperature_celsius
moisture_percent

# Avoid
temp
moisture
```

### 2. Include Units in Names
```
# Good
distance_meters
speed_kmh
battery_percentage

# Avoid
distance
speed
battery
```

### 3. Add Timestamps
```
int64 timestamp    # Always include when data was collected
```

### 4. Use Appropriate Data Types
```
float64   # For precise measurements
int32     # For counts and IDs
bool      # For true/false status
string    # For text and identifiers
```

### 5. Group Related Data
```
# Good - group related sensor data together
float64 ph
float64 moisture_percent
float64 temperature_celsius

# Avoid - separate messages for each measurement
```

## Exercise Challenges

### Challenge 1: Monitor Data Types
1. Start all three programs
2. Watch the different data types being published
3. Observe how the farm monitor combines all data types
4. Note the alert system in action

### Challenge 2: Understand Message Structure
1. Look at the message definitions in the `msg/` folder
2. Compare them to the JSON data being published
3. Understand how each field maps to the output

### Challenge 3: Create Your Own Message
Design a message for weather station data:
```
# WeatherStation.msg
float64 temperature_celsius
float64 humidity_percent
float64 wind_speed_kmh
float64 rainfall_mm
string weather_condition
string location
int64 timestamp
```

### Challenge 4: Data Analysis
1. Run the farm monitor for several minutes
2. Watch for alerts being generated
3. Note which conditions trigger alerts
4. Observe the dashboard summary information

## Real-World Applications

### Precision Agriculture
- **Soil Monitoring**: Multiple sensors across large fields
- **Crop Health**: Disease detection and growth tracking
- **Equipment Status**: Robot fleet management
- **Environmental Data**: Weather and climate monitoring

### Benefits of Custom Messages
- **Organized Data**: Related information grouped together
- **Type Safety**: ROS2 validates message structure
- **Efficiency**: Send complex data in single messages
- **Clarity**: Clear data structure for team development

## Common Issues and Solutions

### Issue 1: JSON Parsing Errors
**Problem:** Error parsing message data

**Solution:**
- Check JSON format is valid
- Verify all required fields are present
- Use try/catch blocks for error handling

### Issue 2: Missing Data Fields
**Problem:** KeyError when accessing message fields

**Solution:**
```python
# Use .get() with defaults
ph = data.get("ph", 7.0)  # Default to 7.0 if missing
location = data.get("location", "unknown")
```

### Issue 3: Data Type Mismatches
**Problem:** Wrong data types in messages

**Solution:**
- Ensure numbers are float/int as expected
- Convert strings to appropriate types
- Validate data before publishing

## ROS2 Commands for Custom Messages

### List All Message Types
```bash
ros2 interface list | grep msg
```

### Show Message Structure
```bash
# For built-in messages
ros2 interface show std_msgs/msg/String

# For custom messages (after compilation)
ros2 interface show your_package/msg/SoilSensor
```

### Monitor Topics
```bash
# See available topics
ros2 topic list

# Monitor topic data
ros2 topic echo /soil_data

# Check topic info
ros2 topic info /soil_data
```

## Next Steps

After completing this module, you understand:
- ✅ When to use custom messages vs built-in types
- ✅ How to design message structures
- ✅ How to organize complex sensor data
- ✅ How to coordinate multiple data types
- ✅ Real-world agricultural data applications

Continue to **Module 6: Turtlesim Movement** to practice robot movement patterns with visual feedback!

## Advanced Concepts

### Message Arrays
```
# For multiple measurements
float64[] soil_ph_readings      # Array of pH values
string[] sensor_locations       # Array of location names
int64[] measurement_timestamps  # Array of timestamps
```

### Nested Messages
```
# Use other message types as fields
geometry_msgs/Point location    # GPS coordinates
std_msgs/Header header          # Standard timestamp info
```

### Message Composition
```
# Combine multiple message types
SoilSensor[] soil_readings      # Array of soil sensors
CropHealth[] crop_data          # Array of crop health data
RobotStatus[] robot_fleet       # Array of robot statuses
``` 