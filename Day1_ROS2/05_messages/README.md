# Module 5: Custom Messages

## Learning Objectives
By the end of this module, you will be able to:
- Understand when and why to create custom messages
- Create your own message types for agricultural data
- Use custom messages in publishers and subscribers
- Organize complex sensor data efficiently

## Agricultural Context: Sensor Data Structures
In agricultural robotics, we need specialized data structures for:
- **Soil Sensor Data**: pH, moisture, temperature, nutrients
- **Crop Health Data**: Disease detection, growth stage, yield estimates
- **Weather Station Data**: Temperature, humidity, wind, rainfall
- **Robot Status**: Battery, position, current task, alerts

## Why Custom Messages?

### Built-in Messages (Good for basic data)
```python
# Simple data types
std_msgs/String: "Hello"
std_msgs/Float64: 25.3
geometry_msgs/Point: {x: 1.0, y: 2.0, z: 0.0}
```

### Custom Messages (Perfect for complex data)
```python
# Agricultural sensor data
SoilSensor:
  ph: 6.8
  moisture: 75.2
  temperature: 22.1
  location: "field_A_zone_3"
  timestamp: 1640995200
```

## Simple Examples

### Example 1: Soil Sensor Message
```
# SoilSensor.msg
float64 ph                    # pH level (0-14)
float64 moisture_percent      # Moisture percentage (0-100)
float64 temperature_celsius   # Temperature in Celsius
string location              # Location identifier
int64 timestamp              # Unix timestamp
```

### Example 2: Crop Health Message
```
# CropHealth.msg
string crop_type              # e.g., "tomato", "wheat", "corn"
string growth_stage           # e.g., "seedling", "flowering", "mature"
float64 health_score          # Health score (0-100)
bool disease_detected         # True if disease found
string disease_type           # Disease name if detected
geometry_msgs/Point location  # GPS coordinates
```

### Example 3: Robot Status Message
```
# RobotStatus.msg
string robot_id               # Unique robot identifier
string current_task           # e.g., "navigating", "harvesting", "idle"
float64 battery_percentage    # Battery level (0-100)
geometry_msgs/Pose position   # Robot position and orientation
string[] alerts              # List of current alerts
bool emergency_stop          # Emergency stop status
```

## Files in this Module

1. **`msg/SoilSensor.msg`** - Soil sensor data structure
2. **`msg/CropHealth.msg`** - Crop health information
3. **`msg/RobotStatus.msg`** - Robot status data
4. **`soil_sensor_publisher.py`** - Publishes soil sensor data
5. **`crop_health_subscriber.py`** - Subscribes to crop health data
6. **`farm_data_monitor.py`** - Monitors all farm data types
7. **`QUICKSTART.md`** - Step-by-step guide

## Message Structure Rules

### Basic Types
```
bool       # True/False
int8       # -128 to 127
int16      # -32,768 to 32,767  
int32      # -2,147,483,648 to 2,147,483,647
int64      # Very large integers
float32    # 32-bit decimal numbers
float64    # 64-bit decimal numbers (more precise)
string     # Text
```

### Arrays
```
int32[] numbers              # Variable-length array
int32[5] fixed_numbers       # Fixed-length array of 5 elements
string[] crop_types          # Array of strings
```

### Other Messages
```
geometry_msgs/Point location      # Use existing message types
std_msgs/Header header           # Standard header with timestamp
sensor_msgs/Image camera_data    # Camera image data
```

## Creating Custom Messages

### Step 1: Create Message File
```bash
# Create message directory
mkdir -p msg

# Create SoilSensor.msg
echo "float64 ph
float64 moisture_percent
float64 temperature_celsius
string location
int64 timestamp" > msg/SoilSensor.msg
```

### Step 2: Use in Python Code
```python
# Import your custom message
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

## Real-World Applications

### Precision Agriculture
- **Data Collection**: Sensors send structured data
- **Decision Making**: Process multiple sensor types
- **Monitoring**: Track farm conditions over time
- **Automation**: React to specific data patterns

### Benefits of Custom Messages
- **Organization**: Group related data together
- **Efficiency**: Send all data in one message
- **Clarity**: Clear structure for other developers
- **Validation**: ROS2 checks message format

## Common Patterns

### Sensor Data Pattern
```python
# Always include these fields for sensor data
std_msgs/Header header        # Timestamp and frame info
string sensor_id             # Unique sensor identifier
bool sensor_healthy          # Sensor status
float64 data_value           # Main measurement
string units                 # Units of measurement
```

### Array Data Pattern
```python
# For multiple measurements
float64[] measurements       # Array of values
string[] measurement_types   # What each value represents
int64[] timestamps          # When each was measured
```

### Status Pattern
```python
# For system status messages
string component_name        # What component this is about
int32 status_code           # Numeric status (0=OK, 1=Warning, 2=Error)
string status_message       # Human-readable status
int64 last_update          # When status was last checked
```

## Best Practices

### Message Design
1. **Keep it simple**: Don't put too much in one message
2. **Use clear names**: `temperature_celsius` not `temp`
3. **Include metadata**: timestamps, sensor IDs, units
4. **Use appropriate types**: `float64` for precise measurements
5. **Document units**: Always specify units in comments

### Field Organization
```python
# Good organization
std_msgs/Header header       # Standard header first
string sensor_id            # Identification
float64 temperature         # Main data
float64 humidity            # Related data  
bool sensor_healthy         # Status last
```

### Naming Conventions
- **Messages**: PascalCase (`SoilSensor`, `CropHealth`)
- **Fields**: snake_case (`moisture_percent`, `sensor_id`)
- **Units**: Include in name (`temperature_celsius`)
- **Booleans**: Start with `is_` or descriptive (`sensor_healthy`)

## Integration Examples

### Multiple Sensor Types
```python
# Combine different sensors
farm_data = FarmData()
farm_data.soil_sensors = [soil1, soil2, soil3]
farm_data.weather_data = weather_reading
farm_data.robot_status = robot_state
```

### Time Series Data
```python
# Historical data collection
history = SensorHistory()
history.sensor_id = "soil_01"
history.readings = [reading1, reading2, reading3]
history.timestamps = [t1, t2, t3]
```

### Alerts and Notifications
```python
# Alert message
alert = FarmAlert()
alert.alert_type = "LOW_MOISTURE"
alert.severity = "WARNING"
alert.affected_zones = ["field_A", "field_B"]
alert.recommended_action = "Increase irrigation"
```

## Troubleshooting

### Common Issues

1. **Message not found**
   - Check message file exists in `msg/` directory
   - Verify package is built correctly
   - Import path should match package name

2. **Field type errors**
   - Check field types match message definition
   - Verify array sizes if using fixed arrays
   - Ensure string fields get string values

3. **Import errors**
   - Message package must be built first
   - Check Python path includes message package
   - Verify message generation worked

### Debug Commands
```bash
# List available message types
ros2 interface list | grep YourPackage

# Show message structure
ros2 interface show your_package/msg/SoilSensor

# Test message publishing
ros2 topic echo /soil_data

# Check message format
ros2 topic info /soil_data
```

## Exercise Ideas

### Simple Exercises
1. Create a `WeatherStation` message with temperature, humidity, and pressure
2. Make a publisher that sends weather data every 5 seconds
3. Create a subscriber that logs weather data to console

### Intermediate Exercises
1. Design a `CropMonitoring` message with growth stage and health metrics
2. Create multiple publishers for different crop types
3. Build a subscriber that tracks crop health over time

### Advanced Exercises
1. Design a complete farm monitoring system with multiple message types
2. Create a data logger that saves all sensor data to files
3. Build an alert system that triggers on specific conditions

## Next Steps

After completing this module, you understand:
- ✅ When to create custom messages
- ✅ How to design message structures
- ✅ How to use custom messages in code
- ✅ Best practices for message organization
- ✅ Real-world agricultural data structures

Continue to **Module 6: Turtlesim Movement** to practice movement patterns with a visual robot! 