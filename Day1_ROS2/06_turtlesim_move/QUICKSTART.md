# Module 6 Quickstart Guide: Turtlesim Movement Patterns

## Overview
This guide shows you how to control robot movement using turtlesim, demonstrating agricultural field patterns. You'll learn coordinate systems, velocity control, and movement patterns used in real farming robots.

## Prerequisites
- ROS2 installed and sourced
- Python 3.7+
- Turtlesim package installed
- Basic understanding of ROS2 concepts

## What You'll Learn
- Robot movement control with velocity commands
- Agricultural movement patterns (rows, spirals, perimeters)
- Coordinate systems and position feedback
- Visual feedback for movement validation

## Quick Start (5-minute demo)

### Step 1: Start Turtlesim
```bash
# Terminal 1: Start the turtlesim simulator
ros2 run turtlesim turtlesim_node
```

You should see a window with a turtle in the center.

### Step 2: Test Basic Movement
```bash
# Terminal 2: Test manual control
ros2 run turtlesim turtle_teleop_key
```

Use arrow keys to move the turtle around, then press `Ctrl+C` to stop.

### Step 3: Run Field Row Pattern
```bash
# Terminal 2: Run automated field row pattern
cd RAISE2025/Day1_ROS2/06_turtlesim_move
python3 field_row_pattern.py
```

**Expected Output:**
```
🚜 Field Row Pattern Controller started
📏 Field: 4 rows, 6.0m long, 1.2m apart
⏳ Waiting for turtle position...
📋 FIELD ROW PATTERN INFO:
   Number of rows: 4
   Row length: 6.0 units
   Row spacing: 1.2 units
   Forward speed: 2.0 units/sec
   Rotation speed: 1.5 rad/sec

🌾 AGRICULTURAL APPLICATION:
   - Harvesting crops planted in rows
   - Systematic crop monitoring
   - Precision spraying along crop lines
   - Seeding operations
🌾 Starting field row following pattern...
📍 Row 1/4
🚜 Moving forward 6.0m
✅ Moved 6.0m
🔄 Moving to next row...
🔄 Rotating 90.0°
✅ Rotation complete
```

Watch the turtle draw a field row pattern in the turtlesim window!

## Understanding Movement Patterns

### 1. Field Row Pattern
**Purpose**: Simulate harvesting crops planted in rows

**Pattern**:
```
Row 1: ────────────────► 
       ┌─────────────────┐
       │                 │
       ▼                 │
Row 2: ◄─────────────────┘
```

**Applications**:
- Crop harvesting
- Row-by-row monitoring
- Precision spraying
- Seeding operations

### 2. Spiral Coverage Pattern
**Purpose**: Efficient area coverage from center outward

```bash
# Terminal 2: Run spiral pattern
python3 spiral_coverage.py
```

**Expected Output**:
```
🌀 Spiral Coverage Pattern Controller started
📏 Coverage: radius up to 4.0 units
📍 Moving to field center...
✅ Reached field center
🌀 Simple spiral demo (continuous movement)...
✅ Simple spiral demo completed!
```

### 3. Rectangle Perimeter Pattern
**Purpose**: Field boundary inspection

```bash
# Terminal 2: Run perimeter pattern
python3 rectangle_perimeter.py
```

**Expected Output**:
```
📐 Rectangle Perimeter Pattern Controller started
📏 Field: 7.0x5.0 units
📍 Moving to field corner...
✅ Reached field corner
📐 Starting rectangle perimeter inspection...
📍 Bottom side (1/4)
➡️  Moving forward 7.0 units
✅ Moved 7.0 units
🔄 Turning left (90°)
✅ Turn completed
```

## Monitoring and Debugging

### Monitor Turtle Position
```bash
# Terminal 3: Watch turtle position updates
ros2 topic echo /turtle1/pose
```

You'll see:
```
x: 5.544444561004639
y: 5.544444561004639
theta: 0.0
linear_velocity: 0.0
angular_velocity: 0.0
```

### Monitor Velocity Commands
```bash
# Terminal 3: Watch velocity commands being sent
ros2 topic echo /turtle1/cmd_vel
```

You'll see:
```
linear:
  x: 2.0
  y: 0.0
  z: 0.0
angular:
  x: 0.0
  y: 0.0
  z: 0.0
```

### Clear Turtle Path
```bash
# Clear the drawing in turtlesim
ros2 service call /clear std_srvs/srv/Empty
```

## Understanding the Code

### Basic Movement Control
```python
# Create velocity message
vel_msg = Twist()
vel_msg.linear.x = 2.0      # Forward speed (units/sec)
vel_msg.angular.z = 1.5     # Rotation speed (rad/sec)

# Publish velocity command
self.velocity_pub.publish(vel_msg)
```

### Position Feedback
```python
def pose_callback(self, msg):
    """Handle turtle position updates."""
    self.current_pose = msg
    current_x = msg.x        # X position (0-11)
    current_y = msg.y        # Y position (0-11)
    current_theta = msg.theta # Orientation (radians)
```

### Distance-Based Movement
```python
def move_forward(self, distance):
    """Move turtle forward by specified distance."""
    start_x = self.current_pose.x
    start_y = self.current_pose.y
    
    while rclpy.ok():
        # Calculate distance traveled
        distance_traveled = math.sqrt(
            (self.current_pose.x - start_x)**2 + 
            (self.current_pose.y - start_y)**2
        )
        
        if distance_traveled >= distance:
            break
        
        # Continue moving
        self.velocity_pub.publish(vel_msg)
```

## Coordinate System
Turtlesim uses a coordinate system where:
- **X-axis**: Left (0) to right (11.09)
- **Y-axis**: Bottom (0) to top (11.09)
- **Origin**: Bottom-left corner (0, 0)
- **Orientation**: Radians (0 = facing right, π/2 = facing up)

## Exercise Challenges

### Challenge 1: Simple Shapes
Create a turtle that draws basic shapes:

1. **Square**: 4 equal sides with 90° turns
2. **Circle**: Constant angular velocity
3. **Triangle**: 3 sides with 120° turns

### Challenge 2: Modify Field Patterns
Edit the field row pattern to:
1. Change the number of rows
2. Adjust row spacing
3. Modify movement speeds
4. Add more complex turns

### Challenge 3: Create Your Own Pattern
Design a pattern for:
1. **Zigzag Coverage**: Back-and-forth like a lawnmower
2. **Star Pattern**: Multi-pointed star shape
3. **Figure-8**: Continuous loops for training

### Challenge 4: Agricultural Applications
Think about how these patterns apply to:
1. **Drone Surveillance**: Systematic field monitoring
2. **Tractor Automation**: Autonomous field operations
3. **Robot Harvesting**: Efficient fruit collection
4. **Precision Spraying**: Accurate pesticide application

## Common Issues and Solutions

### Issue 1: Turtle Moves Too Fast
**Problem**: Turtle overshoots targets

**Solution**:
```python
# Reduce speeds
self.linear_speed = 1.0    # Instead of 2.0
self.angular_speed = 0.8   # Instead of 1.5
```

### Issue 2: Turtle Goes Off-Screen
**Problem**: Turtle moves outside the visible area

**Solution**:
```python
# Add boundary checks
if self.current_pose.x < 0.5 or self.current_pose.x > 10.5:
    self.get_logger().warn("Approaching boundary!")
    self.stop_turtle()
```

### Issue 3: Rotation Errors
**Problem**: Turtle doesn't turn to the right angle

**Solution**:
```python
# Normalize angles properly
while angle_diff > math.pi:
    angle_diff -= 2 * math.pi
while angle_diff < -math.pi:
    angle_diff += 2 * math.pi
```

### Issue 4: No Position Updates
**Problem**: Turtle position not updating

**Solution**:
```bash
# Check if turtlesim is running
ros2 node list | grep turtlesim

# Check topic connections
ros2 topic info /turtle1/pose
```

## ROS2 Commands for Turtlesim

### Basic Commands
```bash
# Start turtlesim
ros2 run turtlesim turtlesim_node

# Manual control
ros2 run turtlesim turtle_teleop_key

# List available topics
ros2 topic list

# List available services
ros2 service list
```

### Monitoring Commands
```bash
# Watch turtle position
ros2 topic echo /turtle1/pose

# Watch velocity commands
ros2 topic echo /turtle1/cmd_vel

# Monitor topic frequency
ros2 topic hz /turtle1/pose
```

### Service Commands
```bash
# Clear the drawing
ros2 service call /clear std_srvs/srv/Empty

# Reset turtle position
ros2 service call /reset std_srvs/srv/Empty

# Kill the turtle
ros2 service call /kill turtlesim/srv/Kill '{name: "turtle1"}'

# Spawn a new turtle
ros2 service call /spawn turtlesim/srv/Spawn '{x: 5, y: 5, theta: 0, name: "turtle2"}'
```

## Real-World Applications

### Agricultural Robotics
- **Autonomous Tractors**: Following GPS waypoints
- **Harvesting Robots**: Systematic fruit collection
- **Spray Drones**: Precise pesticide application
- **Monitoring Systems**: Regular field inspection

### Movement Patterns in Practice
- **Row Following**: Corn harvesting combines
- **Spiral Coverage**: Fertilizer spreading
- **Perimeter Inspection**: Fence line monitoring
- **Zigzag Patterns**: Orchard maintenance

### From Simulation to Reality
- **Velocity Commands** → Motor control signals
- **Position Feedback** → GPS coordinates
- **Coordinate System** → Real-world mapping
- **Safety Boundaries** → Geofencing

## Advanced Concepts

### Path Planning
```python
# Calculate optimal path between points
def calculate_path(start, end):
    # Simple straight-line path
    return [start, end]

# Follow a series of waypoints
def follow_waypoints(waypoints):
    for waypoint in waypoints:
        move_to_position(waypoint.x, waypoint.y)
```

### Obstacle Avoidance
```python
# Check for obstacles (simulated)
def check_obstacles():
    # In real robots, this would use sensors
    return False

# Avoid obstacles while moving
def safe_move_forward(distance):
    if not check_obstacles():
        move_forward(distance)
    else:
        self.get_logger().warn("Obstacle detected!")
```

### Speed Control
```python
# Adaptive speed based on conditions
def adaptive_speed(base_speed, conditions):
    if conditions['wet']:
        return base_speed * 0.7
    elif conditions['steep']:
        return base_speed * 0.5
    else:
        return base_speed
```

## Next Steps

After completing this module, you understand:
- ✅ Robot movement control with ROS2
- ✅ Agricultural movement patterns
- ✅ Coordinate systems and position feedback
- ✅ Visual feedback for movement validation
- ✅ Real-world applications in agricultural robotics

## Congratulations! 🎉

You've completed **Day 1** of the RAISE 2025 program! You now have a solid foundation in:

### Core ROS2 Concepts
- ✅ Computation graphs and node communication
- ✅ Publisher/subscriber patterns
- ✅ Service/client architecture
- ✅ Action servers and clients
- ✅ Custom message design
- ✅ Robot movement control

### Agricultural Applications
- ✅ Sensor networks and data collection
- ✅ Irrigation control systems
- ✅ Field navigation patterns
- ✅ Crop monitoring systems
- ✅ Robot coordination

### Programming Skills
- ✅ Python ROS2 development
- ✅ Asynchronous programming
- ✅ Error handling and logging
- ✅ System integration
- ✅ Real-time control

## Continue to Day 2! 🚀

Ready to add intelligence to your agricultural robots? Continue to:

**Day 2: AI and Computer Vision**
- YOLO object detection for crop recognition
- Gemini Vision API for plant disease detection
- OpenCV for image processing
- Integration with ROS2 for intelligent farming

The foundation you've built today will power the intelligent systems you'll create tomorrow!

## Additional Resources

### Documentation
- [ROS2 Official Documentation](https://docs.ros.org/en/humble/)
- [Turtlesim Tutorials](https://docs.ros.org/en/humble/Tutorials/Beginner-CLI-Tools/Introducing-Turtlesim/Introducing-Turtlesim.html)
- [Geometry Messages](https://docs.ros.org/en/humble/p/geometry_msgs/)

### Agricultural Robotics
- [Agricultural Robot Examples](https://www.agrirobotics.com/)
- [Precision Agriculture Technology](https://www.precisionag.com/)
- [Field Robotics Research](https://fieldrobotics.org/)

### Mathematics
- [Coordinate Systems](https://en.wikipedia.org/wiki/Coordinate_system)
- [Robotics Mathematics](https://www.roboticsacademy.org/mathematics/)
- [Path Planning Algorithms](https://planning.cs.uiuc.edu/) 