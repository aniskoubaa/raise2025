# Module 6: Turtlesim Movement Patterns

## Learning Objectives
By the end of this module, you will be able to:
- Control the turtlesim robot with ROS2 commands
- Create agricultural movement patterns (field rows, spirals, rectangles)
- Understand velocity control and coordinate systems
- Apply movement patterns to real agricultural robotics

## Agricultural Context: Field Navigation Patterns
In agricultural robotics, robots need to follow specific movement patterns:
- **Row Following**: Moving between crop rows for harvesting
- **Field Coverage**: Systematic coverage for spraying/seeding  
- **Spiral Patterns**: Efficient area coverage from center outward
- **Perimeter Inspection**: Following field boundaries
- **Zigzag Patterns**: Back-and-forth coverage like a lawn mower

## What is Turtlesim?
Turtlesim is a simple ROS2 simulation tool that shows:
- A turtle (robot) moving in a 2D space
- Visual feedback of movement patterns
- Practice with velocity commands
- Understanding of coordinate systems

## Movement Concepts

### Coordinate System
- **X-axis**: Left to right (0 to 11.09)
- **Y-axis**: Bottom to top (0 to 11.09)
- **Origin**: Bottom-left corner (0, 0)
- **Orientation**: Angle in radians (0 = facing right)

### Velocity Control
```python
# Linear velocity (forward/backward)
linear.x = 2.0    # Move forward at 2 units/second
linear.x = -1.0   # Move backward at 1 unit/second

# Angular velocity (rotation)
angular.z = 1.57  # Rotate counterclockwise (π/2 radians/second)
angular.z = -1.57 # Rotate clockwise
```

## Files in this Module

1. **`field_row_pattern.py`** - Simulate crop row following
2. **`spiral_coverage.py`** - Spiral pattern for field coverage
3. **`rectangle_perimeter.py`** - Rectangular field boundary inspection
4. **`zigzag_pattern.py`** - Back-and-forth field coverage
5. **`turtle_farm_controller.py`** - Interactive farm pattern controller
6. **`QUICKSTART.md`** - Step-by-step guide

## Agricultural Movement Patterns

### 1. Row Following Pattern
```
Start ────────────────► End
      ┌─────────────────┐
      │                 │
      ▼                 │
Start ◄─────────────────┘ End
```
**Use Case**: Harvesting crops planted in rows

### 2. Spiral Coverage Pattern
```
      ┌─────────────┐
      │ ┌─────────┐ │
      │ │ ┌─────┐ │ │
      │ │ │Start│ │ │
      │ │ └─────┘ │ │
      │ └─────────┘ │
      └─────────────┘
```
**Use Case**: Efficient area coverage from center outward

### 3. Rectangle Perimeter Pattern
```
Start ►──────────────┐
      │              │
      │              ▼
      │              │
      │              │
      └──────────────┘ End
```
**Use Case**: Field boundary inspection, fence line following

### 4. Zigzag Coverage Pattern
```
Start ►────────────────┐
                       │
      ┌────────────────┘
      │
      └────────────────┐
                       │
      ┌────────────────┘ End
```
**Use Case**: Complete field coverage like a lawn mower

## Basic Commands

### Start Turtlesim
```bash
# Start the turtlesim node
ros2 run turtlesim turtlesim_node

# In another terminal, run movement patterns
python3 field_row_pattern.py
```

### Manual Control
```bash
# Control turtle with keyboard
ros2 run turtlesim turtle_teleop_key
```

### Monitor Position
```bash
# See turtle's current position
ros2 topic echo /turtle1/pose
```

## Real-World Applications

### Agricultural Robotics
- **Precision Farming**: Following GPS waypoints in real fields
- **Crop Monitoring**: Systematic coverage for imaging/sensing
- **Spraying Patterns**: Ensuring complete coverage without overlap
- **Harvesting Routes**: Efficient collection paths

### Movement Algorithms
- **Path Planning**: Computing optimal routes
- **Obstacle Avoidance**: Navigating around obstacles
- **GPS Following**: Converting GPS coordinates to movement
- **Speed Control**: Adjusting speed for different operations

## Programming Concepts

### Publishing Velocity Commands
```python
from geometry_msgs.msg import Twist

# Create velocity message
vel_msg = Twist()
vel_msg.linear.x = 2.0      # Forward speed
vel_msg.angular.z = 0.5     # Rotation speed

# Publish the command
publisher.publish(vel_msg)
```

### Subscribing to Position
```python
from turtlesim.msg import Pose

def pose_callback(msg):
    current_x = msg.x
    current_y = msg.y
    current_theta = msg.theta
    print(f"Position: ({current_x:.2f}, {current_y:.2f})")

# Subscribe to turtle position
subscription = self.create_subscription(
    Pose, '/turtle1/pose', pose_callback, 10
)
```

### Movement Functions
```python
def move_forward(distance):
    """Move turtle forward by specified distance."""
    vel_msg = Twist()
    vel_msg.linear.x = 2.0  # Speed
    
    # Calculate time needed
    time_needed = distance / 2.0
    
    # Publish velocity for calculated time
    # (Implementation details in actual code)

def rotate(angle):
    """Rotate turtle by specified angle in radians."""
    vel_msg = Twist()
    vel_msg.angular.z = 1.0  # Rotation speed
    
    # Calculate time needed for rotation
    time_needed = abs(angle) / 1.0
    
    # Publish angular velocity
```

## Pattern Implementations

### Field Row Pattern
```python
def follow_field_rows():
    """Simulate following crop rows."""
    row_length = 8.0
    row_spacing = 1.0
    num_rows = 5
    
    for row in range(num_rows):
        # Move forward along row
        move_forward(row_length)
        
        if row < num_rows - 1:  # Not last row
            # Turn to next row
            rotate(math.pi/2)      # Turn 90 degrees
            move_forward(row_spacing)
            rotate(math.pi/2)      # Turn 90 degrees again
```

### Spiral Coverage
```python
def spiral_coverage():
    """Create expanding spiral pattern."""
    radius = 0.5
    angle_increment = 0.1
    
    for step in range(200):
        # Calculate position on spiral
        x = radius * math.cos(step * angle_increment)
        y = radius * math.sin(step * angle_increment)
        
        # Move to position
        move_to_position(x, y)
        
        # Expand spiral
        radius += 0.02
```

## Safety and Best Practices

### Movement Safety
1. **Check boundaries**: Ensure turtle stays within simulation area
2. **Smooth movements**: Avoid sudden velocity changes
3. **Stop conditions**: Always have a way to stop the turtle
4. **Speed limits**: Don't exceed reasonable velocities

### Code Structure
1. **Modular functions**: Separate functions for different movements
2. **Error handling**: Check for valid positions and velocities
3. **Clear naming**: Use descriptive function and variable names
4. **Documentation**: Comment complex movement logic

## Integration with Other Modules

### Using Topics
- Publish velocity commands to `/turtle1/cmd_vel`
- Subscribe to position updates from `/turtle1/pose`

### Using Services
- Clear drawing with `/clear` service
- Reset turtle position with `/reset` service
- Spawn new turtles with `/spawn` service

### Using Parameters
- Configure movement speeds
- Set field dimensions
- Adjust pattern parameters

## Debugging and Monitoring

### Useful Commands
```bash
# List all topics
ros2 topic list

# Monitor velocity commands
ros2 topic echo /turtle1/cmd_vel

# Monitor turtle position
ros2 topic echo /turtle1/pose

# List available services
ros2 service list

# Clear the drawing
ros2 service call /clear std_srvs/srv/Empty
```

### Visual Debugging
- Watch turtle movement in turtlesim window
- Trace patterns left by turtle's path
- Use different colors for different patterns

## Exercise Ideas

### Beginner Exercises
1. Make turtle move in a square
2. Create a simple back-and-forth pattern
3. Draw a circle using angular velocity

### Intermediate Exercises
1. Implement field row following
2. Create a spiral pattern
3. Navigate to specific coordinates

### Advanced Exercises
1. Combine multiple movement patterns
2. Create interactive pattern selection
3. Implement obstacle avoidance (pretend obstacles)

## Real-World Translation

### From Simulation to Reality
- **Velocity commands** → Motor control signals
- **Position feedback** → GPS coordinates
- **Movement patterns** → Field navigation algorithms
- **Coordinate system** → Real-world GPS coordinates

### Agricultural Applications
- **Tractor automation**: Following precise field patterns
- **Drone surveillance**: Systematic area coverage
- **Robot harvesters**: Efficient crop collection routes
- **Sprayer control**: Ensuring complete coverage

## Next Steps

After completing this module, you understand:
- ✅ How to control robot movement with ROS2
- ✅ Agricultural movement patterns and their applications
- ✅ Coordinate systems and velocity control
- ✅ Visual feedback for movement validation
- ✅ Translation from simulation to real-world robotics

This completes **Day 1** of the RAISE 2025 program! You now have a solid foundation in:
- ROS2 computation graphs and communication
- Publisher/subscriber patterns
- Service/client architecture
- Action servers and clients
- Custom message design
- Robot movement control

Continue to **Day 2: AI and Computer Vision** to add intelligence to your agricultural robots! 