# ROS2 Command Line Cheatsheet
*Using TurtleSim Simulation and Teleop_Key*

## Setup & Launch Commands

### Start TurtleSim
```bash
# Launch turtlesim node
ros2 run turtlesim turtlesim_node

# Launch teleop keyboard control (in separate terminal)
ros2 run turtlesim turtle_teleop_key
```

### Launch Files
```bash
# Note: turtlesim has no built-in launch files
# For multiple turtles, use spawn service instead:
ros2 service call /spawn turtlesim/srv/Spawn "{x: 2.0, y: 2.0, theta: 0.0, name: 'turtle2'}"
```

---

## Node Commands

### List and Info
```bash
# List all running nodes
ros2 node list
```
**Output:**
```
/turtlesim
/teleop_turtle
```

```bash
# Get detailed info about a node
ros2 node info /turtlesim
```
**Output:**
```
/turtlesim
  Subscribers:
    /turtle1/cmd_vel: geometry_msgs/msg/Twist
  Publishers:
    /turtle1/color_sensor: turtlesim/msg/Color
    /turtle1/pose: turtlesim/msg/Pose
  Service Servers:
    /clear: std_srvs/srv/Empty
    /kill: turtlesim/srv/Kill
    /reset: std_srvs/srv/Empty
    /spawn: turtlesim/srv/Spawn
    /turtle1/set_pen: turtlesim/srv/SetPen
    /turtle1/teleport_absolute: turtlesim/srv/TeleportAbsolute
    /turtle1/teleport_relative: turtlesim/srv/TeleportRelative
  Action Servers:
    /turtle1/rotate_absolute: turtlesim/action/RotateAbsolute
```

```bash
# Show node's graph connections
ros2 node info /teleop_turtle
```
**Output:**
```
/teleop_turtle
  Subscribers:
  Publishers:
    /turtle1/cmd_vel: geometry_msgs/msg/Twist
  Service Servers:
  Action Servers:
```

### Kill Nodes
```bash
# Stop nodes with Ctrl+C in their terminals
# Turtlesim nodes are not lifecycle-managed
# Use OS-level kill if needed: kill <PID>
```

---

## Topic Commands

### Basic Topic Operations
```bash
# List all topics
ros2 topic list
```
**Output:**
```
/turtle1/cmd_vel
/turtle1/color_sensor
/turtle1/pose
```

```bash
# List topics with message types
ros2 topic list -t
```
**Output:**
```
/turtle1/cmd_vel [geometry_msgs/msg/Twist]
/turtle1/color_sensor [turtlesim/msg/Color]
/turtle1/pose [turtlesim/msg/Pose]
```

```bash
# Get topic info
ros2 topic info /turtle1/cmd_vel
```
**Output:**
```
Type: geometry_msgs/msg/Twist
Publisher count: 1
Subscription count: 1
```

```bash
ros2 topic info /turtle1/pose
```
**Output:**
```
Type: turtlesim/msg/Pose
Publisher count: 1
Subscription count: 0
```

```bash
# Show topic type
ros2 topic type /turtle1/cmd_vel
```
**Output:**
```
geometry_msgs/msg/Twist
```

### Monitor Topics
```bash
# Echo topic messages
ros2 topic echo /turtle1/pose
```
**Output:**
```
x: 5.544444561004639
y: 5.544444561004639
theta: 0.0
linear_velocity: 0.0
angular_velocity: 0.0
---
x: 5.544444561004639
y: 5.544444561004639
theta: 0.0
linear_velocity: 0.0
angular_velocity: 0.0
---
```

```bash
ros2 topic echo /turtle1/cmd_vel
```
**Output:**
```
linear:
  x: 2.0
  y: 0.0
  z: 0.0
angular:
  x: 0.0
  y: 0.0
  z: 1.8
---
```

```bash
# Show topic frequency
ros2 topic hz /turtle1/pose
```
**Output:**
```
average rate: 62.500
  min: 0.016s max: 0.016s std dev: 0.00000s window: 2
```

```bash
# Show topic bandwidth
ros2 topic bw /turtle1/cmd_vel
```
**Output:**
```
subscribed to [/turtle1/cmd_vel]
average: 1.74KB/s
  mean: 0.04KB min: 0.04KB max: 0.04KB window: 100
```

### Publish to Topics
```bash
# Publish velocity commands (move turtle)
ros2 topic pub /turtle1/cmd_vel geometry_msgs/msg/Twist \
  "linear: {x: 2.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 1.8}"
```
**Output:**
```
publisher: beginning loop
publishing #1: geometry_msgs.msg.Twist(linear=geometry_msgs.msg.Vector3(x=2.0, y=0.0, z=0.0), angular=geometry_msgs.msg.Vector3(x=0.0, y=0.0, z=1.8))
publishing #2: geometry_msgs.msg.Twist(linear=geometry_msgs.msg.Vector3(x=2.0, y=0.0, z=0.0), angular=geometry_msgs.msg.Vector3(x=0.0, y=0.0, z=1.8))
...
```

```bash
# Publish once
ros2 topic pub --once /turtle1/cmd_vel geometry_msgs/msg/Twist \
  "linear: {x: 1.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}"
```
**Output:**
```
publisher: beginning loop
publishing #1: geometry_msgs.msg.Twist(linear=geometry_msgs.msg.Vector3(x=1.0, y=0.0, z=0.0), angular=geometry_msgs.msg.Vector3(x=0.0, y=0.0, z=0.0))
```

```bash
# Publish at specific rate (10 Hz)
ros2 topic pub --rate 10 /turtle1/cmd_vel geometry_msgs/msg/Twist \
  "linear: {x: 1.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}"
```
**Output:**
```
publisher: beginning loop
publishing #1: geometry_msgs.msg.Twist(linear=geometry_msgs.msg.Vector3(x=1.0, y=0.0, z=0.0), angular=geometry_msgs.msg.Vector3(x=0.0, y=0.0, z=0.0))
publishing #2: geometry_msgs.msg.Twist(linear=geometry_msgs.msg.Vector3(x=1.0, y=0.0, z=0.0), angular=geometry_msgs.msg.Vector3(x=0.0, y=0.0, z=0.0))
...
```

```bash
# Publish with wait for subscribers
ros2 topic pub --wait-matching-subscriptions 1 /turtle1/cmd_vel geometry_msgs/msg/Twist \
  "linear: {x: 1.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}"
```
**Output:**
```
waiting for 1 subscribers to connect...
publisher: beginning loop
publishing #1: geometry_msgs.msg.Twist(linear=geometry_msgs.msg.Vector3(x=1.0, y=0.0, z=0.0), angular=geometry_msgs.msg.Vector3(x=0.0, y=0.0, z=0.0))
...
```

---

## Service Commands

### List and Info
```bash
# List all services
ros2 service list
```
**Output:**
```
/clear
/kill
/reset
/spawn
/turtle1/set_pen
/turtle1/teleport_absolute
/turtle1/teleport_relative
```

```bash
# List services with types
ros2 service list -t
```
**Output:**
```
/clear [std_srvs/srv/Empty]
/kill [turtlesim/srv/Kill]
/reset [std_srvs/srv/Empty]
/spawn [turtlesim/srv/Spawn]
/turtle1/set_pen [turtlesim/srv/SetPen]
/turtle1/teleport_absolute [turtlesim/srv/TeleportAbsolute]
/turtle1/teleport_relative [turtlesim/srv/TeleportRelative]
```

```bash
# Get service type
ros2 service type /spawn
```
**Output:**
```
turtlesim/srv/Spawn
```

```bash
ros2 service type /turtle1/set_pen
```
**Output:**
```
turtlesim/srv/SetPen
```

```bash
# Find services by type
ros2 service find turtlesim/srv/Spawn
```
**Output:**
```
/spawn
```

### Call Services
```bash
# Spawn a new turtle
ros2 service call /spawn turtlesim/srv/Spawn \
  "{x: 5.0, y: 5.0, theta: 0.0, name: 'turtle2'}"
```
**Output:**
```
requester: making request: turtlesim.srv.Spawn_Request(x=5.0, y=5.0, theta=0.0, name='turtle2')

response:
turtlesim.srv.Spawn_Response(name='turtle2')
```

```bash
# Change pen color and width
ros2 service call /turtle1/set_pen turtlesim/srv/SetPen \
  "{r: 255, g: 0, b: 0, width: 5, 'off': 0}"
```
**Output:**
```
requester: making request: turtlesim.srv.SetPen_Request(r=255, g=0, b=0, width=5, off=0)

response:
turtlesim.srv.SetPen_Response()
```

```bash
# Turn pen off
ros2 service call /turtle1/set_pen turtlesim/srv/SetPen \
  "{r: 0, g: 0, b: 0, width: 0, 'off': 1}"
```
**Output:**
```
requester: making request: turtlesim.srv.SetPen_Request(r=0, g=0, b=0, width=0, off=1)

response:
turtlesim.srv.SetPen_Response()
```

```bash
# Teleport turtle (absolute)
ros2 service call /turtle1/teleport_absolute turtlesim/srv/TeleportAbsolute \
  "{x: 1.0, y: 1.0, theta: 0.0}"
```
**Output:**
```
requester: making request: turtlesim.srv.TeleportAbsolute_Request(x=1.0, y=1.0, theta=0.0)

response:
turtlesim.srv.TeleportAbsolute_Response()
```

```bash
# Teleport turtle (relative)
ros2 service call /turtle1/teleport_relative turtlesim/srv/TeleportRelative \
  "{linear: 2.0, angular: 1.57}"
```
**Output:**
```
requester: making request: turtlesim.srv.TeleportRelative_Request(linear=2.0, angular=1.57)

response:
turtlesim.srv.TeleportRelative_Response()
```

```bash
# Clear screen
ros2 service call /clear std_srvs/srv/Empty
```
**Output:**
```
requester: making request: std_srvs.srv.Empty_Request()

response:
std_srvs.srv.Empty_Response()
```

---

## Action Commands

### List and Info
```bash
# List all actions
ros2 action list
```
**Output:**
```
/turtle1/rotate_absolute
```

```bash
# List actions with types
ros2 action list -t
```
**Output:**
```
/turtle1/rotate_absolute [turtlesim/action/RotateAbsolute]
```

```bash
# Get action info
ros2 action info /turtle1/rotate_absolute
```
**Output:**
```
Action: /turtle1/rotate_absolute
Action clients: 0
Action servers: 1
```

### Send Action Goals
```bash
# Rotate turtle to absolute angle
ros2 action send_goal /turtle1/rotate_absolute turtlesim/action/RotateAbsolute \
  "{theta: 1.57}"
```
**Output:**
```
Waiting for an action server to become available...
Sending goal:
     theta: 1.57

Goal accepted with ID: 6a9b8c7d-e5f4-3a2b-9c8d-7e6f5a4b3c2d

Result:
    delta: 1.5699999999999998

Goal finished with status: SUCCEEDED
```

```bash
# Rotate with feedback
ros2 action send_goal --feedback /turtle1/rotate_absolute \
  turtlesim/action/RotateAbsolute "{theta: 3.14}"
```
**Output:**
```
Waiting for an action server to become available...
Sending goal:
     theta: 3.14

Goal accepted with ID: 8b7a6c5d-f9e8-2b3c-4d5e-6f7a8b9c0d1e

Feedback:
    remaining: 3.0799999999999996

Feedback:
    remaining: 2.919999999999999

Feedback:
    remaining: 2.759999999999999

...

Result:
    delta: 3.14

Goal finished with status: SUCCEEDED
```

---

## Parameter Commands

### List and Get Parameters
```bash
# List all parameters
ros2 param list
```
**Output:**
```
/turtlesim:
  background_b
  background_g
  background_r
  qos_overrides./parameter_events.publisher.depth
  qos_overrides./parameter_events.publisher.durability
  qos_overrides./parameter_events.publisher.history
  qos_overrides./parameter_events.publisher.reliability
  use_sim_time
```

```bash
# Get parameter value
ros2 param get /turtlesim background_r
```
**Output:**
```
Integer value is: 69
```

```bash
ros2 param get /turtlesim background_g
```
**Output:**
```
Integer value is: 86
```

```bash
ros2 param get /turtlesim background_b
```
**Output:**
```
Integer value is: 255
```

```bash
# Get all parameters from node
ros2 param dump /turtlesim
```
**Output:**
```
/turtlesim:
  ros__parameters:
    background_b: 255
    background_g: 86
    background_r: 69
    qos_overrides:
      /parameter_events:
        publisher:
          depth: 1000
          durability: volatile
          history: keep_last
          reliability: reliable
    use_sim_time: false
```

### Set Parameters
```bash
# Set background color
ros2 param set /turtlesim background_r 255
```
**Output:**
```
Set parameter successful
```

```bash
ros2 param set /turtlesim background_g 0
```
**Output:**
```
Set parameter successful
```

```bash
ros2 param set /turtlesim background_b 0
```
**Output:**
```
Set parameter successful
```

```bash
# Apply background changes (call after setting colors)
ros2 service call /clear std_srvs/srv/Empty
```
**Output:**
```
requester: making request: std_srvs.srv.Empty_Request()

response:
std_srvs.srv.Empty_Response()
```

```bash
# Set multiple parameters
ros2 param set /turtlesim background_r 0
```
**Output:**
```
Set parameter successful
```

```bash
ros2 param set /turtlesim background_g 255
```
**Output:**
```
Set parameter successful
```

```bash
ros2 param set /turtlesim background_b 0
```
**Output:**
```
Set parameter successful
```

### Parameter Files
```bash
# Save parameters to file
ros2 param dump /turtlesim > turtlesim_params.yaml
```
**Output:** (Creates file with content)
```yaml
/turtlesim:
  ros__parameters:
    background_b: 255
    background_g: 86
    background_r: 69
    qos_overrides:
      /parameter_events:
        publisher:
          depth: 1000
          durability: volatile
          history: keep_last
          reliability: reliable
    use_sim_time: false
```

```bash
# Load parameters from file
ros2 param load /turtlesim turtlesim_params.yaml
```
**Output:**
```
Set parameter background_b successful
Set parameter background_g successful
Set parameter background_r successful
Set parameter qos_overrides./parameter_events.publisher.depth successful
Set parameter qos_overrides./parameter_events.publisher.durability successful
Set parameter qos_overrides./parameter_events.publisher.history successful
Set parameter qos_overrides./parameter_events.publisher.reliability successful
Set parameter use_sim_time successful
```

---

## Interface Commands

### Message Interfaces
```bash
# List all message types
ros2 interface list --msgs
```
**Output:** (truncated)
```
geometry_msgs/msg/Twist
geometry_msgs/msg/Vector3
sensor_msgs/msg/Image
std_msgs/msg/String
turtlesim/msg/Color
turtlesim/msg/Pose
...
```

```bash
# Show message definition
ros2 interface show geometry_msgs/msg/Twist
```
**Output:**
```
geometry_msgs/Vector3 linear
        float64 x
        float64 y
        float64 z
geometry_msgs/Vector3 angular
        float64 x
        float64 y
        float64 z
```

```bash
ros2 interface show turtlesim/msg/Pose
```
**Output:**
```
float32 x
float32 y
float32 theta
float32 linear_velocity
float32 angular_velocity
```

```bash
# Show message prototype
ros2 interface proto geometry_msgs/msg/Twist
```
**Output:**
```
linear:
  x: 0.0
  y: 0.0
  z: 0.0
angular:
  x: 0.0
  y: 0.0
  z: 0.0
```

### Service Interfaces
```bash
# List all service types
ros2 interface list --srvs
```
**Output:** (truncated)
```
std_srvs/srv/Empty
std_srvs/srv/SetBool
std_srvs/srv/Trigger
turtlesim/srv/Kill
turtlesim/srv/SetPen
turtlesim/srv/Spawn
turtlesim/srv/TeleportAbsolute
turtlesim/srv/TeleportRelative
...
```

```bash
# Show service definition
ros2 interface show turtlesim/srv/Spawn
```
**Output:**
```
float32 x
float32 y
float32 theta
string name
---
string name
```

```bash
ros2 interface show turtlesim/srv/SetPen
```
**Output:**
```
uint8 r
uint8 g
uint8 b
uint8 width
uint8 off
---
```

```bash
ros2 interface show turtlesim/srv/TeleportAbsolute
```
**Output:**
```
float32 x
float32 y
float32 theta
---
```

### Action Interfaces
```bash
# List all action types
ros2 interface list --actions
```
**Output:**
```
turtlesim/action/RotateAbsolute
```

```bash
# Show action definition
ros2 interface show turtlesim/action/RotateAbsolute
```
**Output:**
```
# The desired heading in radians
float32 theta
---
# The angular displacement in radians to the starting position
float32 delta
---
# The remaining rotation in radians
float32 remaining
```

---

## Debugging & Monitoring

### RQt Tools
```bash
# Launch RQt (GUI tools)
rqt

# RQt graph (visualize nodes and topics)
rqt_graph

# RQt console (view log messages)
rqt_console

# RQt plot (plot topic data)
rqt_plot /turtle1/pose/x /turtle1/pose/y
```

### Logging
```bash
# View logs with rqt_console (GUI)
rqt_console

# Echo system logs
ros2 topic echo /rosout

# Note: ros2 log commands are not standard ROS2 CLI
# Logging levels are set in node code or via parameters
```

### System Info
```bash
# Check ROS2 environment
ros2 doctor
```
**Output:**
```
Checking ROS2 environment...
All systems go!
```
*Note: Actual output is more detailed and reports specific checks*

```bash
# List packages
ros2 pkg list
```
**Output:** (truncated)
```
action_msgs
builtin_interfaces
geometry_msgs
rclcpp
rclpy
std_msgs
turtlesim
...
```

```bash
# Find package
ros2 pkg prefix turtlesim
```
**Output:**
```
/opt/ros/jazzy
```
*Note: Output varies by ROS2 distribution (e.g., /opt/ros/humble for Humble)*

---

## Recording & Playback

### Bag Files
```bash
# Record topics
ros2 bag record /turtle1/cmd_vel /turtle1/pose
```
**Output:**
```
[INFO] [1640995200.123456789] [rosbag2_recorder]: Listening for topics...
[INFO] [1640995200.234567890] [rosbag2_recorder]: All requested topics are subscribed. Stopping discovery...
[INFO] [1640995200.345678901] [rosbag2_recorder]: Recording...
```

```bash
# Record specific topics
ros2 bag record -o my_bag /turtle1/cmd_vel
```
**Output:**
```
[INFO] [1640995200.123456789] [rosbag2_recorder]: Listening for topics...
[INFO] [1640995200.234567890] [rosbag2_recorder]: All requested topics are subscribed. Stopping discovery...
[INFO] [1640995200.345678901] [rosbag2_recorder]: Recording...
```

```bash
# Play back recorded data
ros2 bag play my_bag
```
**Output:**
```
[INFO] [1640995200.456789012] [rosbag2_player]: Opening bag 'my_bag'...
[INFO] [1640995200.567890123] [rosbag2_player]: Starting playback...
[INFO] [1640995200.678901234] [rosbag2_player]: Playback finished
```

```bash
# Get bag info
ros2 bag info my_bag
```
**Output:**
```
Files:             my_bag_0.db3
Bag size:          224.5 KiB
Storage id:        sqlite3
Duration:          20.103s
Start:             Dec 31 2021 18:00:00.123 (1640995200.123456789)
End:               Dec 31 2021 18:00:20.226 (1640995220.226456789)
Messages:          404
Topic information: Topic: /turtle1/cmd_vel | Type: geometry_msgs/msg/Twist | Count: 202 | Serialization Format: cdr
```

```bash
# Record all topics
ros2 bag record -a
```
**Output:**
```
[INFO] [1640995200.123456789] [rosbag2_recorder]: Listening for topics...
[INFO] [1640995200.234567890] [rosbag2_recorder]: All requested topics are subscribed. Stopping discovery...
[INFO] [1640995200.345678901] [rosbag2_recorder]: Recording...
```

```bash
# Play with rate control
ros2 bag play my_bag --rate 0.5
```
**Output:**
```
[INFO] [1640995200.456789012] [rosbag2_player]: Opening bag 'my_bag'...
[INFO] [1640995200.567890123] [rosbag2_player]: Starting playback at 0.5x speed...
[INFO] [1640995200.678901234] [rosbag2_player]: Playback finished
```

---

## Common Message Formats

### Twist (cmd_vel)
```yaml
linear:
  x: 2.0    # forward/backward
  y: 0.0    # left/right (usually 0 for differential drive)
  z: 0.0    # up/down (usually 0 for ground robots)
angular:
  x: 0.0    # roll (usually 0)
  y: 0.0    # pitch (usually 0)
  z: 1.8    # yaw (turning)
```

### Pose (turtle position)
```yaml
x: 5.544444561004639        # x position
y: 5.544444561004639        # y position
theta: 0.0                  # orientation (radians)
linear_velocity: 0.0        # current linear velocity
angular_velocity: 0.0       # current angular velocity
```

---

## Quick Reference

### Essential TurtleSim Commands
```bash
# Start simulation
ros2 run turtlesim turtlesim_node

# Start teleop
ros2 run turtlesim turtle_teleop_key

# Control turtle2 after spawning (remapping)
ros2 run turtlesim turtle_teleop_key --ros-args -r turtle1/cmd_vel:=turtle2/cmd_vel

# Move turtle programmatically
ros2 topic pub /turtle1/cmd_vel geometry_msgs/msg/Twist \
  "linear: {x: 2.0}, angular: {z: 1.8}"

# Spawn new turtle
ros2 service call /spawn turtlesim/srv/Spawn \
  "{x: 2.0, y: 2.0, theta: 0.0, name: 'turtle2'}"

# Change pen color
ros2 service call /turtle1/set_pen turtlesim/srv/SetPen \
  "{r: 255, g: 0, b: 0, width: 3}"

# Watch turtle position
ros2 topic echo /turtle1/pose
```

### Keyboard Controls (teleop_key)
- **Arrow keys**: Move turtle
- **Space**: Stop turtle
- **q/w**: Increase/decrease linear speed
- **a/s**: Increase/decrease angular speed