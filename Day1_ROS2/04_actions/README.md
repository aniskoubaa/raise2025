# Module 4: ROS2 Actions

## Learning Objectives
By the end of this module, you will be able to:
- Understand what ROS2 actions are and when to use them
- Create action servers for long-running tasks
- Build action clients that can cancel and monitor tasks
- Apply actions to agricultural robotics scenarios

## Agricultural Context: Autonomous Field Navigation
In agricultural robotics, many tasks take time and need monitoring:
- **Field Navigation**: Robot moving between crop rows
- **Harvest Collection**: Picking fruits along a route
- **Field Inspection**: Scanning crops for health monitoring
- **Precision Spraying**: Applying treatments along planned paths

## Actions vs Services vs Topics

### Topics (Continuous Data)
- **Use for**: Sensor data, status updates
- **Pattern**: Publish/Subscribe
- **Example**: GPS position updates

### Services (Quick Operations)
- **Use for**: Simple requests (< 1-2 seconds)
- **Pattern**: Request/Response
- **Example**: Check if robot is ready

### Actions (Long Tasks)
- **Use for**: Long-running operations (seconds to minutes)
- **Pattern**: Goal/Feedback/Result
- **Example**: Navigate to field coordinates

## Action Components

### 1. Goal
What you want the robot to do:
```
target_x: 10.5
target_y: 25.0
max_speed: 1.0
```

### 2. Feedback (During execution)
Progress updates while running:
```
current_x: 5.2
current_y: 12.1
distance_remaining: 15.3
estimated_time: 8.5
```

### 3. Result (When finished)
Final outcome:
```
success: true
final_x: 10.5
final_y: 25.0
total_distance: 20.1
total_time: 18.2
```

## Simple Example: Field Navigation

### Action Definition
```
# Goal: Where to go
float64 target_x
float64 target_y
float64 max_speed
---
# Result: What happened
bool success
string message
float64 final_x
float64 final_y
float64 total_distance
float64 total_time
---
# Feedback: Progress updates
float64 current_x
float64 current_y
float64 distance_remaining
float64 estimated_time
string status
```

## Files in this Module

1. **`field_navigator_server.py`** - Action server that simulates robot navigation
2. **`field_navigator_client.py`** - Simple client to send navigation goals
3. **`harvest_action_server.py`** - Example of harvesting action
4. **`harvest_client.py`** - Client for harvest operations
5. **`QUICKSTART.md`** - Step-by-step guide

## Key Concepts

### Action Server
- **Receives goals** from clients
- **Executes long tasks** (navigation, harvesting)
- **Sends feedback** during execution
- **Returns results** when done
- **Can be cancelled** by clients

### Action Client
- **Sends goals** to servers
- **Receives feedback** during execution
- **Gets final results**
- **Can cancel** running actions

## Real-World Applications

### Autonomous Farming
- **Field mapping**: Robot scans entire field
- **Precision planting**: Plant seeds at exact locations
- **Selective harvesting**: Pick only ripe fruits
- **Pest control**: Spray only affected areas

### Benefits of Actions
- **Cancellation**: Stop robot if weather changes
- **Monitoring**: Track progress in real-time
- **Feedback**: Know where robot is and what it's doing
- **Fault handling**: Detect and respond to problems

## Common Patterns

### Send Goal and Wait
```python
# Send a goal and wait for completion
goal = FieldNavigate.Goal()
goal.target_x = 10.0
goal.target_y = 20.0
goal.max_speed = 1.5

future = client.send_goal_async(goal)
result = await future
```

### Send Goal with Feedback
```python
# Monitor progress during execution
def feedback_callback(feedback_msg):
    feedback = feedback_msg.feedback
    print(f"Position: ({feedback.current_x}, {feedback.current_y})")
    print(f"Remaining: {feedback.distance_remaining}m")

goal_handle = await client.send_goal_async(
    goal, 
    feedback_callback=feedback_callback
)
```

### Cancel Action
```python
# Cancel if needed
if need_to_stop:
    await goal_handle.cancel_goal_async()
```

## Error Handling

### Server-Side
- Validate goal parameters
- Handle hardware failures
- Provide meaningful feedback
- Set appropriate timeouts

### Client-Side
- Check if action server is available
- Handle goal rejection
- Monitor for cancellation
- Process results appropriately

## Best Practices

### Action Design
1. **Make goals specific**: Include all necessary parameters
2. **Provide useful feedback**: Position, progress, status
3. **Handle cancellation**: Stop safely when cancelled
4. **Return meaningful results**: Success/failure with details

### Client Usage
1. **Check server availability**: Verify action server is running
2. **Validate goals**: Ensure parameters make sense
3. **Monitor feedback**: Track progress for user interface
4. **Handle failures**: Respond appropriately to errors

## Integration with Agricultural Systems

### With Topics
- Subscribe to GPS for current position
- Publish robot status to monitoring systems
- Use sensor data for navigation decisions

### With Services
- Call services to check robot readiness
- Use services for quick configuration changes
- Integrate with safety systems

### With Parameters
- Configure navigation speeds and tolerances
- Set field boundaries and restricted areas
- Adjust behavior for different crops

## Troubleshooting

### Common Issues

1. **Action server not found**
   - Check if server is running
   - Verify action name spelling
   - Use `ros2 action list` to see available actions

2. **Goals rejected**
   - Check goal parameter validity
   - Verify robot is in correct state
   - Check for safety constraints

3. **Slow feedback**
   - Reduce feedback frequency in server
   - Check network connectivity
   - Optimize feedback message size

### Debug Commands
```bash
# List available actions
ros2 action list

# Show action interface
ros2 action info /field_navigate

# Send test goal
ros2 action send_goal /field_navigate example_interfaces/action/FieldNavigate "{target_x: 5.0, target_y: 10.0, max_speed: 1.0}"

# Monitor action feedback
ros2 action send_goal /field_navigate example_interfaces/action/FieldNavigate "{target_x: 5.0, target_y: 10.0, max_speed: 1.0}" --feedback
```

## Next Steps
After completing this module, you'll understand:
- ✅ When to use actions vs services vs topics
- ✅ How to create action servers and clients
- ✅ How to handle long-running tasks
- ✅ How to implement cancellation and feedback
- ✅ Real-world agricultural robotics applications

Continue to **Module 5: Custom Messages** to learn about creating your own message types! 