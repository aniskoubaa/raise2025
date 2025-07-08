# Module 4 Quickstart Guide: ROS2 Actions

## Overview
This guide shows you how to use ROS2 actions with a simple field navigation example. You'll learn how to create action servers for long-running tasks and clients that can monitor progress and cancel tasks.

## Prerequisites
- ROS2 installed and sourced
- Python 3.7+
- Basic understanding of ROS2 concepts from previous modules

## What are Actions?
Actions are for **long-running tasks** that:
- Take time to complete (seconds to minutes)
- Provide progress feedback while running
- Can be cancelled if needed
- Return a final result

## Quick Start (3-minute demo)

### Terminal 1: Start Action Server
```bash
cd RAISE2025/Day1_ROS2/04_actions
python3 field_navigator_server.py
```

**Expected Output:**
```
🤖 Field Navigator Server started
📡 Ready to receive navigation goals at: /field_navigate
🤖 Starting field navigation server...
```

### Terminal 2: Send Navigation Goal
```bash
cd RAISE2025/Day1_ROS2/04_actions
python3 field_navigator_client.py 5
```

**Expected Output:**
```
🎮 Field Navigator Client started
⏳ Waiting for field navigator server...
✅ Connected to field navigator!
🎯 Sending goal: Navigate 5 meters
✅ Goal accepted by server
📊 Progress: Step 1 | Latest value: 0
📊 Progress: Step 2 | Latest value: 1
📊 Progress: Step 3 | Latest value: 1
📊 Progress: Step 4 | Latest value: 2
📊 Progress: Step 5 | Latest value: 3
✅ Navigation completed! Total steps: 5
```

## Detailed Examples

### Example 1: Basic Navigation
```bash
# Navigate 3 meters
python3 field_navigator_client.py 3

# Navigate 7 meters  
python3 field_navigator_client.py 7
```

### Example 2: Navigation Sequence
```bash
# Run a sequence of navigation goals
python3 field_navigator_client.py sequence
```

This will send multiple goals automatically:
- Navigate 3 meters
- Navigate 5 meters  
- Navigate 2 meters
- Navigate 4 meters

### Example 3: Interactive Mode
```bash
python3 field_navigator_client.py interactive
```

This starts an interactive session where you can:
- Enter distances to navigate (e.g., `5`)
- Cancel current goal with `c`
- Quit with `q`

## Understanding the Output

### Server Messages
- `🎯 Received navigation goal` - Server got a new goal
- `🚜 Starting navigation to distance: 5m` - Beginning to execute
- `📍 Position: (2.5, 1.5) | Distance: 5/5m` - Progress updates
- `✅ Navigation completed!` - Goal finished successfully
- `🛑 Navigation cancelled` - Goal was cancelled

### Client Messages
- `✅ Goal accepted by server` - Server will execute the goal
- `📊 Progress: Step 3` - Feedback from server
- `✅ Navigation completed!` - Goal finished
- `🛑 Navigation was cancelled` - Goal was stopped

## Monitoring with ROS2 Commands

### Check Available Actions
```bash
ros2 action list
```

**Expected Output:**
```
/field_navigate
```

### Send Goal from Command Line
```bash
ros2 action send_goal /field_navigate example_interfaces/action/Fibonacci "{order: 4}"
```

### Send Goal with Feedback
```bash
ros2 action send_goal /field_navigate example_interfaces/action/Fibonacci "{order: 6}" --feedback
```

### Check Action Info
```bash
ros2 action info /field_navigate
```

## Key Concepts Demonstrated

### 1. Action Server
- **Receives goals** from clients
- **Executes long tasks** step by step
- **Sends progress feedback** regularly
- **Returns final results** when done
- **Handles cancellation** if requested

### 2. Action Client
- **Sends goals** to servers
- **Receives feedback** during execution
- **Gets final results** when complete
- **Can cancel** running actions

### 3. Goal/Feedback/Result Pattern
- **Goal**: What you want (navigate 5 meters)
- **Feedback**: Progress updates (current position)
- **Result**: Final outcome (success/failure)

## Exercise Challenges

### Challenge 1: Basic Action Usage
1. Start the field navigator server
2. Send a navigation goal for 4 meters
3. Watch the progress feedback
4. Observe the final result

### Challenge 2: Multiple Goals
1. Send a goal for 3 meters
2. Wait for completion
3. Send another goal for 6 meters
4. Compare the execution times

### Challenge 3: Interactive Mode
1. Start interactive mode
2. Send various navigation distances
3. Try cancelling a goal mid-execution
4. Send multiple goals in sequence

### Challenge 4: Monitor Status
1. Start the server
2. In another terminal, monitor robot status:
   ```bash
   ros2 topic echo /robot_status
   ```
3. Send navigation goals and watch status updates

## Common Issues and Solutions

### Issue 1: Action Server Not Found
**Problem:** Client can't find the action server

**Solution:**
```bash
# Check if server is running
ros2 action list

# Make sure field_navigator_server.py is running
```

### Issue 2: No Feedback
**Problem:** Not seeing progress updates

**Solution:**
- Server might be running too fast
- Check the server logs for progress messages
- Increase navigation distance for longer execution

### Issue 3: Goal Rejected
**Problem:** Server rejects the goal

**Solution:**
- Check if distance is positive (> 0)
- Verify goal format is correct
- Check server logs for rejection reason

## Real-World Applications

### Agricultural Robotics
- **Field mapping**: Robot surveys entire field
- **Crop inspection**: Move between plant rows
- **Precision spraying**: Follow planned treatment paths
- **Harvest collection**: Navigate to ripe fruit locations

### Why Actions are Perfect for This
- **Long duration**: Field navigation takes time
- **Progress tracking**: Know where robot is
- **Cancellation**: Stop if weather changes
- **Status updates**: Monitor for safety

## Next Steps

After completing this module, you understand:
- ✅ When to use actions vs services vs topics
- ✅ How to create action servers for long tasks
- ✅ How to build clients that monitor progress
- ✅ How to handle cancellation and feedback
- ✅ Real-world agricultural applications

Continue to **Module 5: Custom Messages** to learn about creating your own message types!

## Tips for Success

1. **Start Simple**: Begin with basic goals before trying complex scenarios
2. **Watch Feedback**: Pay attention to progress updates
3. **Try Cancellation**: Understand how to stop long-running tasks
4. **Monitor Status**: Use topic echo to see status updates
5. **Experiment**: Try different distances and sequences

## Advanced Exploration

### Custom Action Types
In real applications, you would create custom action types:
```
# NavigateToPoint.action
geometry_msgs/Point target_position
float64 max_speed
---
bool success
string message
geometry_msgs/Point final_position
float64 distance_traveled
float64 time_taken
---
geometry_msgs/Point current_position
float64 distance_remaining
float64 estimated_time
string current_status
```

### Multiple Action Servers
Complex robots often have multiple action servers:
- Navigation actions
- Manipulation actions  
- Perception actions
- Planning actions

### Action Composition
Actions can coordinate with each other:
- Navigation action calls planning service
- Harvest action uses navigation to reach targets
- Inspection action coordinates camera and movement 