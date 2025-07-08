# Day 1: ROS2 Agricultural Robotics Fundamentals

## 🎉 Completion Status: COMPLETED ✅

Welcome to Day 1 of the RAISE 2025 International Summer School! This day covers the fundamentals of ROS2 (Robot Operating System 2) with a focus on agricultural robotics applications.

## 🎯 Learning Objectives

By the end of Day 1, you will be able to:
- ✅ Understand ROS2 computation graph concepts (nodes, topics, services, actions)
- ✅ Create and run ROS2 nodes using Python
- ✅ Implement publisher/subscriber communication patterns
- ✅ Use services for synchronous communication
- ✅ Work with actions for long-running tasks
- ✅ Create and use custom message types
- ✅ Control a robot using ROS2 commands

## 📅 Schedule (9:00 - 17:00)

### Morning Session (9:00 - 12:00)
- **9:00-9:30:** Introduction to ROS2 and Agricultural Robotics
- **9:30-10:15:** Module 1 - Computation Graph
- **10:15-10:30:** ☕ Break
- **10:30-11:15:** Module 2 - Publish/Subscribe
- **11:15-12:00:** Module 3 - Service/Client

### Afternoon Session (13:00 - 16:00)
- **13:00-13:45:** Module 4 - Actions
- **13:45-14:30:** Module 5 - Custom Messages
- **14:30-14:45:** ☕ Break
- **14:45-16:00:** Module 6 - Turtlesim Integration

### Assessment (16:15 - 17:00)
- **16:15-17:00:** ExamGPT Quiz #1 (70% required for Level-1 Certificate)

## 🤖 What is ROS2?

**ROS2 (Robot Operating System 2)** is a middleware framework for building robot applications. Think of it as a "communication system" that allows different parts of a robot to talk to each other.

### Why ROS2 for Agricultural Robots?
- **Modularity:** Separate components (vision, navigation, control) can work independently
- **Scalability:** Easy to add new sensors or capabilities
- **Reliability:** Built-in fault tolerance and error handling
- **Community:** Large ecosystem of agricultural robotics packages

## 🏗️ ROS2 Architecture Overview

```
┌─────────────────────────────────────────────────────────────┐
│                    ROS2 Computation Graph                   │
├─────────────────────────────────────────────────────────────┤
│  ┌─────────┐    Topics    ┌─────────┐    Services   ┌─────────┐  │
│  │  Node   │ ←----------→ │  Node   │ ←----------→ │  Node   │  │
│  │ Camera  │              │ Vision  │              │ Control │  │
│  └─────────┘              └─────────┘              └─────────┘  │
│       │                        │                        │       │
│       │                        │                        │       │
│       └────────────────────────┼────────────────────────┘       │
│                                │                                │
│                         ┌─────────┐                             │
│                         │  Node   │                             │
│                         │ Logger  │                             │
│                         └─────────┘                             │
└─────────────────────────────────────────────────────────────────┘
```

## 📚 Module Overview

### [Module 1: Computation Graph](01_computation_graph/)
**Duration:** 45 minutes  
**Concepts:** Nodes, topics, services, actions, parameters  
**Practice:** Creating your first ROS2 node

### [Module 2: Publish/Subscribe](02_publish_subscribe/)
**Duration:** 45 minutes  
**Concepts:** Publisher/subscriber pattern, message types  
**Practice:** Publishing robot velocity commands

### [Module 3: Service/Client](03_service_client/)
**Duration:** 45 minutes  
**Concepts:** Synchronous communication, request/response  
**Practice:** Calling robot services

### [Module 4: Actions](04_actions/)
**Duration:** 45 minutes  
**Concepts:** Long-running tasks, goal/feedback/result  
**Practice:** Implementing navigation goals

### [Module 5: Custom Messages](05_messages/)
**Duration:** 45 minutes  
**Concepts:** Message definition, compilation, usage  
**Practice:** Creating agricultural sensor messages

### [Module 6: Turtlesim Integration](06_turtlesim_move/)
**Duration:** 75 minutes  
**Concepts:** Bringing it all together  
**Practice:** Autonomous greenhouse navigation

## 🚀 Getting Started

### Prerequisites Check
```bash
# Check ROS2 installation
ros2 --version

# Check Python
python3 --version

# Check colcon build tool
colcon --version
```

### Environment Setup
```bash
# Source ROS2 environment
source /opt/ros/humble/setup.bash

# Create workspace
mkdir -p ~/raise2025_ws/src
cd ~/raise2025_ws

# Clone our repository
git clone https://github.com/raiseschool/RAISE2025.git src/RAISE2025
```

### Verify Setup
```bash
# Build workspace
colcon build

# Source workspace
source install/setup.bash

# Test with turtlesim
ros2 run turtlesim turtlesim_node
```

## 💡 Learning Tips

### For Complete Beginners
1. **Take your time** - Don't rush through the concepts
2. **Type the code** - Don't just copy-paste, understand each line
3. **Ask questions** - Instructors are here to help
4. **Practice** - Run each example multiple times
5. **Experiment** - Try changing parameters and see what happens

### For Intermediate Students
1. **Focus on integration** - How do modules work together?
2. **Think about applications** - How would you use this in agriculture?
3. **Help others** - Teaching reinforces your own learning
4. **Extend examples** - Add your own features
5. **Read documentation** - Explore ROS2 docs for deeper understanding

## 🌱 Agricultural Context

Throughout Day 1, we'll use examples relevant to agricultural robotics:
- **Crop monitoring robots** navigating between plant rows
- **Sensor data** from soil moisture and plant health sensors
- **Autonomous actions** like irrigation and pest control
- **Multi-robot coordination** for large-scale farming

## 🔍 Assessment Criteria

The ExamGPT Quiz will test:
- **Conceptual understanding** (40%): ROS2 architecture, communication patterns
- **Code comprehension** (30%): Reading and understanding ROS2 code
- **Practical application** (30%): Writing simple ROS2 nodes and launch files

## 🆘 Need Help?

### During the Lab
- **Slack:** #raise2025-day1
- **Instructors:** Available throughout the session
- **Peer support:** Work with your neighbors

### Debugging Tips
```bash
# Check running nodes
ros2 node list

# Check topics
ros2 topic list

# Check services
ros2 service list

# Monitor topic data
ros2 topic echo /topic_name

# Check node info
ros2 node info /node_name
```

### Common Issues
1. **"ros2 command not found"** → Source ROS2: `source /opt/ros/humble/setup.bash`
2. **"No module named 'rclpy'"** → Install ROS2 Python: `sudo apt install python3-rclpy`
3. **"Permission denied"** → Check file permissions: `chmod +x script.py`

## 🎯 Success Metrics

By the end of Day 1, you should be able to:
- [ ] Explain the ROS2 computation graph
- [ ] Create a simple publisher node
- [ ] Create a simple subscriber node
- [ ] Call a service from the command line
- [ ] Implement a basic action client
- [ ] Define a custom message type
- [ ] Control turtlesim robot autonomously
- [ ] Pass the ExamGPT Quiz with ≥70%

---

**Ready to start? Let's begin with [Module 1: Computation Graph](01_computation_graph/)!** 🚀 