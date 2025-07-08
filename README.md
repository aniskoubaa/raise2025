# RAISE 2025 - International Summer School on Robotics & AI in Systems Engineering

## 🤖 AI-Empowered Robotics for Smart Agriculture

Welcome to the RAISE 2025 hands-on laboratory materials! This repository contains all the code, tutorials, and exercises for our 3-day intensive program on robotics and AI for smart agriculture.

## 📚 Repository Structure

```
RAISE2025/
│
├── README.md                    # This file
├── Day1_ROS2/                   # ROS2 Foundations
│   ├── 01_computation_graph/    # Understanding ROS2 nodes and topics
│   ├── 02_publish_subscribe/    # Publisher/Subscriber pattern
│   ├── 03_service_client/       # Service/Client communication
│   ├── 04_actions/              # Action servers and clients
│   ├── 05_messages/             # Custom message types
│   ├── 06_turtlesim_move/       # Integrated turtle control
│   └── practice_tests/          # Assessment materials
│
├── Day2_AI_CV/                  # AI & Computer Vision
│   ├── 01_yolo_basics/          # Object detection with YOLO
│   ├── 02_train_tomato_dataset/ # Training on agricultural data
│   ├── 03_gemini_vision_api/    # LLM integration for plant diagnostics
│   ├── 04_ros2_cv_integration/  # Bridging AI and ROS2
│   └── practice_tests/          # Assessment materials
│
├── Day3_Challenge/              # Integrated competition
│   ├── challenge_instructions.md
│   ├── starter_code/
│   └── submission_guidelines.md
│
└── ExamGPT_Quizzes/            # Auto-graded assessments
    ├── quiz1_day1.md
    └── quiz2_day2.md
```

## 🚀 Quick Start Guide

### Prerequisites

Before starting, ensure you have:
- **Ubuntu 22.04 LTS** (recommended) or WSL2 on Windows
- **Docker** installed and running
- **Git** installed
- At least **20 GB** free disk space
- Basic knowledge of **Python** and **linear algebra**

### 1. Clone the Repository

```bash
git clone https://github.com/raiseschool/RAISE2025.git
cd RAISE2025
```

### 2. Environment Setup

We provide multiple setup options:

#### Option A: Docker Setup (Recommended for beginners)
```bash
# Build the Docker environment
docker build -t raise2025 .

# Run the container
docker run -it --rm -v $(pwd):/workspace raise2025
```

#### Option B: Local Installation
```bash
# Install ROS2 Humble
sudo apt update
sudo apt install software-properties-common
sudo add-apt-repository universe
sudo apt update && sudo apt install curl gnupg lsb-release
curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(source /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
sudo apt update
sudo apt install ros-humble-desktop

# Install additional dependencies
sudo apt install python3-pip python3-colcon-common-extensions
pip3 install opencv-python torch torchvision ultralytics google-generativeai
```

#### Option C: Development Container (VS Code)
If you're using VS Code, simply open the repository and it will prompt you to reopen in a container.

### 3. Verify Installation

```bash
# Check ROS2 installation
ros2 --version

# Check Python packages
python3 -c "import cv2, torch, ultralytics; print('All packages installed successfully!')"
```

## 📖 Learning Path

### Day 1: ROS2 Foundations (9 hours)
**Goal:** Master ROS2 basics and robot control

1. **[Computation Graph](Day1_ROS2/01_computation_graph/)** - Understanding nodes, topics, services
2. **[Publish/Subscribe](Day1_ROS2/02_publish_subscribe/)** - Communication patterns
3. **[Service/Client](Day1_ROS2/03_service_client/)** - Synchronous communication
4. **[Actions](Day1_ROS2/04_actions/)** - Long-running tasks
5. **[Messages](Day1_ROS2/05_messages/)** - Custom data types
6. **[Turtlesim Move](Day1_ROS2/06_turtlesim_move/)** - Integrated robot control

**Assessment:** ExamGPT Quiz #1 (70% required for Level-1 Certificate)

### Day 2: AI & Computer Vision (9 hours)
**Goal:** Apply AI for plant diagnostics

1. **[YOLO Basics](Day2_AI_CV/01_yolo_basics/)** - Object detection fundamentals
2. **[Tomato Dataset Training](Day2_AI_CV/02_train_tomato_dataset/)** - Agricultural AI
3. **[Gemini Vision API](Day2_AI_CV/03_gemini_vision_api/)** - LLM integration
4. **[ROS2 CV Integration](Day2_AI_CV/04_ros2_cv_integration/)** - Bridging AI and robotics

**Assessment:** ExamGPT Quiz #2 (70% required for Level-2 Certificate)

### Day 3: Integrated Challenge (8 hours)
**Goal:** Build and deploy a complete agri-robot solution

Teams compete to create the best AI-powered agricultural robot using knowledge from Days 1-2.

## 🎯 Learning Objectives

By completing this program, you will:

✅ **Deploy ROS2 stacks** for mobile manipulation and navigation  
✅ **Implement YOLO-based vision pipelines** for crop analysis  
✅ **Integrate LLMs** for multimodal reasoning in robotics  
✅ **Prototype end-to-end agri-robot solutions** efficiently  
✅ **Collaborate cross-disciplinarily** under time pressure  

## 🔧 Common Issues & Solutions

### Docker Issues
```bash
# If Docker permission denied
sudo usermod -aG docker $USER
# Then logout and login again
```

### ROS2 Issues
```bash
# If ROS2 commands not found
source /opt/ros/humble/setup.bash
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
```

### Python Package Issues
```bash
# If import errors
pip3 install --upgrade pip
pip3 install -r requirements.txt
```

## 📊 Assessment & Certification

### ExamGPT Quizzes
- **Quiz 1 (Day 1):** ROS2 concepts and programming
- **Quiz 2 (Day 2):** AI/CV concepts and integration
- **Auto-graded** with immediate feedback
- **70% threshold** for certification

### Certificates
- **Certificate of Completion:** ≥70% on both quizzes + active challenge participation
- **Certificate of Participation:** All other attendees

## 🤝 Getting Help

### During the Summer School
- **Instructors:** Available during lab sessions
- **Slack Channel:** #raise2025-help
- **Peer Support:** Work in teams, help each other!

### After the Summer School
- **GitHub Issues:** Report bugs or request features
- **Community Forum:** Continue discussions
- **Documentation:** All materials remain available

## 🏆 Competition Guidelines

### Day 3 Challenge Rules
1. **Teams of 3-4 students** (mixed tracks encouraged)
2. **6-hour development time** with provided hardware
3. **Judging criteria:** Technical implementation, innovation, presentation
4. **Prizes:** Recognition and networking opportunities

### Submission Requirements
- **Code repository** with clear documentation
- **Video demonstration** (3-5 minutes)
- **Technical report** (max 2 pages)
- **Live presentation** (10 minutes + Q&A)

## 🌟 Tips for Success

### For Beginners
1. **Start early** - Read through materials before each day
2. **Ask questions** - No question is too basic
3. **Practice regularly** - Run code examples multiple times
4. **Collaborate** - Learn from your peers
5. **Document** - Keep notes of what you learn

### For Advanced Students
1. **Extend examples** - Add your own features
2. **Help others** - Teaching reinforces learning
3. **Experiment** - Try different approaches
4. **Optimize** - Improve performance and efficiency
5. **Connect concepts** - Link theory to practice

## 📞 Contact Information

- **Website:** [raiseschool.github.io](https://raiseschool.github.io)
- **Email:** info@raiseschool.org
- **Social:** Follow us on Twitter @RAISESchool

## 📄 License

This repository is licensed under the MIT License. See [LICENSE](LICENSE) for details.

---

**Welcome to RAISE 2025! Let's build the future of agricultural robotics together! 🚀🌱** 