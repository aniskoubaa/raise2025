#!/bin/bash

# RAISE 2025 - Startup Script
echo "🚀 Starting RAISE 2025 Development Environment..."

# Source ROS2 setup
source /opt/ros/humble/setup.bash

# Set up environment variables
export PYTHONPATH=/workspace:$PYTHONPATH
export ROS_DOMAIN_ID=42

# Change to workspace
cd /workspace

# Print welcome message
echo "✅ RAISE 2025 Environment Ready!"
echo "📚 Repository: /workspace"
echo "🤖 ROS2 Humble sourced"
echo "🐍 Python packages installed"
echo ""
echo "🎯 Quick Start Commands:"
echo "  ros2 --version                    # Check ROS2 installation"
echo "  python3 -c 'import torch; print(torch.__version__)'  # Check PyTorch"
echo "  cd Day1_ROS2                      # Start Day 1 tutorials"
echo "  cd Day2_AI_CV                     # Start Day 2 tutorials"
echo ""
echo "📖 Read README.md for detailed instructions"
echo "🤝 Need help? Check the documentation or ask an instructor!"
echo ""

# Start bash shell
exec /bin/bash 