# Day 2: AI & Computer Vision for Plant Diagnostics

## 🎯 Learning Objectives

By the end of Day 2, you will be able to:
- ✅ Understand computer vision fundamentals and applications in agriculture
- ✅ Use YOLO for real-time plant disease detection
- ✅ Train and fine-tune AI models on agricultural datasets
- ✅ Integrate Large Language Models (LLMs) for plant health analysis
- ✅ Bridge AI/CV systems with ROS2 for robotic applications
- ✅ Create end-to-end AI-powered agricultural diagnostic systems

## 📅 Schedule (9:00 - 17:00)

### Morning Session (9:00 - 12:00)
- **9:00-9:30:** Introduction to AI in Agricultural Robotics
- **9:30-10:15:** Module 1 - YOLO Basics & Object Detection
- **10:15-10:30:** ☕ Break
- **10:30-11:15:** Module 2 - Training on Tomato Disease Dataset
- **11:15-12:00:** Module 3 - Gemini Vision API Integration

### Afternoon Session (13:00 - 16:00)
- **13:00-13:45:** Module 4 - ROS2 Computer Vision Integration
- **13:45-14:30:** Advanced Integration Techniques
- **14:30-14:45:** ☕ Break
- **14:45-16:00:** Complete Integration Project

### Assessment (16:15 - 17:00)
- **16:15-17:00:** ExamGPT Quiz #2 (70% required for Level-2 Certificate)

## 🌱 Why AI in Agriculture?

Agricultural robotics with AI can revolutionize farming by:

### 🔍 **Precision Detection**
- **Disease identification** before visible symptoms appear
- **Pest detection** with 95%+ accuracy
- **Nutrient deficiency** analysis from leaf color
- **Ripeness assessment** for optimal harvest timing

### 🚜 **Automated Decision Making**
- **Smart irrigation** based on plant stress indicators
- **Targeted spraying** reducing chemical usage by 80%
- **Harvest planning** optimizing labor and resources
- **Quality control** ensuring consistent product standards

### 📊 **Data-Driven Insights**
- **Crop yield prediction** for better planning
- **Growth monitoring** tracking plant development
- **Environmental correlation** linking conditions to outcomes
- **Economic optimization** maximizing profit margins

## 🧠 AI/CV Technology Stack

```
Agricultural AI Pipeline
┌─────────────────────────────────────────────────────────────────┐
│                          Data Input                             │
├─────────────────────────────────────────────────────────────────┤
│  📷 Cameras   │  🛰️ Drones   │  📱 Phones   │  🔬 Sensors    │
│  RGB Images   │  Multispectral│  Farm Photos │  Hyperspectral  │
└─────────────────────────────────────────────────────────────────┘
                                │
                                ▼
┌─────────────────────────────────────────────────────────────────┐
│                       AI Processing                             │
├─────────────────────────────────────────────────────────────────┤
│  🎯 YOLO        │  🧠 CNNs      │  🔤 LLMs      │  📈 ML Models  │
│  Detection      │  Classification│  Reasoning    │  Prediction    │
└─────────────────────────────────────────────────────────────────┘
                                │
                                ▼
┌─────────────────────────────────────────────────────────────────┐
│                        ROS2 Integration                         │
├─────────────────────────────────────────────────────────────────┤
│  📡 Topics      │  🔧 Services  │  ⚡ Actions   │  🤖 Robots     │
│  Data Stream    │  Analysis     │  Long Tasks   │  Automation    │
└─────────────────────────────────────────────────────────────────┘
```

## 📚 Module Overview

### [Module 1: YOLO Basics](01_yolo_basics/)
**Duration:** 45 minutes | **Difficulty:** Beginner  
**What you'll learn:**
- Computer vision fundamentals
- Object detection vs classification
- YOLO architecture and advantages
- Setting up YOLOv10 for plant detection
- Running inference on agricultural images

**Hands-on:** Detect tomato diseases in real images

### [Module 2: Training on Tomato Dataset](02_train_tomato_dataset/)
**Duration:** 45 minutes | **Difficulty:** Intermediate  
**What you'll learn:**
- Agricultural dataset preparation
- Data augmentation techniques
- Transfer learning for plant diseases
- Training, validation, and testing
- Model evaluation metrics

**Hands-on:** Train a custom model on tomato leaf disease data

### [Module 3: Gemini Vision API](03_gemini_vision_api/)
**Duration:** 45 minutes | **Difficulty:** Intermediate  
**What you'll learn:**
- Large Language Models (LLMs) for vision
- Multi-modal AI (text + images)
- Prompt engineering for agriculture
- API integration and error handling
- Cost optimization strategies

**Hands-on:** Create an AI plant health consultant using Gemini

### [Module 4: ROS2 Computer Vision Integration](04_ros2_cv_integration/)
**Duration:** 75 minutes | **Difficulty:** Advanced  
**What you'll learn:**
- cv_bridge for image conversion
- Real-time video processing
- Publishing detection results
- Creating diagnostic services
- Building complete AI-ROS2 pipelines

**Hands-on:** Build a complete robotic plant diagnostic system

## 🚀 Getting Started

### Prerequisites Check
```bash
# Check Python and key packages
python3 --version
python3 -c "import cv2; print('OpenCV version:', cv2.__version__)"
python3 -c "import torch; print('PyTorch version:', torch.__version__)"
python3 -c "import ultralytics; print('Ultralytics YOLO installed')"

# Check ROS2 integration
python3 -c "import rclpy; print('ROS2 Python OK')"
```

### Environment Setup
```bash
# Navigate to Day 2 directory
cd RAISE2025/Day2_AI_CV/

# Install additional AI packages (if needed)
pip install ultralytics roboflow supervision
pip install google-generativeai pillow matplotlib

# Download sample datasets
python3 -c "
import ultralytics
from ultralytics import YOLO
model = YOLO('yolov8n.pt')  # Download pre-trained model
print('Sample model downloaded')
"
```

### Test Your Setup
```bash
# Test YOLO installation
python3 -c "
from ultralytics import YOLO
model = YOLO('yolov8n.pt')
results = model.predict('https://ultralytics.com/images/bus.jpg')
print('YOLO working correctly!')
"

# Test Gemini API (requires API key)
python3 -c "
import google.generativeai as genai
print('Gemini API package installed')
"
```

## 🌿 Agricultural Computer Vision Applications

### Real-World Use Cases

#### 🍅 **Tomato Disease Detection**
```python
# Example: Detect early blight, late blight, leaf mold
diseases = [
    "Early Blight",      # Dark spots with concentric rings
    "Late Blight",       # Water-soaked lesions
    "Leaf Mold",         # Yellow spots on leaves
    "Bacterial Spot",    # Small dark spots
    "Healthy"           # Normal green leaves
]
```

#### 🌾 **Crop Health Monitoring**
```python
# Multi-spectral analysis for crop health
health_indicators = {
    "NDVI": "Normalized Difference Vegetation Index",
    "Chlorophyll": "Photosynthetic capacity",
    "Water_Stress": "Irrigation needs",
    "Nutrient_Status": "Fertilizer requirements"
}
```

#### 🦗 **Pest Detection**
```python
# Common agricultural pests
pests = [
    "Aphids",           # Small green insects
    "Whiteflies",       # Tiny white flying insects
    "Spider_Mites",     # Microscopic mites
    "Thrips",           # Slender insects
    "Caterpillars"      # Larvae of moths/butterflies
]
```

## 💡 Learning Tips

### For Computer Vision Beginners
1. **Start with basics** - Understand images as numbers
2. **Visual learning** - Always look at your data
3. **Iterative approach** - Test small changes frequently
4. **Error analysis** - Study where your model fails
5. **Domain knowledge** - Learn about plant diseases

### For AI/ML Beginners
1. **Focus on concepts** - Don't get lost in math
2. **Hands-on practice** - Code along with examples
3. **Experiment** - Try different parameters
4. **Validation** - Always test on unseen data
5. **Documentation** - Read papers and tutorials

### For ROS2 Integration
1. **Step by step** - Build complexity gradually
2. **Message types** - Understand image transport
3. **Performance** - Monitor processing speeds
4. **Error handling** - Robust error management
5. **Testing** - Use simulation before real robots

## 🔬 Understanding the Technology

### Computer Vision Basics
```python
# An image is just a matrix of numbers
import numpy as np
import cv2

# RGB image: Height x Width x 3 channels
image = np.zeros((480, 640, 3), dtype=np.uint8)  # Black image
image[:, :, 1] = 255  # Make it green

# Grayscale image: Height x Width
gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
```

### YOLO Architecture
```
Input Image (640x640x3)
         │
         ▼
    Feature Extraction
    (Backbone Network)
         │
         ▼
    Feature Pyramid
    (Multi-scale features)
         │
         ▼
    Detection Head
    (Bounding boxes + Classes)
         │
         ▼
    Output Detections
    (x, y, w, h, confidence, class)
```

### LLM Integration
```python
# Multi-modal AI workflow
workflow = {
    "Input": "Image + Text prompt",
    "Processing": "Vision encoder + Language model",
    "Output": "Structured text response",
    "Application": "Agricultural diagnosis"
}
```

## 🎯 Success Metrics

### Technical Metrics
- **Detection Accuracy**: >85% for common diseases
- **Processing Speed**: <2 seconds per image
- **False Positive Rate**: <10%
- **ROS2 Integration**: Real-time image processing

### Practical Metrics
- **Farmer Adoption**: Easy to use interface
- **Cost Effectiveness**: Reduces manual inspection
- **Reliability**: Consistent results across conditions
- **Scalability**: Works across different crops

## 🛠️ Development Environment

### Recommended Setup
```bash
# Development tools
sudo apt install python3-opencv
pip install jupyter notebook
pip install wandb  # For experiment tracking

# ROS2 computer vision packages
sudo apt install ros-humble-cv-bridge
sudo apt install ros-humble-image-transport
sudo apt install ros-humble-vision-msgs
```

### IDE Configuration
```json
// VS Code settings for AI development
{
    "python.defaultInterpreterPath": "/usr/bin/python3",
    "python.linting.enabled": true,
    "python.linting.pylintEnabled": false,
    "python.linting.flake8Enabled": true,
    "jupyter.askForKernelRestart": false
}
```

## 📊 Assessment Preview

### ExamGPT Quiz #2 Topics
1. **Computer Vision Concepts** (25%)
   - Image processing basics
   - Object detection principles
   - Evaluation metrics

2. **YOLO Implementation** (25%)
   - Model architecture
   - Training process
   - Inference pipeline

3. **LLM Integration** (25%)
   - Multi-modal AI
   - Prompt engineering
   - API usage

4. **ROS2 Integration** (25%)
   - cv_bridge usage
   - Real-time processing
   - System architecture

## 🌟 Advanced Topics

### Beyond Basic Detection
- **Semantic Segmentation**: Pixel-level classification
- **Instance Segmentation**: Individual object masks
- **3D Object Detection**: Depth-aware detection
- **Temporal Analysis**: Video-based tracking

### Production Considerations
- **Model Optimization**: TensorRT, ONNX conversion
- **Edge Deployment**: Jetson, Raspberry Pi
- **Cloud Integration**: AWS, Google Cloud
- **Monitoring**: MLOps and model drift

## 🤝 Getting Help

### During the Workshop
- **Instructors**: Available for technical questions
- **Peer Learning**: Work together on challenges
- **Slack Channel**: #raise2025-day2-ai-cv
- **Code Reviews**: Get feedback on your implementation

### Debugging Tips
```bash
# Common debugging commands
python3 -c "import torch; print(torch.cuda.is_available())"  # Check GPU
python3 -c "import cv2; print(cv2.getBuildInformation())"    # Check OpenCV
ros2 topic echo /camera/image_raw --field data | wc -c       # Check image size
```

## 🔗 Resources

### Documentation
- [Ultralytics YOLO Docs](https://docs.ultralytics.com/)
- [OpenCV Python Tutorial](https://opencv-python-tutroals.readthedocs.io/)
- [ROS2 Computer Vision](https://docs.ros.org/en/humble/Tutorials/Advanced/Computer-Vision.html)

### Datasets
- [PlantNet](https://plantnet.org/) - Plant identification
- [PlantVillage](https://plantvillage.psu.edu/) - Disease images
- [iNaturalist](https://www.inaturalist.org/) - Biodiversity data

### Research Papers
- [YOLOv8 Paper](https://arxiv.org/abs/2305.09972)
- [Agricultural AI Survey](https://arxiv.org/abs/2009.07415)
- [Plant Disease Detection Review](https://arxiv.org/abs/2009.04365)

---

**Ready to revolutionize agriculture with AI? Let's start with [Module 1: YOLO Basics](01_yolo_basics/)!** 🚀🌱

## 📄 Quick Reference

### Essential Commands
```bash
# YOLO inference
yolo predict model=yolov8n.pt source=image.jpg

# Train custom model
yolo train model=yolov8n.pt data=dataset.yaml epochs=100

# ROS2 image topics
ros2 topic echo /camera/image_raw
ros2 run image_view image_view image:=/camera/image_raw
```

### Important File Formats
- **Images**: JPG, PNG, TIFF
- **Annotations**: YOLO format (.txt), COCO format (.json)
- **Models**: PyTorch (.pt), ONNX (.onnx)
- **Data**: YAML configuration files

**Success awaits! Let's build the future of smart agriculture together! 🌟** 