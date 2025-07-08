# Module 4: ROS2 + Computer Vision Integration

## 🎯 Learning Objectives

By the end of this module, you will understand:
- ✅ How to integrate computer vision with ROS2
- ✅ Using cv_bridge to convert between ROS and OpenCV images
- ✅ Creating vision-based ROS2 nodes
- ✅ Publishing and subscribing to image topics
- ✅ Building complete agricultural robotics applications

**Duration:** 75 minutes  
**Difficulty:** Intermediate  
**Prerequisites:** Day 1 (ROS2 basics) + Modules 1-3 (Computer Vision)

## 🤖 What is ROS2 + Computer Vision?

Combining ROS2 with computer vision creates intelligent robotic systems that can:
- **See and understand** their environment
- **Make decisions** based on visual information
- **Interact with** the physical world
- **Communicate** findings to other systems

### 🌱 **Agricultural Robotics Pipeline**

```
┌─────────────────────────────────────────────────────────────────────┐
│                    Complete Agricultural Robot                       │
├─────────────────────────────────────────────────────────────────────┤
│                                                                     │
│  📹 Camera    →    🧠 AI Vision    →    📡 ROS2    →    🤖 Actions  │
│  (Sensor)          (Analysis)          (Communication)   (Response)  │
│                                                                     │
│  • RGB Images      • YOLO Detection    • Topics        • Movement   │
│  • Depth Data      • Gemini Analysis   • Services      • Spraying   │
│  • Multispectral   • Custom Models     • Actions       • Harvesting │
│                                                                     │
└─────────────────────────────────────────────────────────────────────┘
```

## 🔧 Core Components

### 📸 **cv_bridge - The Connector**

`cv_bridge` converts between ROS image messages and OpenCV/NumPy arrays:

```python
# ROS2 → OpenCV
ros_image = Image()  # ROS2 sensor_msgs/Image
cv_image = bridge.imgmsg_to_cv2(ros_image, "bgr8")

# OpenCV → ROS2
cv_image = cv2.imread("plant.jpg")  # OpenCV image
ros_image = bridge.cv2_to_imgmsg(cv_image, "bgr8")
```

### 📡 **Image Topics**

ROS2 uses special topics for images:
- **sensor_msgs/Image**: Raw image data
- **sensor_msgs/CompressedImage**: Compressed JPEG images
- **sensor_msgs/CameraInfo**: Camera calibration data

### 🎯 **Vision Node Architecture**

```python
class VisionNode(Node):
    def __init__(self):
        super().__init__('vision_node')
        
        # Subscribe to camera feed
        self.image_sub = self.create_subscription(
            Image, '/camera/image_raw', self.image_callback, 10)
        
        # Publish detection results
        self.detection_pub = self.create_publisher(
            String, '/plant_detections', 10)
        
        # CV Bridge for image conversion
        self.bridge = CvBridge()
        
        # AI model (YOLO, Gemini, etc.)
        self.ai_model = YOLO('plant_model.pt')
    
    def image_callback(self, msg):
        # Convert ROS image to OpenCV
        cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        
        # Run AI analysis
        results = self.ai_model.predict(cv_image)
        
        # Process and publish results
        self.publish_detections(results)
```

## 🌾 Agricultural Applications

### 🍅 **1. Plant Disease Detection Robot**

```python
# Autonomous greenhouse robot
class PlantDoctorRobot(Node):
    def __init__(self):
        super().__init__('plant_doctor')
        
        # Vision system
        self.image_sub = self.create_subscription(
            Image, '/camera/image', self.analyze_plant, 10)
        
        # Movement system
        self.move_pub = self.create_publisher(
            Twist, '/cmd_vel', 10)
        
        # Disease detection results
        self.disease_pub = self.create_publisher(
            String, '/disease_alerts', 10)
    
    def analyze_plant(self, image_msg):
        # Convert to OpenCV
        cv_image = self.bridge.imgmsg_to_cv2(image_msg, "bgr8")
        
        # Run disease detection
        disease_result = self.detect_disease(cv_image)
        
        if disease_result['diseased']:
            # Stop robot
            self.stop_robot()
            
            # Alert farmer
            alert = f"Disease detected: {disease_result['type']}"
            self.disease_pub.publish(String(data=alert))
            
            # Log location
            self.log_disease_location()
```

### 🌽 **2. Crop Monitoring System**

```python
# Field monitoring with multiple cameras
class CropMonitor(Node):
    def __init__(self):
        super().__init__('crop_monitor')
        
        # Multiple camera feeds
        self.camera_subs = []
        for i in range(4):  # 4 cameras
            sub = self.create_subscription(
                Image, f'/camera_{i}/image', 
                lambda msg, cam=i: self.process_field_section(msg, cam), 10)
            self.camera_subs.append(sub)
        
        # Health monitoring results
        self.health_pub = self.create_publisher(
            String, '/crop_health', 10)
    
    def process_field_section(self, image_msg, camera_id):
        cv_image = self.bridge.imgmsg_to_cv2(image_msg, "bgr8")
        
        # Analyze crop health
        health_score = self.analyze_crop_health(cv_image)
        
        # Publish results
        result = {
            'camera_id': camera_id,
            'health_score': health_score,
            'timestamp': time.time()
        }
        self.health_pub.publish(String(data=str(result)))
```

### 🚜 **3. Precision Agriculture Robot**

```python
# Robot that applies treatments based on vision
class PrecisionAgriculture(Node):
    def __init__(self):
        super().__init__('precision_agriculture')
        
        # Vision input
        self.image_sub = self.create_subscription(
            Image, '/camera/image', self.analyze_field, 10)
        
        # Treatment control
        self.spray_pub = self.create_publisher(
            Bool, '/spray_control', 10)
        
        # Navigation
        self.nav_pub = self.create_publisher(
            PoseStamped, '/move_base_simple/goal', 10)
    
    def analyze_field(self, image_msg):
        cv_image = self.bridge.imgmsg_to_cv2(image_msg, "bgr8")
        
        # Detect weeds vs crops
        weed_locations = self.detect_weeds(cv_image)
        
        for weed_loc in weed_locations:
            # Navigate to weed
            self.navigate_to_location(weed_loc)
            
            # Apply targeted treatment
            self.spray_pub.publish(Bool(data=True))
            
            # Log treatment
            self.log_treatment(weed_loc)
```

## 📊 Data Flow Examples

### 🔄 **Simple Detection Pipeline**

```
Camera → Image Topic → Vision Node → Detection Topic → Action Node
  📷         📡           🧠            📡              🤖
```

### 🔄 **Complex Agricultural System**

```
Multiple Cameras → Image Processing → AI Analysis → Decision Making → Robot Control
     📷📷📷           🔄 cv_bridge      🧠 YOLO        🤔 Logic        🤖 Actions
        │                   │            │               │               │
        ▼                   ▼            ▼               ▼               ▼
   /camera_*/image    OpenCV Images   Detections    Treatment Plan    Movement
```

## 🛠️ Hands-On Exercise: Agricultural Vision System

### 🚀 **Building Your First Vision Robot**

We'll create a simple system that:
1. **Captures images** from a camera (or webcam)
2. **Analyzes plant health** using AI
3. **Publishes results** to ROS2 topics
4. **Takes actions** based on findings

### 📱 **Interactive Demo**

```bash
# Navigate to module directory
cd RAISE2025/Day2_AI_CV/04_ros2_cv_integration/

# Install additional packages
pip install opencv-python cv_bridge

# Run the vision demo
python3 agricultural_vision_demo.py
```

## 📦 Required ROS2 Packages

### 🔧 **Installation**

```bash
# Install cv_bridge and image transport
sudo apt install ros-humble-cv-bridge ros-humble-image-transport

# Python packages
pip install opencv-python sensor-msgs-py geometry-msgs-py

# Camera packages (optional)
sudo apt install ros-humble-usb-cam ros-humble-image-view
```

### 📸 **Camera Setup**

```bash
# Test camera connection
ros2 run usb_cam usb_cam_node_exe

# View camera feed
ros2 run image_view image_view --ros-args --remap image:=/image_raw

# List image topics
ros2 topic list | grep image
```

## 🎯 Common Patterns

### 📸 **1. Image Subscriber Pattern**

```python
class ImageProcessor(Node):
    def __init__(self):
        super().__init__('image_processor')
        self.subscription = self.create_subscription(
            Image, '/camera/image_raw', self.image_callback, 10)
        self.bridge = CvBridge()
    
    def image_callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            self.process_image(cv_image)
        except Exception as e:
            self.get_logger().error(f'Image processing error: {e}')
```

### 🧠 **2. AI Integration Pattern**

```python
class AIVisionNode(Node):
    def __init__(self):
        super().__init__('ai_vision')
        
        # Load AI model
        self.model = YOLO('agricultural_model.pt')
        
        # ROS2 setup
        self.image_sub = self.create_subscription(
            Image, '/camera/image', self.ai_callback, 10)
        
        self.result_pub = self.create_publisher(
            String, '/ai_results', 10)
    
    def ai_callback(self, msg):
        # Convert image
        cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        
        # Run AI inference
        results = self.model.predict(cv_image)
        
        # Process results
        for result in results:
            detection_msg = self.format_detection(result)
            self.result_pub.publish(detection_msg)
```

### 🤖 **3. Action-Based Pattern**

```python
class VisionActionNode(Node):
    def __init__(self):
        super().__init__('vision_action')
        
        # Vision subscription
        self.image_sub = self.create_subscription(
            Image, '/camera/image', self.vision_callback, 10)
        
        # Action publishers
        self.move_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.spray_pub = self.create_publisher(Bool, '/spray_control', 10)
    
    def vision_callback(self, msg):
        cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        
        # Analyze image
        analysis = self.analyze_scene(cv_image)
        
        # Take action based on analysis
        if analysis['action'] == 'spray':
            self.spray_pub.publish(Bool(data=True))
        elif analysis['action'] == 'move':
            twist = Twist()
            twist.linear.x = analysis['speed']
            self.move_pub.publish(twist)
```

## 🔄 Integration with Previous Modules

### 📊 **Module 1 (YOLO) + ROS2**

```python
# YOLO detection in ROS2
class YOLORosNode(Node):
    def __init__(self):
        super().__init__('yolo_detector')
        self.model = YOLO('yolov8n.pt')
        
        self.image_sub = self.create_subscription(
            Image, '/camera/image', self.yolo_callback, 10)
        
        self.detection_pub = self.create_publisher(
            String, '/yolo_detections', 10)
    
    def yolo_callback(self, msg):
        cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        results = self.model.predict(cv_image)
        
        # Process YOLO results
        detections = []
        for result in results:
            for box in result.boxes:
                detection = {
                    'class': self.model.names[int(box.cls)],
                    'confidence': float(box.conf),
                    'bbox': box.xyxy.tolist()
                }
                detections.append(detection)
        
        self.detection_pub.publish(String(data=str(detections)))
```

### 🧠 **Module 3 (Gemini) + ROS2**

```python
# Gemini analysis in ROS2
class GeminiRosNode(Node):
    def __init__(self):
        super().__init__('gemini_analyzer')
        
        # Initialize Gemini
        genai.configure(api_key=os.getenv('GEMINI_API_KEY'))
        self.model = genai.GenerativeModel('gemini-pro-vision')
        
        self.image_sub = self.create_subscription(
            Image, '/camera/image', self.gemini_callback, 10)
        
        self.analysis_pub = self.create_publisher(
            String, '/plant_analysis', 10)
    
    def gemini_callback(self, msg):
        cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        
        # Convert to PIL for Gemini
        pil_image = Image.fromarray(cv2.cvtColor(cv_image, cv2.COLOR_BGR2RGB))
        
        # Analyze with Gemini
        prompt = "Analyze this plant image. What's the health status?"
        response = self.model.generate_content([prompt, pil_image])
        
        self.analysis_pub.publish(String(data=response.text))
```

## 🌐 Real-World System Architecture

### 🏭 **Complete Agricultural Robot**

```
┌─────────────────────────────────────────────────────────────────────────┐
│                        Agricultural Robot System                         │
├─────────────────────────────────────────────────────────────────────────┤
│                                                                         │
│  Sensors          Processing         Decision          Actuators         │
│  ┌─────────┐      ┌─────────┐       ┌─────────┐       ┌─────────┐       │
│  │📷 Camera│ ---> │🧠 Vision│ ----> │🤔 Logic │ ----> │🤖 Motors│       │
│  │📡 GPS   │      │📊 AI    │       │📋 Plans │       │💨 Spray │       │
│  │🌡️ Sensors│      │🔄 Filter│       │⚡ Actions│       │📡 Comms │       │
│  └─────────┘      └─────────┘       └─────────┘       └─────────┘       │
│                                                                         │
│  ROS2 Topics:                                                           │
│  • /camera/image_raw    • /plant_detections    • /cmd_vel             │
│  • /gps/fix            • /ai_analysis          • /spray_control        │
│  • /sensor_data        • /treatment_plan       • /system_status        │
│                                                                         │
└─────────────────────────────────────────────────────────────────────────┘
```

## 💡 Best Practices

### ✅ **Do's**
1. **Handle errors gracefully** - images can be corrupted
2. **Optimize processing** - vision algorithms are computationally expensive
3. **Use appropriate message types** - CompressedImage for bandwidth
4. **Implement proper synchronization** - coordinate vision with actions
5. **Log important events** - track robot behavior and decisions

### ❌ **Don'ts**
1. **Don't block callbacks** - keep image processing fast
2. **Don't ignore timestamps** - old images can mislead decisions
3. **Don't hardcode parameters** - use ROS2 parameters
4. **Don't skip error handling** - vision systems can fail
5. **Don't forget about lighting** - vision performance varies with conditions

## 🚀 Getting Started

### 📦 **Quick Setup**
```bash
# Install dependencies
pip install opencv-python sensor-msgs geometry-msgs

# Run the demo
python3 agricultural_vision_demo.py
```

### 🎮 **Testing Your System**
```bash
# Terminal 1: Start ROS2 core
ros2 daemon start

# Terminal 2: Run vision node
python3 agricultural_vision_demo.py

# Terminal 3: Monitor topics
ros2 topic list
ros2 topic echo /plant_detections
```

## 🎯 Learning Outcomes

After completing this module, you will:
- ✅ Understand ROS2 + computer vision integration
- ✅ Be able to create vision-based ROS2 nodes
- ✅ Know how to use cv_bridge for image conversion
- ✅ Understand agricultural robotics system architecture
- ✅ Be ready to build complete agricultural robots

## 📚 What's Next?

After this module, you'll be ready for:
- **Practice Tests**: Apply everything you've learned
- **Day 3 Challenge**: Build a complete agricultural robot system
- **Real-world projects**: Deploy your knowledge in actual farms!

This module brings together everything from Day 1 (ROS2) and Day 2 (AI/CV) to create intelligent agricultural robots! 🤖🌱 