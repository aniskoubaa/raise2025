# Coding Exercises: Day 2 Practice Tests

## 💻 Programming Assessment

These coding exercises test your practical implementation skills in AI and Computer Vision for agricultural robotics.

**Total Points**: 40 points  
**Duration**: 45 minutes  
**Skills Tested**: Implementation, Problem-solving, Debugging

## 📋 Exercise Overview

### 1. YOLO Detection (5 points)
**File**: `exercise_1_yolo_detection.py`  
**Task**: Implement YOLO object detection on agricultural images  
**Skills**: YOLO usage, result interpretation, agricultural relevance

### 2. Model Evaluation (8 points)
**File**: `exercise_2_model_evaluation.py`  
**Task**: Calculate accuracy, precision, recall, and F1-score metrics  
**Skills**: Performance metrics, confusion matrix, evaluation analysis

### 3. Data Preparation (7 points)
**File**: `exercise_3_data_preparation.py`  
**Task**: Implement data augmentation techniques for agricultural images  
**Skills**: OpenCV transformations, data augmentation, image processing

### 4. Prompt Engineering (8 points)
**File**: `exercise_4_prompt_engineering.py`  
**Task**: Write effective prompts for agricultural plant analysis  
**Skills**: LLM interaction, prompt optimization, agricultural knowledge

### 5. Image Processing (7 points)
**File**: `exercise_5_image_processing.py`  
**Task**: Basic OpenCV operations for agricultural image analysis  
**Skills**: OpenCV, image filtering, feature extraction

### 6. ROS2 Integration (5 points)
**File**: `exercise_6_ros2_integration.py`  
**Task**: Create simple vision node structure for agricultural robotics  
**Skills**: ROS2 patterns, cv_bridge, node architecture

## 🚀 Getting Started

### Prerequisites
```bash
# Navigate to coding exercises
cd RAISE2025/Day2_AI_CV/practice_tests/coding_exercises/

# Install required packages
pip install ultralytics opencv-python numpy matplotlib

# Optional for full functionality
pip install google-generativeai rclpy cv_bridge
```

### Running Exercises

#### Option 1: Run All Exercises
```bash
# Complete assessment with timer
python3 run_exercises.py

# Practice mode (no timer)
python3 run_exercises.py --practice
```

#### Option 2: Individual Exercises
```bash
# Run specific exercise
python3 exercise_1_yolo_detection.py
python3 exercise_2_model_evaluation.py
python3 exercise_3_data_preparation.py
python3 exercise_4_prompt_engineering.py
python3 exercise_5_image_processing.py
python3 exercise_6_ros2_integration.py
```

## 📊 Grading Criteria

### Code Quality (40%)
- **Clean Code**: Proper naming, comments, structure
- **Best Practices**: Following Python conventions
- **Error Handling**: Robust error management
- **Documentation**: Clear explanations

### Functionality (40%)
- **Correctness**: Code works as intended
- **Completeness**: All requirements met
- **Edge Cases**: Handles various scenarios
- **Agricultural Relevance**: Addresses farming needs

### Efficiency (20%)
- **Algorithm Choice**: Appropriate methods used
- **Performance**: Optimized for agricultural robotics
- **Resource Usage**: Memory and CPU efficient
- **Scalability**: Can handle real-world data

## 🎯 Learning Objectives

### Exercise 1: YOLO Detection
- [ ] Load and use YOLO models
- [ ] Process agricultural images
- [ ] Interpret detection results
- [ ] Filter for agricultural relevance

### Exercise 2: Model Evaluation
- [ ] Calculate performance metrics
- [ ] Understand precision vs recall tradeoffs
- [ ] Analyze confusion matrices
- [ ] Apply metrics to agricultural scenarios

### Exercise 3: Data Preparation
- [ ] Implement image augmentation
- [ ] Preserve agricultural image characteristics
- [ ] Create training-ready datasets
- [ ] Understand augmentation impact

### Exercise 4: Prompt Engineering
- [ ] Write effective prompts for plant analysis
- [ ] Optimize for agricultural insights
- [ ] Handle API responses
- [ ] Cost-effective prompt design

### Exercise 5: Image Processing
- [ ] Apply OpenCV operations
- [ ] Extract agricultural features
- [ ] Enhance image quality
- [ ] Prepare images for AI analysis

### Exercise 6: ROS2 Integration
- [ ] Create ROS2 vision nodes
- [ ] Use cv_bridge for image conversion
- [ ] Implement agricultural robotics patterns
- [ ] Coordinate vision with actions

## 💡 Tips for Success

### Time Management
- **Read all exercises first** (5 minutes)
- **Start with familiar topics** (build confidence)
- **Leave time for testing** (test your code)
- **Don't get stuck** (move on if needed)

### Problem-Solving Strategy
1. **Understand the requirement**
2. **Break into smaller steps**
3. **Write pseudocode first**
4. **Implement incrementally**
5. **Test with sample data**
6. **Add error handling**

### Agricultural Focus
- **Think about real farming scenarios**
- **Consider practical constraints**
- **Optimize for field conditions**
- **Design for non-technical users**

## 📚 Reference Materials

### Quick References
- **YOLO**: `model.predict(image, conf=0.5)`
- **OpenCV**: `cv2.imread()`, `cv2.resize()`, `cv2.flip()`
- **Metrics**: `precision = TP/(TP+FP)`
- **ROS2**: `self.create_subscription(Image, topic, callback)`

### Documentation Links
- Module 1: YOLO Basics documentation
- Module 2: Training concepts and metrics
- Module 3: Gemini Vision API examples
- Module 4: ROS2 + CV integration patterns

## 🔧 Common Issues

### Import Errors
```python
# If YOLO not available
try:
    from ultralytics import YOLO
except ImportError:
    print("YOLO not available - using simulation")
```

### ROS2 Not Available
```python
# Graceful fallback
try:
    import rclpy
    ROS2_AVAILABLE = True
except ImportError:
    ROS2_AVAILABLE = False
```

### API Key Issues
```python
# Safe API key handling
api_key = os.getenv('GEMINI_API_KEY')
if not api_key:
    print("API key not found - using demo mode")
```

## 🎯 Success Criteria

### Minimum Requirements (70%)
- All exercises attempt made
- Code runs without errors
- Basic functionality implemented
- Agricultural context considered

### Good Performance (80%)
- Clean, well-commented code
- Proper error handling
- Agricultural relevance clear
- Efficient implementations

### Excellent Performance (90%+)
- Optimized solutions
- Creative agricultural applications
- Robust error handling
- Professional code quality

## 🚀 Next Steps

After completing coding exercises:
1. **Review your solutions** with the provided answers
2. **Understand any mistakes** for learning
3. **Move to integration challenge** for final assessment
4. **Practice weak areas** before Day 3

Good luck with your coding exercises! Focus on practical agricultural applications and clean, working code. 🌱💻🚀 