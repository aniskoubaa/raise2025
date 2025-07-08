# Day 2 Practice Tests: AI & Computer Vision

## 🎯 Assessment Overview

These practice tests evaluate your understanding of AI and Computer Vision for agricultural robotics, covering all Day 2 modules:

- **Module 1**: YOLO Basics and Object Detection
- **Module 2**: Training on Tomato Disease Dataset  
- **Module 3**: Gemini Vision API for Plant Analysis
- **Module 4**: ROS2 + Computer Vision Integration

**Total Time**: 90 minutes  
**Passing Score**: 70% overall  
**Format**: Mixed (Quiz + Coding + Integration)

## 📚 Test Categories

### 1. 🧠 Knowledge Quiz (25 points)
**Duration**: 20 minutes  
**Questions**: 25 multiple choice  
**Topics**: AI/CV fundamentals, agricultural applications, best practices

**Coverage**:
- Computer vision basics (classification vs detection)
- YOLO architecture and applications
- Machine learning concepts (training, validation, testing)
- Transfer learning and data augmentation
- Large Language Models and prompt engineering
- ROS2 + CV integration patterns
- Agricultural AI use cases

### 2. 💻 Coding Exercises (40 points)
**Duration**: 45 minutes  
**Exercises**: 6 practical coding tasks  
**Skills**: Implementation, problem-solving, debugging

**Exercises**:
1. **YOLO Detection** (5 pts): Run object detection on agricultural images
2. **Model Evaluation** (8 pts): Calculate accuracy, precision, recall metrics
3. **Data Preparation** (7 pts): Implement data augmentation techniques
4. **Prompt Engineering** (8 pts): Write effective prompts for plant analysis
5. **Image Processing** (7 pts): Basic OpenCV operations for agriculture
6. **ROS2 Integration** (5 pts): Create simple vision node structure

### 3. 🚀 Integration Challenge (35 points)
**Duration**: 25 minutes  
**Task**: Build complete agricultural AI system  
**Skills**: System design, integration, real-world application

**Challenge**: Create an intelligent greenhouse monitoring system that:
- Processes camera feeds
- Detects plant health issues
- Makes treatment recommendations
- Integrates with ROS2 for robot control
- Provides farmer-friendly reports

## 🎯 Learning Objectives Tested

### 🔍 Computer Vision Understanding
- [ ] Distinguish between classification and object detection
- [ ] Understand YOLO architecture and workflow
- [ ] Apply computer vision to agricultural problems
- [ ] Interpret confidence scores and model outputs

### 🧠 Machine Learning Concepts
- [ ] Explain training, validation, and testing datasets
- [ ] Understand transfer learning benefits
- [ ] Implement data augmentation techniques
- [ ] Evaluate model performance with metrics

### 🤖 AI Integration
- [ ] Use Large Language Models for plant analysis
- [ ] Write effective prompts for agricultural AI
- [ ] Integrate multiple AI systems
- [ ] Handle API costs and limitations

### 🌐 ROS2 + Computer Vision
- [ ] Use cv_bridge for image conversion
- [ ] Create vision-based ROS2 nodes
- [ ] Implement real-time image processing
- [ ] Coordinate vision with robot actions

### 🌱 Agricultural Applications
- [ ] Apply AI to plant disease detection
- [ ] Design precision agriculture systems
- [ ] Optimize crop monitoring workflows
- [ ] Implement harvest automation

## 📊 Grading Criteria

### 🎯 Grade Levels

| Score | Grade | Meaning |
|-------|-------|---------|
| 90-100% | **A** | Excellent - Ready for advanced agricultural AI projects |
| 80-89% | **B** | Good - Strong understanding with minor gaps |
| 70-79% | **C** | Satisfactory - Meets basic requirements |
| 60-69% | **D** | Needs improvement - Review key concepts |
| <60% | **F** | Insufficient - Retake recommended |

### 📋 Assessment Rubric

#### Knowledge Quiz (25 points)
- **Excellent (23-25)**: 90%+ correct, demonstrates deep understanding
- **Good (20-22)**: 80-89% correct, solid grasp of concepts
- **Satisfactory (18-19)**: 70-79% correct, basic understanding
- **Needs Work (<18)**: <70% correct, significant gaps

#### Coding Exercises (40 points)
- **Code Quality (40%)**: Clean, well-commented, follows best practices
- **Functionality (40%)**: Works correctly, handles edge cases
- **Efficiency (20%)**: Optimized solutions, proper algorithm choice

#### Integration Challenge (35 points)
- **System Design (30%)**: Well-architected, modular, scalable
- **Implementation (30%)**: Working code, proper integration
- **Agricultural Relevance (25%)**: Addresses real farming needs
- **Documentation (15%)**: Clear explanations, usage instructions

## 🚀 Getting Started

### 📦 Prerequisites
```bash
# Navigate to practice tests
cd RAISE2025/Day2_AI_CV/practice_tests/

# Install required packages
pip install ultralytics opencv-python matplotlib numpy

# Optional: Install for full functionality
pip install google-generativeai rclpy cv_bridge
```

### 🎮 Running the Tests

#### Option 1: Complete Assessment (Recommended)
```bash
# Run all tests in sequence
python3 run_complete_assessment.py

# Follow the guided workflow:
# 1. Knowledge Quiz (20 min)
# 2. Coding Exercises (45 min) 
# 3. Integration Challenge (25 min)
# 4. Automated grading and feedback
```

#### Option 2: Individual Components
```bash
# Knowledge quiz only
python3 knowledge_quiz.py

# Coding exercises only
cd coding_exercises/
python3 run_exercises.py

# Integration challenge only
python3 integration_challenge.py
```

#### Option 3: Practice Mode
```bash
# Untimed practice with hints
python3 run_complete_assessment.py --practice-mode

# Review mode with explanations
python3 run_complete_assessment.py --review-mode
```

## 📚 Study Guide

### 🔍 Key Topics to Review

#### Module 1: YOLO Basics
- Object detection vs image classification
- YOLO architecture components
- Confidence scores and bounding boxes
- Agricultural object detection applications
- Pre-trained vs custom models

#### Module 2: Model Training
- Training/validation/test dataset splits
- Transfer learning advantages
- Data augmentation techniques
- Performance metrics (accuracy, precision, recall, F1)
- Overfitting vs underfitting

#### Module 3: Gemini Vision API
- Large Language Models capabilities
- Vision + language integration
- Prompt engineering best practices
- API cost management
- Agricultural analysis applications

#### Module 4: ROS2 Integration
- cv_bridge image conversion
- ROS2 image topics and message types
- Vision node architecture patterns
- Real-time processing considerations
- Robot action coordination

### 💡 Study Tips

1. **Hands-on Practice**: Run all module demos before testing
2. **Understand Concepts**: Don't just memorize, understand why
3. **Agricultural Focus**: Think about real farming applications
4. **Integration**: Practice combining different AI techniques
5. **Time Management**: Practice under time constraints

### 📖 Recommended Review Materials

1. **Module Documentation**: Re-read all README files
2. **Code Examples**: Study and run all demo scripts
3. **QUICKSTART Guides**: Practice the step-by-step workflows
4. **Agricultural Research**: Read about real AI farming applications
5. **Error Handling**: Practice debugging common issues

## 🔄 Retake Policy

### 📅 Retake Schedule
- **Immediate**: Practice mode (unlimited attempts)
- **Same Day**: Review specific weak areas
- **Next Day**: Full retake if score <70%
- **One Week**: Additional study time if needed

### 📈 Improvement Strategy
1. **Analyze Results**: Review detailed feedback report
2. **Focus Study**: Target lowest-scoring areas
3. **Additional Practice**: Use practice mode extensively
4. **Seek Help**: Ask instructors about challenging concepts
5. **Practical Application**: Try building small projects

## 🏆 Certification Preparation

### 📜 Day 2 Certificate Requirements
- **Minimum Score**: 70% overall
- **No Section**: Below 60% in any category
- **Time Limit**: Complete within allocated time
- **Academic Integrity**: Original work, no external help during assessment

### 🎯 Advanced Certification Track
- **Score Target**: 90%+ for advanced recognition
- **Bonus Challenges**: Optional extra credit projects
- **Portfolio**: Document your agricultural AI projects
- **Peer Review**: Help other students and get feedback

## 📞 Support and Resources

### 🆘 Getting Help
- **During Tests**: Limited help available (clarification only)
- **Study Phase**: Full instructor support
- **Technical Issues**: Immediate assistance available
- **Concept Questions**: Office hours and discussion forums

### 📚 Additional Resources
- **Documentation**: All module READMEs and QUICKSTARTs
- **Code Examples**: Complete working demos in each module
- **External Links**: Agricultural AI research papers and articles
- **Video Tutorials**: Supplementary learning materials
- **Practice Datasets**: Additional images for experimentation

Good luck with your Day 2 assessment! Remember, this is about demonstrating your understanding of AI and Computer Vision for agricultural robotics. Focus on practical applications and real-world problem solving! 🌱🤖🚀 