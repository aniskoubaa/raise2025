# QUICKSTART: YOLO Basics

## ⚡ 3-Minute Demo

### Step 1: Setup
```bash
# Navigate to the module directory
cd RAISE2025/Day2_AI_CV/01_yolo_basics/

# Install required packages
pip install ultralytics opencv-python matplotlib pillow

# Test your installation
python3 -c "from ultralytics import YOLO; print('✅ YOLO installed successfully!')"
```

### Step 2: Run Basic Detection
```bash
# Run the demo with sample images
python3 basic_yolo_demo.py

# Follow the interactive prompts:
# 1. Press Enter to see all YOLO classes
# 2. Choose option 1 for demo images
# 3. Choose option 2 for sample agricultural images
```

### Step 3: Try Your Own Images
```bash
# Put your image in this directory and run:
python3 basic_yolo_demo.py

# Then select option 3 (custom image)
# Enter your image filename when prompted
```

## 📸 Sample Images to Try

### Good Images for Detection:
- Photos with clear objects (people, vehicles, animals)
- Well-lit images
- Images with multiple objects
- Farm scenes with tractors, animals, people

### Avoid These:
- Very dark/blurry images
- Images with only plants/crops (YOLO isn't trained on plant diseases)
- Very small objects
- Abstract or artistic images

## 🔧 Quick Commands

```bash
# See all available YOLO classes
python3 -c "from ultralytics import YOLO; m=YOLO('yolov8n.pt'); print(list(m.names.values()))"

# Quick detection on any image
python3 -c "from ultralytics import YOLO; YOLO('yolov8n.pt')('your_image.jpg')[0].show()"

# Check if an image has agricultural objects
python3 basic_yolo_demo.py --quick your_image.jpg
```

## 🎯 Expected Results

After running the demo, you should see:
- ✅ Objects detected with bounding boxes
- ✅ Confidence scores (0.0-1.0)
- ✅ Agricultural relevance analysis
- ✅ Visualization of results

## 🔍 Understanding Your Results

### Confidence Scores:
- **0.9+**: Very confident detection
- **0.7-0.9**: High confidence
- **0.5-0.7**: Medium confidence
- **0.3-0.5**: Low confidence
- **<0.3**: Very low confidence

### Agricultural Relevance:
- 🌾 **High**: Directly related to farming
- 🏭 **Low**: General objects not specific to agriculture

## 🚨 Troubleshooting

### Common Issues:

**"No module named ultralytics"**
```bash
pip install ultralytics
```

**"Model download failed"**
```bash
# Check internet connection, then try:
python3 -c "from ultralytics import YOLO; YOLO('yolov8n.pt')"
```

**"No objects detected"**
- Try different images
- Check image quality
- Ensure objects are clearly visible
- Use images with common objects (people, vehicles, animals)

**"Image not found"**
- Check file path and name
- Ensure image is in the correct directory
- Try absolute path: `/full/path/to/image.jpg`

## 📚 What's Next?

After completing this module, you'll be ready for:
- **Module 2**: Training custom models on agricultural data
- **Module 3**: Using AI language models for plant analysis
- **Module 4**: Integrating computer vision with ROS2 robots

## 💡 Pro Tips

1. **Start Simple**: Use the demo images first
2. **Check Results**: Look at confidence scores
3. **Try Different Images**: See what works best
4. **Take Notes**: Record interesting findings
5. **Ask Questions**: Don't hesitate to experiment!

Happy detecting! 🎯 