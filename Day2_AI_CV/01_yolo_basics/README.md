# Module 1: YOLO Basics for Agricultural Computer Vision

## 🎯 Learning Objectives

By the end of this module, you will understand:
- ✅ What computer vision is and why it matters for agriculture
- ✅ The difference between classification and object detection
- ✅ How YOLO (You Only Look Once) works
- ✅ Setting up and running YOLO on agricultural images
- ✅ Interpreting detection results and confidence scores
- ✅ Basic image processing for plant diagnostics

**Duration:** 45 minutes  
**Difficulty:** Beginner  
**Prerequisites:** Basic Python knowledge

## 🌱 What is Computer Vision?

Computer vision teaches computers to "see" and understand images, just like humans do. In agriculture, this means:

### 👁️ **Human Vision vs Computer Vision**

| Human Brain | Computer Vision |
|-------------|-----------------|
| Sees plant diseases instantly | Analyzes pixel patterns |
| Recognizes healthy vs sick plants | Compares color histograms |
| Estimates crop maturity | Measures color/texture features |
| Counts fruits on trees | Detects and counts objects |

### 🌾 **Why Computer Vision in Agriculture?**

1. **24/7 Monitoring**: Computers never get tired
2. **Precision**: Detect diseases before human eye can see
3. **Consistency**: Same analysis every time
4. **Speed**: Process thousands of images per hour
5. **Documentation**: Keep records of plant health over time

## 🔍 Classification vs Object Detection

Understanding the difference is crucial for agricultural applications:

### 📋 **Image Classification**
```
Input: Whole image of a tomato plant
Output: "Diseased" or "Healthy"

Question: "What type of plant is this?"
Answer: "Tomato with Early Blight"
```

### 🎯 **Object Detection**
```
Input: Image of a tomato field
Output: Multiple bounding boxes with labels
- Box 1: "Healthy Tomato" (confidence: 0.92)
- Box 2: "Early Blight" (confidence: 0.87)
- Box 3: "Leaf" (confidence: 0.95)

Question: "Where are the diseased areas in this image?"
Answer: Exact locations with bounding boxes
```

### 🚜 **Agricultural Applications**

| Task | Classification | Object Detection |
|------|----------------|------------------|
| **Crop Health** | "Field is healthy/diseased" | "Disease at row 3, plant 15" |
| **Ripeness** | "Field is ready for harvest" | "Ripe tomatoes at coordinates (x,y)" |
| **Pest Control** | "Pests detected in greenhouse" | "Aphids on leaf #3, spray here" |
| **Quality Control** | "Batch grade: Premium/Standard" | "Defective fruits: remove these 5" |

## 🚀 What is YOLO?

**YOLO (You Only Look Once)** is a revolutionary object detection algorithm that can:
- Detect multiple objects in a single image
- Run in real-time (30+ FPS)
- Provide bounding boxes and confidence scores
- Work with any type of object (including plants, diseases, pests)

### 🏗️ YOLO Architecture (Simplified)

```
┌─────────────────────────────────────────────────────────────┐
│                     YOLO Network                            │
├─────────────────────────────────────────────────────────────┤
│                                                             │
│  Input Image        Feature            Detection            │
│  (640x640x3)        Extraction         Head                │
│       │                 │                 │                │
│       ▼                 ▼                 ▼                │
│  ┌─────────┐       ┌─────────┐       ┌─────────┐           │
│  │ Tomato  │  ---> │ CNN     │  ---> │Bounding │           │
│  │ Leaf    │       │Features │       │Boxes +  │           │
│  │ Image   │       │         │       │Classes  │           │
│  └─────────┘       └─────────┘       └─────────┘           │
│                                                             │
│  "Look at this      "Extract visual   "This is an Early    │
│   tomato leaf"       patterns"         Blight at (x,y)"    │
│                                                             │
└─────────────────────────────────────────────────────────────┘
```

### 🎯 YOLO Output Format

For each detected object, YOLO provides:
```python
detection = {
    "class": "Early Blight",          # What object was detected
    "confidence": 0.87,               # How sure the model is (0-1)
    "bbox": [x, y, width, height],    # Location and size
    "center": [x_center, y_center]    # Center coordinates
}
```

## 🛠️ Hands-On Exercise 1: Your First YOLO Detection

Let's detect objects in a sample agricultural image using pre-trained YOLO.

### Step 1: Setup and Installation

```python
# First, let's install YOLO
# Run this in your terminal or notebook
!pip install ultralytics pillow matplotlib

# Import required libraries
from ultralytics import YOLO
import cv2
import matplotlib.pyplot as plt
from PIL import Image
import numpy as np
```

### Step 2: Load Pre-trained YOLO Model

Create `basic_yolo_demo.py`:

```python
#!/usr/bin/env python3
"""
RAISE 2025 - Basic YOLO Demonstration
Introduction to object detection with YOLO for complete beginners.
"""

from ultralytics import YOLO
import cv2
import matplotlib.pyplot as plt
import requests
from PIL import Image
import numpy as np

class YOLOBasicsDemo:
    def __init__(self):
        print("🎯 Loading YOLO model...")
        # Load a pre-trained YOLO model (downloads automatically first time)
        self.model = YOLO('yolov8n.pt')  # 'n' = nano (smallest, fastest)
        print("✅ YOLO model loaded successfully!")
        
        # Define some agricultural-relevant classes that YOLO can detect
        self.agricultural_classes = [
            'person',      # Farmers, workers
            'truck',       # Farm vehicles
            'bird',        # Agricultural pests/wildlife
            'cat',         # Farm animals
            'dog',         # Farm animals
            'horse',       # Farm animals
            'sheep',       # Livestock
            'cow',         # Livestock
            'bottle',      # Pesticide containers
            'chair',       # Farm equipment
            'potted plant' # Potted plants
        ]

    def detect_objects(self, image_path, show_results=True):
        """
        Detect objects in an image using YOLO
        
        Args:
            image_path: Path to image or URL
            show_results: Whether to display the results
        """
        print(f"🔍 Analyzing image: {image_path}")
        
        # Run YOLO detection
        results = self.model(image_path)
        
        # Process results
        detections = []
        for result in results:
            # Get detection data
            boxes = result.boxes
            if boxes is not None:
                for box in boxes:
                    # Extract detection information
                    class_id = int(box.cls[0])
                    confidence = float(box.conf[0])
                    bbox = box.xyxy[0].tolist()  # [x1, y1, x2, y2]
                    class_name = self.model.names[class_id]
                    
                    detection = {
                        'class': class_name,
                        'confidence': confidence,
                        'bbox': bbox,
                        'class_id': class_id
                    }
                    detections.append(detection)
        
        # Print results
        print(f"\n📊 Detection Results:")
        print(f"   Found {len(detections)} objects")
        
        for i, det in enumerate(detections):
            print(f"   {i+1}. {det['class']} (confidence: {det['confidence']:.2f})")
            
            # Highlight agricultural relevance
            if det['class'] in self.agricultural_classes:
                print(f"      🌾 Agricultural relevance: YES")
            else:
                print(f"      🏭 Agricultural relevance: Limited")
        
        # Show results if requested
        if show_results and len(results) > 0:
            self.visualize_results(results[0])
        
        return detections

    def visualize_results(self, result):
        """Visualize detection results with bounding boxes"""
        # Get the annotated image
        annotated_image = result.plot()
        
        # Convert BGR to RGB for matplotlib
        annotated_image_rgb = cv2.cvtColor(annotated_image, cv2.COLOR_BGR2RGB)
        
        # Display
        plt.figure(figsize=(12, 8))
        plt.imshow(annotated_image_rgb)
        plt.title("YOLO Object Detection Results", fontsize=16)
        plt.axis('off')
        plt.tight_layout()
        plt.show()

    def analyze_agricultural_image(self, image_path):
        """
        Analyze an image from an agricultural perspective
        """
        print(f"\n🌱 Agricultural Analysis of: {image_path}")
        print("=" * 60)
        
        detections = self.detect_objects(image_path, show_results=True)
        
        # Agricultural insights
        agricultural_objects = [d for d in detections if d['class'] in self.agricultural_classes]
        
        print(f"\n📈 Agricultural Insights:")
        print(f"   • Total objects detected: {len(detections)}")
        print(f"   • Agricultural objects: {len(agricultural_objects)}")
        
        if agricultural_objects:
            print(f"   • Agricultural objects found:")
            for obj in agricultural_objects:
                print(f"     - {obj['class']} (confidence: {obj['confidence']:.2f})")
        else:
            print(f"   • No common agricultural objects detected")
            print(f"   • Consider using specialized agricultural AI models")

def main():
    # Create YOLO demo instance
    demo = YOLOBasicsDemo()
    
    # Test with sample images
    print("🚀 YOLO Basics Demo - Agricultural Focus")
    print("=" * 50)
    
    # Example 1: General farm scene
    print("\n📷 Example 1: Analyzing a sample image...")
    try:
        # Use a sample image from the internet
        sample_url = "https://ultralytics.com/images/bus.jpg"
        demo.analyze_agricultural_image(sample_url)
    except Exception as e:
        print(f"❌ Error with online image: {e}")
        print("💡 Try with a local image file instead")
    
    # Example 2: Local image (if available)
    print("\n📷 Example 2: You can also analyze local images...")
    print("💡 To analyze your own images, replace 'your_image.jpg' with actual file path")
    
    # Uncomment the line below and add your own image path
    # demo.analyze_agricultural_image('path/to/your/agricultural_image.jpg')

if __name__ == "__main__":
    main()
```

### Step 3: Understanding YOLO Output

When you run the demo, YOLO will output something like:

```
🔍 Analyzing image: sample_farm.jpg

📊 Detection Results:
   Found 5 objects
   1. person (confidence: 0.92)
      🌾 Agricultural relevance: YES
   2. truck (confidence: 0.87)
      🌾 Agricultural relevance: YES
   3. potted plant (confidence: 0.76)
      🌾 Agricultural relevance: YES
   4. bottle (confidence: 0.65)
      🌾 Agricultural relevance: YES
   5. chair (confidence: 0.58)
      🌾 Agricultural relevance: YES
```

### Step 4: Understanding Confidence Scores

```python
def interpret_confidence(confidence):
    """Helper function to interpret YOLO confidence scores"""
    if confidence >= 0.9:
        return "Very High - Almost certain"
    elif confidence >= 0.7:
        return "High - Very likely correct"
    elif confidence >= 0.5:
        return "Medium - Probably correct"
    elif confidence >= 0.3:
        return "Low - Might be correct"
    else:
        return "Very Low - Likely incorrect"

# Example usage
confidence = 0.87
interpretation = interpret_confidence(confidence)
print(f"Confidence: {confidence} = {interpretation}")
```

## 🌿 Exercise 2: Agricultural Image Analysis

Now let's create a specialized agricultural analyzer.

### Create `agricultural_analyzer.py`:

```python
#!/usr/bin/env python3
"""
RAISE 2025 - Agricultural Image Analyzer
Specialized YOLO analysis for agricultural applications.
"""

from ultralytics import YOLO
import cv2
import matplotlib.pyplot as plt
import numpy as np
from PIL import Image
import os

class AgriculturalAnalyzer:
    def __init__(self):
        print("🌱 Initializing Agricultural Image Analyzer...")
        self.model = YOLO('yolov8n.pt')
        
        # Define agricultural object categories
        self.agricultural_mapping = {
            # Farm Animals
            'cow': 'Livestock',
            'sheep': 'Livestock', 
            'horse': 'Livestock',
            'bird': 'Wildlife/Pest',
            
            # Equipment and Vehicles
            'truck': 'Farm Vehicle',
            'car': 'Farm Vehicle',
            'motorcycle': 'Farm Vehicle',
            
            # People and Infrastructure
            'person': 'Farm Worker',
            'chair': 'Farm Equipment',
            'umbrella': 'Weather Protection',
            
            # Containers and Tools
            'bottle': 'Pesticide/Fertilizer Container',
            'cup': 'Container',
            'bowl': 'Container',
            
            # Plants and Growing
            'potted plant': 'Crop/Plant',
            'apple': 'Fruit Crop',
            'orange': 'Fruit Crop',
            'banana': 'Fruit Crop',
        }
        
        print("✅ Agricultural Analyzer ready!")

    def analyze_farm_image(self, image_path, save_result=False):
        """
        Comprehensive agricultural analysis of an image
        """
        print(f"\n🔍 Agricultural Analysis: {os.path.basename(image_path)}")
        print("=" * 60)
        
        # Run detection
        results = self.model(image_path)
        
        # Process detections
        agricultural_detections = []
        general_detections = []
        
        for result in results:
            if result.boxes is not None:
                for box in result.boxes:
                    class_id = int(box.cls[0])
                    confidence = float(box.conf[0])
                    bbox = box.xyxy[0].tolist()
                    class_name = self.model.names[class_id]
                    
                    detection = {
                        'class': class_name,
                        'confidence': confidence,
                        'bbox': bbox,
                        'agricultural_category': self.agricultural_mapping.get(class_name, 'Non-Agricultural')
                    }
                    
                    if class_name in self.agricultural_mapping:
                        agricultural_detections.append(detection)
                    else:
                        general_detections.append(detection)
        
        # Generate report
        self.generate_agricultural_report(agricultural_detections, general_detections)
        
        # Visualize results
        if len(results) > 0:
            self.visualize_agricultural_analysis(results[0], agricultural_detections)
        
        return agricultural_detections, general_detections

    def generate_agricultural_report(self, ag_detections, general_detections):
        """Generate a detailed agricultural analysis report"""
        
        print(f"📊 AGRICULTURAL ANALYSIS REPORT")
        print(f"{'='*40}")
        
        # Summary statistics
        total_objects = len(ag_detections) + len(general_detections)
        print(f"📈 Summary:")
        print(f"   • Total objects detected: {total_objects}")
        print(f"   • Agricultural objects: {len(ag_detections)}")
        print(f"   • General objects: {len(general_detections)}")
        
        if total_objects > 0:
            ag_percentage = (len(ag_detections) / total_objects) * 100
            print(f"   • Agricultural relevance: {ag_percentage:.1f}%")
        
        # Agricultural objects breakdown
        if ag_detections:
            print(f"\n🌾 Agricultural Objects Detected:")
            
            # Group by category
            categories = {}
            for det in ag_detections:
                category = det['agricultural_category']
                if category not in categories:
                    categories[category] = []
                categories[category].append(det)
            
            for category, objects in categories.items():
                print(f"\n   📂 {category}:")
                for obj in objects:
                    confidence_str = f"{obj['confidence']:.2f}"
                    print(f"      • {obj['class']} (confidence: {confidence_str})")
        
        # Recommendations
        print(f"\n💡 Recommendations:")
        self.generate_recommendations(ag_detections, general_detections)

    def generate_recommendations(self, ag_detections, general_detections):
        """Generate actionable recommendations based on detections"""
        
        if not ag_detections and not general_detections:
            print("   • No objects detected - check image quality and lighting")
            return
        
        # Check for livestock
        livestock = [d for d in ag_detections if 'Livestock' in d['agricultural_category']]
        if livestock:
            print("   • Livestock detected - monitor animal health and behavior")
        
        # Check for workers
        workers = [d for d in ag_detections if 'Farm Worker' in d['agricultural_category']]
        if workers:
            print("   • Farm workers present - ensure safety protocols are followed")
        
        # Check for vehicles
        vehicles = [d for d in ag_detections if 'Vehicle' in d['agricultural_category']]
        if vehicles:
            print("   • Farm vehicles detected - check maintenance schedules")
        
        # Check for crops/plants
        plants = [d for d in ag_detections if 'Crop' in d['agricultural_category'] or 'Plant' in d['agricultural_category']]
        if plants:
            print("   • Crops/plants detected - monitor for disease and growth patterns")
        
        # General recommendations
        if len(ag_detections) == 0:
            print("   • Consider using specialized agricultural AI models for plant disease detection")
            print("   • Current model focuses on general objects - may miss plant-specific issues")

    def visualize_agricultural_analysis(self, result, ag_detections):
        """Create visualization highlighting agricultural objects"""
        
        # Get annotated image
        annotated_image = result.plot()
        annotated_image_rgb = cv2.cvtColor(annotated_image, cv2.COLOR_BGR2RGB)
        
        # Create figure with subplots
        fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(16, 8))
        
        # Original image with all detections
        ax1.imshow(annotated_image_rgb)
        ax1.set_title("All Object Detections", fontsize=14)
        ax1.axis('off')
        
        # Agricultural objects summary
        ax2.axis('off')
        ax2.set_title("Agricultural Analysis Summary", fontsize=14)
        
        # Create text summary
        summary_text = "Agricultural Objects Detected:\n\n"
        if ag_detections:
            for i, det in enumerate(ag_detections, 1):
                summary_text += f"{i}. {det['class']}\n"
                summary_text += f"   Category: {det['agricultural_category']}\n"
                summary_text += f"   Confidence: {det['confidence']:.2f}\n\n"
        else:
            summary_text += "No agricultural objects detected.\n\n"
            summary_text += "This image may not contain\n"
            summary_text += "common farm objects, or you may\n"
            summary_text += "need specialized plant disease\n"
            summary_text += "detection models."
        
        ax2.text(0.1, 0.9, summary_text, transform=ax2.transAxes, 
                fontsize=12, verticalalignment='top', fontfamily='monospace')
        
        plt.tight_layout()
        plt.show()

def main():
    """Main function to demonstrate agricultural image analysis"""
    
    print("🌾 RAISE 2025 - Agricultural Image Analyzer")
    print("=" * 50)
    
    analyzer = AgriculturalAnalyzer()
    
    # Example analysis
    print("\n📷 Analyzing sample images...")
    
    # You can replace these with your own agricultural images
    sample_images = [
        "https://ultralytics.com/images/bus.jpg",  # Sample image
        # Add your own image paths here:
        # "path/to/your/farm_image.jpg",
        # "path/to/your/greenhouse_image.jpg",
    ]
    
    for image_path in sample_images:
        try:
            analyzer.analyze_farm_image(image_path)
            print("\n" + "="*60 + "\n")
        except Exception as e:
            print(f"❌ Error analyzing {image_path}: {e}")
    
    print("💡 Tips:")
    print("   • Add your own agricultural images for better results")
    print("   • For plant disease detection, we'll use specialized models in Module 2")
    print("   • YOLO general models work best with larger objects (animals, vehicles, people)")

if __name__ == "__main__":
    main()
```

## 🧠 Understanding Computer Vision Basics

### Image Representation

```python
def understand_images():
    """
    Learn how computers see images as numbers
    """
    import numpy as np
    import matplotlib.pyplot as plt
    
    # Create a simple 5x5 image
    # 0 = black, 255 = white
    simple_image = np.array([
        [0,   0,   255, 0,   0  ],    # Black, Black, White, Black, Black
        [0,   255, 255, 255, 0  ],    # Black, White, White, White, Black  
        [255, 255, 255, 255, 255],    # All White (bright line)
        [0,   255, 255, 255, 0  ],    # Black, White, White, White, Black
        [0,   0,   255, 0,   0  ]     # Black, Black, White, Black, Black
    ])
    
    print("🖼️ How computers see images:")
    print("Image as numbers:")
    print(simple_image)
    
    # Visualize
    plt.figure(figsize=(10, 4))
    
    plt.subplot(1, 2, 1)
    plt.imshow(simple_image, cmap='gray')
    plt.title("What we see")
    plt.axis('off')
    
    plt.subplot(1, 2, 2)
    plt.imshow(simple_image, cmap='gray')
    for i in range(5):
        for j in range(5):
            plt.text(j, i, str(simple_image[i, j]), 
                    ha='center', va='center', color='red', fontsize=8)
    plt.title("What computer sees (pixel values)")
    plt.axis('off')
    
    plt.tight_layout()
    plt.show()

# Run this to understand image representation
understand_images()
```

### Color Images

```python
def understand_color_images():
    """
    Learn about RGB color channels
    """
    import numpy as np
    import matplotlib.pyplot as plt
    
    # Create a small RGB image (3x3x3)
    rgb_image = np.zeros((3, 3, 3), dtype=np.uint8)
    
    # Make it red in top-left
    rgb_image[0, 0] = [255, 0, 0]      # Red
    rgb_image[0, 1] = [0, 255, 0]      # Green  
    rgb_image[0, 2] = [0, 0, 255]      # Blue
    rgb_image[1, 1] = [255, 255, 0]    # Yellow (Red + Green)
    rgb_image[2, 2] = [255, 255, 255]  # White (All colors)
    
    print("🌈 RGB Color Image Structure:")
    print("Shape:", rgb_image.shape)  # (height, width, channels)
    print("Red channel:\n", rgb_image[:,:,0])
    print("Green channel:\n", rgb_image[:,:,1]) 
    print("Blue channel:\n", rgb_image[:,:,2])
    
    # Visualize
    plt.figure(figsize=(12, 3))
    
    plt.subplot(1, 4, 1)
    plt.imshow(rgb_image)
    plt.title("Full Color Image")
    plt.axis('off')
    
    plt.subplot(1, 4, 2)
    plt.imshow(rgb_image[:,:,0], cmap='Reds')
    plt.title("Red Channel")
    plt.axis('off')
    
    plt.subplot(1, 4, 3)
    plt.imshow(rgb_image[:,:,1], cmap='Greens')
    plt.title("Green Channel")
    plt.axis('off')
    
    plt.subplot(1, 4, 4)
    plt.imshow(rgb_image[:,:,2], cmap='Blues')
    plt.title("Blue Channel")
    plt.axis('off')
    
    plt.tight_layout()
    plt.show()

# Uncomment to run
# understand_color_images()
```

## 🎯 Key Takeaways

### 1. Computer Vision Fundamentals
- Images are just matrices of numbers
- RGB images have 3 color channels
- Computer vision finds patterns in these numbers

### 2. YOLO Object Detection
- Detects multiple objects in one pass
- Provides bounding boxes and confidence scores
- Works in real-time for robotic applications

### 3. Agricultural Applications
- General YOLO models detect common farm objects
- Specialized models needed for plant diseases
- Confidence scores indicate detection reliability

### 4. Practical Considerations
- Higher confidence = more reliable detection
- Bounding boxes show exact object locations
- Multiple objects can be detected simultaneously

## 📝 Module 1 Assessment

**Quick Quiz Questions:**
1. What's the difference between classification and object detection?
2. What information does YOLO provide for each detected object?
3. How would you interpret a confidence score of 0.73?
4. Why might general YOLO models miss plant diseases?

**Practical Task:**
Run the agricultural analyzer on 3 different images and interpret the results.

## 🚀 Advanced Exercises

### Exercise A: Custom Visualization
Modify the visualization code to:
1. Draw different colored boxes for different object types
2. Show confidence scores on the image
3. Filter results by confidence threshold

### Exercise B: Performance Analysis
Create a function that:
1. Measures detection time for different image sizes
2. Compares accuracy across different confidence thresholds
3. Analyzes which objects are most commonly detected

### Exercise C: Agricultural Focus
1. Find and test agricultural images online
2. Compare results between general and agricultural images
3. Identify limitations of general YOLO for farming

---

**🎉 Congratulations!** You've completed YOLO Basics and understand how computer vision works for agricultural applications.

**Next:** [Module 2: Training on Tomato Dataset](../02_train_tomato_dataset/) - Learn to train your own agricultural AI models.

## 📚 Additional Resources

### Documentation
- [Ultralytics YOLO Docs](https://docs.ultralytics.com/)
- [OpenCV Python Tutorial](https://docs.opencv.org/master/d6/d00/tutorial_py_root.html)
- [Computer Vision Basics](https://opencv-python-tutroals.readthedocs.io/en/latest/)

### Practice Datasets
- [Google Open Images](https://storage.googleapis.com/openimages/web/index.html)
- [COCO Dataset](https://cocodataset.org/)
- [Agricultural Images](https://github.com/spMohanty/PlantVillage-Dataset)

---

**Questions? Need Help?**
- Review the code examples above
- Test different images and analyze results
- Ask your instructor about specific agricultural applications! 