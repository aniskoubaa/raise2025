#!/usr/bin/env python3
"""
Exercise 1: YOLO Detection (5 points)
Test your ability to use YOLO for agricultural object detection.

Task: Implement YOLO object detection on agricultural images and filter for agricultural relevance.

Skills Tested:
- YOLO model usage
- Result interpretation
- Agricultural relevance filtering
- Confidence score handling

Time: 8 minutes
"""

import os
import numpy as np
import cv2

# Try to import YOLO, fallback to simulation if not available
try:
    from ultralytics import YOLO
    YOLO_AVAILABLE = True
except ImportError:
    YOLO_AVAILABLE = False
    print("⚠️ YOLO not available - using simulation mode")

class YOLODetectionExercise:
    def __init__(self):
        self.agricultural_classes = {
            'person', 'bicycle', 'car', 'motorcycle', 'airplane', 'bus', 'train', 'truck',
            'boat', 'traffic light', 'fire hydrant', 'stop sign', 'parking meter', 'bench',
            'bird', 'cat', 'dog', 'horse', 'sheep', 'cow', 'elephant', 'bear', 'zebra',
            'giraffe', 'backpack', 'umbrella', 'handbag', 'tie', 'suitcase', 'frisbee',
            'skis', 'snowboard', 'sports ball', 'kite', 'baseball bat', 'baseball glove',
            'skateboard', 'surfboard', 'tennis racket', 'bottle', 'wine glass', 'cup',
            'fork', 'knife', 'spoon', 'bowl', 'banana', 'apple', 'sandwich', 'orange',
            'broccoli', 'carrot', 'hot dog', 'pizza', 'donut', 'cake', 'chair', 'couch',
            'potted plant', 'bed', 'dining table', 'toilet', 'tv', 'laptop', 'mouse',
            'remote', 'keyboard', 'cell phone', 'microwave', 'oven', 'toaster', 'sink',
            'refrigerator', 'book', 'clock', 'vase', 'scissors', 'teddy bear', 'hair drier',
            'toothbrush'
        }
        
        # Agricultural relevance mapping
        self.agricultural_relevance = {
            'high': ['potted plant', 'apple', 'banana', 'orange', 'broccoli', 'carrot', 'cow', 'horse', 'sheep', 'bird'],
            'medium': ['person', 'truck', 'bottle', 'bowl', 'chair', 'dining table'],
            'low': ['car', 'motorcycle', 'tv', 'laptop', 'cell phone', 'couch', 'toilet']
        }
        
        if YOLO_AVAILABLE:
            self.model = YOLO('yolov8n.pt')  # Use nano model for speed
        else:
            self.model = None
    
    def create_sample_image(self):
        """Create a sample agricultural scene for testing"""
        # Create a simple scene with agricultural elements
        img = np.ones((480, 640, 3), dtype=np.uint8) * 120  # Gray background
        
        # Add some simple shapes to simulate agricultural objects
        # Green rectangle (plant)
        cv2.rectangle(img, (100, 200), (200, 400), (0, 255, 0), -1)
        
        # Brown rectangle (soil/pot)
        cv2.rectangle(img, (80, 350), (220, 420), (101, 67, 33), -1)
        
        # Red circle (tomato)
        cv2.circle(img, (150, 250), 30, (0, 0, 255), -1)
        
        # Yellow circle (sun/light)
        cv2.circle(img, (500, 100), 50, (0, 255, 255), -1)
        
        # Add some text
        cv2.putText(img, "Agricultural Scene", (200, 50), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)
        
        return img
    
    def run_yolo_detection(self, image, confidence_threshold=0.5):
        """
        TODO: Implement YOLO detection on the given image
        
        Args:
            image: Input image (numpy array)
            confidence_threshold: Minimum confidence for detections
            
        Returns:
            dict: Detection results with format:
            {
                'detections': [
                    {
                        'class': 'class_name',
                        'confidence': 0.85,
                        'bbox': [x1, y1, x2, y2],
                        'agricultural_relevance': 'high'|'medium'|'low'
                    }
                ],
                'total_detections': int,
                'high_relevance_count': int
            }
        """
        
        # YOUR CODE HERE
        # Hint: Use self.model.predict(image, conf=confidence_threshold)
        # Process the results and extract bounding boxes, classes, and confidence scores
        # Use self.get_agricultural_relevance() to determine relevance
        
        if not YOLO_AVAILABLE:
            # Simulation mode - return mock results
            return {
                'detections': [
                    {
                        'class': 'potted plant',
                        'confidence': 0.89,
                        'bbox': [100, 200, 200, 400],
                        'agricultural_relevance': 'high'
                    },
                    {
                        'class': 'person',
                        'confidence': 0.76,
                        'bbox': [300, 150, 400, 450],
                        'agricultural_relevance': 'medium'
                    }
                ],
                'total_detections': 2,
                'high_relevance_count': 1
            }
        
        # Real implementation starts here
        results = self.model.predict(image, conf=confidence_threshold)
        
        detections = []
        high_relevance_count = 0
        
        for result in results:
            if result.boxes is not None:
                for box in result.boxes:
                    class_id = int(box.cls)
                    class_name = self.model.names[class_id]
                    confidence = float(box.conf)
                    bbox = box.xyxy[0].tolist()  # [x1, y1, x2, y2]
                    
                    agricultural_relevance = self.get_agricultural_relevance(class_name)
                    
                    detection = {
                        'class': class_name,
                        'confidence': confidence,
                        'bbox': bbox,
                        'agricultural_relevance': agricultural_relevance
                    }
                    
                    detections.append(detection)
                    
                    if agricultural_relevance == 'high':
                        high_relevance_count += 1
        
        return {
            'detections': detections,
            'total_detections': len(detections),
            'high_relevance_count': high_relevance_count
        }
    
    def get_agricultural_relevance(self, class_name):
        """
        TODO: Determine agricultural relevance of a detected class
        
        Args:
            class_name: Name of the detected class
            
        Returns:
            str: 'high', 'medium', or 'low' relevance
        """
        
        # YOUR CODE HERE
        # Use self.agricultural_relevance dictionary to classify relevance
        # Return 'low' for unknown classes
        
        for relevance_level, classes in self.agricultural_relevance.items():
            if class_name in classes:
                return relevance_level
        
        return 'low'  # Default for unknown classes
    
    def filter_agricultural_detections(self, detections, min_relevance='medium'):
        """
        TODO: Filter detections based on agricultural relevance
        
        Args:
            detections: List of detection dictionaries
            min_relevance: Minimum relevance level ('high', 'medium', 'low')
            
        Returns:
            list: Filtered detections
        """
        
        # YOUR CODE HERE
        # Filter detections based on relevance level
        # 'high' includes only high relevance
        # 'medium' includes medium and high relevance
        # 'low' includes all detections
        
        relevance_hierarchy = ['low', 'medium', 'high']
        min_level = relevance_hierarchy.index(min_relevance)
        
        filtered = []
        for detection in detections:
            detection_level = relevance_hierarchy.index(detection['agricultural_relevance'])
            if detection_level >= min_level:
                filtered.append(detection)
        
        return filtered
    
    def draw_detections(self, image, detections):
        """
        TODO: Draw bounding boxes and labels on the image
        
        Args:
            image: Input image (numpy array)
            detections: List of detection dictionaries
            
        Returns:
            numpy array: Image with drawn detections
        """
        
        # YOUR CODE HERE
        # Draw bounding boxes with different colors for different relevance levels
        # Add labels with class name and confidence
        # Use different colors: high=green, medium=yellow, low=red
        
        img_with_detections = image.copy()
        
        # Color mapping for relevance levels
        color_map = {
            'high': (0, 255, 0),     # Green
            'medium': (0, 255, 255), # Yellow
            'low': (0, 0, 255)       # Red
        }
        
        for detection in detections:
            bbox = detection['bbox']
            class_name = detection['class']
            confidence = detection['confidence']
            relevance = detection['agricultural_relevance']
            
            # Convert bbox to integers
            x1, y1, x2, y2 = map(int, bbox)
            
            # Draw bounding box
            color = color_map[relevance]
            cv2.rectangle(img_with_detections, (x1, y1), (x2, y2), color, 2)
            
            # Draw label
            label = f"{class_name}: {confidence:.2f}"
            label_size = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 0.5, 2)[0]
            cv2.rectangle(img_with_detections, (x1, y1 - label_size[1] - 10), 
                         (x1 + label_size[0], y1), color, -1)
            cv2.putText(img_with_detections, label, (x1, y1 - 5), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)
        
        return img_with_detections
    
    def run_exercise(self):
        """Run the complete exercise"""
        print("🎯 Exercise 1: YOLO Detection for Agriculture")
        print("=" * 45)
        
        # Create or load sample image
        print("📸 Creating sample agricultural image...")
        sample_image = self.create_sample_image()
        
        # Run YOLO detection
        print("🔍 Running YOLO detection...")
        results = self.run_yolo_detection(sample_image, confidence_threshold=0.5)
        
        print(f"✅ Detection Results:")
        print(f"   Total detections: {results['total_detections']}")
        print(f"   High relevance: {results['high_relevance_count']}")
        
        # Display individual detections
        print("\n📋 Individual Detections:")
        for i, detection in enumerate(results['detections'], 1):
            print(f"   {i}. {detection['class']} ({detection['confidence']:.2f}) - {detection['agricultural_relevance']} relevance")
        
        # Filter for agricultural relevance
        print("\n🌱 Filtering for agricultural relevance...")
        high_relevance = self.filter_agricultural_detections(results['detections'], 'high')
        medium_relevance = self.filter_agricultural_detections(results['detections'], 'medium')
        
        print(f"   High relevance only: {len(high_relevance)} detections")
        print(f"   Medium+ relevance: {len(medium_relevance)} detections")
        
        # Draw detections
        print("\n🎨 Drawing detections on image...")
        img_with_detections = self.draw_detections(sample_image, results['detections'])
        
        # Save result
        output_path = "exercise_1_result.jpg"
        cv2.imwrite(output_path, img_with_detections)
        print(f"💾 Result saved to: {output_path}")
        
        # Exercise questions
        self.ask_questions(results)
        
        return results
    
    def ask_questions(self, results):
        """Ask questions about the results"""
        print("\n❓ Exercise Questions:")
        print("=" * 25)
        
        print("1. Why is agricultural relevance important for farming robots?")
        print("   a) It makes detection faster")
        print("   b) It reduces false positives and focuses on farm-relevant objects")
        print("   c) It improves image quality")
        print("   d) It reduces memory usage")
        
        print("\n2. What confidence threshold would you use for critical disease detection?")
        print("   a) 0.3 (low - catch everything)")
        print("   b) 0.5 (medium - balanced)")
        print("   c) 0.8 (high - very confident)")
        print("   d) 0.95 (very high - almost certain)")
        
        print("\n3. In your results, which objects had the highest agricultural relevance?")
        high_relevance = [d['class'] for d in results['detections'] if d['agricultural_relevance'] == 'high']
        print(f"   Answer: {high_relevance}")
        
        print("\n💡 Think about:")
        print("   • How would you adapt this for different crops?")
        print("   • What happens if the lighting changes?")
        print("   • How could you improve agricultural relevance classification?")

def main():
    """Main function to run the exercise"""
    exercise = YOLODetectionExercise()
    
    try:
        results = exercise.run_exercise()
        print("\n🎉 Exercise 1 Complete!")
        print("📝 Make sure to answer the questions and understand the concepts.")
        
    except Exception as e:
        print(f"\n❌ Error in exercise: {e}")
        print("💡 Check your implementation and try again.")

if __name__ == "__main__":
    main() 