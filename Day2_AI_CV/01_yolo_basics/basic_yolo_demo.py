#!/usr/bin/env python3
"""
RAISE 2025 - Basic YOLO Demonstration
Introduction to object detection with YOLO for complete beginners.

This script demonstrates:
- Loading a pre-trained YOLO model
- Running inference on images
- Understanding detection results
- Agricultural relevance analysis

Author: RAISE 2025 Team
Date: July 2025
"""

from ultralytics import YOLO
import cv2
import matplotlib.pyplot as plt
import requests
from PIL import Image
import numpy as np
import os

class YOLOBasicsDemo:
    def __init__(self):
        print("🎯 Loading YOLO model...")
        # Load a pre-trained YOLO model (downloads automatically first time)
        self.model = YOLO('yolov8n.pt')  # 'n' = nano (smallest, fastest)
        print("✅ YOLO model loaded successfully!")
        print(f"📋 Model can detect {len(self.model.names)} different object classes")
        
        # Define some agricultural-relevant classes that YOLO can detect
        self.agricultural_classes = [
            'person',      # Farmers, workers
            'truck',       # Farm vehicles
            'car',         # Farm vehicles
            'bus',         # Transport vehicles
            'bird',        # Agricultural pests/wildlife
            'cat',         # Farm animals
            'dog',         # Farm animals
            'horse',       # Farm animals
            'sheep',       # Livestock
            'cow',         # Livestock
            'bottle',      # Pesticide containers
            'chair',       # Farm equipment
            'potted plant', # Potted plants
            'dining table', # Farm equipment
            'cup',         # Containers
            'bowl',        # Containers
            'apple',       # Fruit crops
            'orange',      # Fruit crops
            'banana',      # Fruit crops
        ]
        
        print(f"🌾 Agricultural classes available: {len(self.agricultural_classes)} out of {len(self.model.names)}")

    def show_all_classes(self):
        """Display all classes that YOLO can detect"""
        print("\n📚 All YOLO Classes:")
        print("=" * 50)
        
        agricultural_count = 0
        for i, class_name in self.model.names.items():
            is_agricultural = class_name in self.agricultural_classes
            marker = "🌾" if is_agricultural else "🏭"
            print(f"{marker} {i:2d}: {class_name}")
            if is_agricultural:
                agricultural_count += 1
        
        print(f"\n📊 Summary:")
        print(f"   • Total classes: {len(self.model.names)}")
        print(f"   • Agricultural relevance: {agricultural_count} classes")
        print(f"   • Agricultural percentage: {(agricultural_count/len(self.model.names)*100):.1f}%")

    def detect_objects(self, image_path, show_results=True):
        """
        Detect objects in an image using YOLO
        
        Args:
            image_path: Path to image or URL
            show_results: Whether to display the results
        """
        print(f"\n🔍 Analyzing image: {os.path.basename(str(image_path))}")
        
        try:
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
                            'class_id': class_id,
                            'is_agricultural': class_name in self.agricultural_classes
                        }
                        detections.append(detection)
            
            # Print results
            print(f"\n📊 Detection Results:")
            print(f"   Found {len(detections)} objects")
            
            if detections:
                print(f"\n📋 Detailed Results:")
                for i, det in enumerate(detections, 1):
                    confidence_str = self.interpret_confidence(det['confidence'])
                    agricultural_marker = "🌾" if det['is_agricultural'] else "🏭"
                    
                    print(f"   {i}. {agricultural_marker} {det['class']}")
                    print(f"      Confidence: {det['confidence']:.3f} ({confidence_str})")
                    print(f"      Location: [{det['bbox'][0]:.0f}, {det['bbox'][1]:.0f}, {det['bbox'][2]:.0f}, {det['bbox'][3]:.0f}]")
                    
                    if det['is_agricultural']:
                        print(f"      🌱 Agricultural relevance: HIGH")
                    else:
                        print(f"      🏭 Agricultural relevance: LOW")
                    print()
            else:
                print("   No objects detected in this image")
                print("   💡 Try images with:")
                print("      • Better lighting")
                print("      • Clear, unblurred objects")
                print("      • Common objects (people, vehicles, animals)")
            
            # Show results if requested
            if show_results and len(results) > 0:
                self.visualize_results(results[0], detections)
            
            return detections
            
        except Exception as e:
            print(f"❌ Error processing image: {e}")
            return []

    def interpret_confidence(self, confidence):
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

    def visualize_results(self, result, detections):
        """Visualize detection results with bounding boxes"""
        try:
            # Get the annotated image
            annotated_image = result.plot()
            
            # Convert BGR to RGB for matplotlib
            annotated_image_rgb = cv2.cvtColor(annotated_image, cv2.COLOR_BGR2RGB)
            
            # Create visualization
            fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(16, 8))
            
            # Show annotated image
            ax1.imshow(annotated_image_rgb)
            ax1.set_title("YOLO Detection Results", fontsize=14)
            ax1.axis('off')
            
            # Show detection summary
            ax2.axis('off')
            ax2.set_title("Detection Summary", fontsize=14)
            
            # Create summary text
            summary_text = "Objects Detected:\n\n"
            agricultural_count = 0
            
            if detections:
                for i, det in enumerate(detections, 1):
                    if det['is_agricultural']:
                        agricultural_count += 1
                        marker = "🌾"
                    else:
                        marker = "🏭"
                    
                    summary_text += f"{marker} {det['class']}\n"
                    summary_text += f"   Confidence: {det['confidence']:.2f}\n\n"
                
                summary_text += f"\nSummary:\n"
                summary_text += f"Total: {len(detections)} objects\n"
                summary_text += f"Agricultural: {agricultural_count}\n"
                summary_text += f"General: {len(detections) - agricultural_count}\n"
            else:
                summary_text += "No objects detected\n\n"
                summary_text += "Tips:\n"
                summary_text += "• Check image quality\n"
                summary_text += "• Ensure good lighting\n"
                summary_text += "• Use common objects\n"
            
            ax2.text(0.1, 0.9, summary_text, transform=ax2.transAxes, 
                    fontsize=11, verticalalignment='top', fontfamily='monospace')
            
            plt.tight_layout()
            plt.show()
            
        except Exception as e:
            print(f"❌ Error creating visualization: {e}")

    def analyze_agricultural_relevance(self, image_path):
        """
        Analyze an image from an agricultural perspective
        """
        print(f"\n🌱 Agricultural Relevance Analysis")
        print("=" * 60)
        
        detections = self.detect_objects(image_path, show_results=True)
        
        if not detections:
            print("\n💡 Agricultural Insights:")
            print("   • No objects detected by general YOLO model")
            print("   • For plant disease detection, use specialized agricultural AI")
            print("   • General YOLO works best for larger objects (vehicles, animals, people)")
            return
        
        # Analyze agricultural relevance
        agricultural_objects = [d for d in detections if d['is_agricultural']]
        general_objects = [d for d in detections if not d['is_agricultural']]
        
        print(f"\n📈 Agricultural Relevance Score:")
        if detections:
            ag_percentage = (len(agricultural_objects) / len(detections)) * 100
            print(f"   • Agricultural relevance: {ag_percentage:.1f}%")
            print(f"   • Agricultural objects: {len(agricultural_objects)}")
            print(f"   • General objects: {len(general_objects)}")
        
        # Provide recommendations
        print(f"\n💡 Recommendations:")
        
        if agricultural_objects:
            print("   ✅ Image contains agricultural objects")
            
            # Categorize agricultural objects
            categories = {}
            for obj in agricultural_objects:
                category = self.get_agricultural_category(obj['class'])
                if category not in categories:
                    categories[category] = []
                categories[category].append(obj)
            
            for category, objects in categories.items():
                print(f"   📂 {category}: {len(objects)} objects")
                for obj in objects:
                    print(f"      • {obj['class']} (confidence: {obj['confidence']:.2f})")
        else:
            print("   ⚠️ No agricultural objects detected")
            print("   💡 Consider using:")
            print("      • Specialized plant disease detection models")
            print("      • Images with farm equipment, animals, or workers")
            print("      • Better quality agricultural images")

    def get_agricultural_category(self, class_name):
        """Categorize agricultural objects"""
        categories = {
            'Livestock': ['cow', 'sheep', 'horse'],
            'Farm Workers': ['person'],
            'Farm Vehicles': ['truck', 'car', 'bus'],
            'Wildlife': ['bird'],
            'Farm Animals': ['cat', 'dog'],
            'Containers': ['bottle', 'cup', 'bowl'],
            'Equipment': ['chair', 'dining table'],
            'Crops': ['potted plant', 'apple', 'orange', 'banana']
        }
        
        for category, classes in categories.items():
            if class_name in classes:
                return category
        return 'Other Agricultural'

def main():
    """Main function to demonstrate YOLO basics"""
    
    print("🚀 YOLO Basics Demo - Agricultural Focus")
    print("=" * 60)
    
    # Create YOLO demo instance
    demo = YOLOBasicsDemo()
    
    # Show all available classes
    print("\n📚 First, let's see what YOLO can detect:")
    demo.show_all_classes()
    
    # Test with sample images
    print("\n📷 Now let's analyze some images...")
    
    # Example 1: Sample image from the internet
    print("\n" + "="*60)
    print("Example 1: Online sample image")
    try:
        sample_url = "https://ultralytics.com/images/bus.jpg"
        demo.analyze_agricultural_relevance(sample_url)
    except Exception as e:
        print(f"❌ Error with online image: {e}")
        print("💡 Check your internet connection or try local images")
    
    # Example 2: Instructions for local images
    print("\n" + "="*60)
    print("Example 2: Your own images")
    print("📷 To analyze your own images:")
    print("   1. Save agricultural images in this directory")
    print("   2. Uncomment and modify the lines below:")
    print("   3. Run the script again")
    print()
    print("# Uncomment these lines to test your own images:")
    print("# demo.analyze_agricultural_relevance('your_farm_image.jpg')")
    print("# demo.analyze_agricultural_relevance('your_greenhouse_image.jpg')")
    
    # Example usage for students to uncomment
    # demo.analyze_agricultural_relevance('your_farm_image.jpg')
    
    print(f"\n🎯 Key Learning Points:")
    print("   ✅ YOLO can detect multiple objects in one image")
    print("   ✅ Each detection has a confidence score (0-1)")
    print("   ✅ Bounding boxes show exact object locations")
    print("   ✅ General YOLO works well for common farm objects")
    print("   ⚠️ Specialized models needed for plant diseases")
    
    print(f"\n📚 Next Steps:")
    print("   • Try analyzing your own agricultural images")
    print("   • Experiment with different confidence thresholds")
    print("   • Move to Module 2 for plant disease detection")

if __name__ == "__main__":
    main() 