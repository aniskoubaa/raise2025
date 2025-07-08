#!/usr/bin/env python3
"""
RAISE 2025 - Agricultural Vision ROS2 Integration Demo
Educational demonstration of integrating computer vision with ROS2 for agricultural robotics.

This script demonstrates:
- ROS2 + Computer Vision integration patterns
- Image processing with cv_bridge
- Vision-based decision making
- Agricultural robotics applications

Author: RAISE 2025 Team
Date: July 2025
"""

import time
import random
import threading
from datetime import datetime
import json
import cv2
import numpy as np

# Try to import ROS2 packages, fall back to simulation if not available
try:
    import rclpy
    from rclpy.node import Node
    from sensor_msgs.msg import Image, CompressedImage
    from std_msgs.msg import String, Bool
    from geometry_msgs.msg import Twist
    from cv_bridge import CvBridge
    ROS2_AVAILABLE = True
    print("✅ ROS2 packages detected - using real ROS2")
except ImportError:
    ROS2_AVAILABLE = False
    print("ℹ️ ROS2 not available - running in simulation mode")
    print("   Install ROS2 and cv_bridge for full functionality")

class AgriculturalVisionDemo:
    def __init__(self):
        print("🤖 Agricultural Vision + ROS2 Integration Demo")
        print("=" * 50)
        
        # Initialize based on ROS2 availability
        if ROS2_AVAILABLE:
            self.setup_ros2_demo()
        else:
            self.setup_simulation_demo()
        
        # Agricultural scenarios
        self.scenarios = {
            'healthy_crop': {
                'description': 'Healthy tomato plants in greenhouse',
                'ai_result': {'health_score': 0.95, 'disease': None, 'action': 'continue'},
                'robot_action': 'continue_monitoring'
            },
            'disease_detected': {
                'description': 'Early blight detected on tomato leaves',
                'ai_result': {'health_score': 0.65, 'disease': 'early_blight', 'action': 'spray'},
                'robot_action': 'spray_fungicide'
            },
            'pest_infestation': {
                'description': 'Aphids detected on corn plants',
                'ai_result': {'health_score': 0.70, 'disease': None, 'pest': 'aphids', 'action': 'spray'},
                'robot_action': 'spray_pesticide'
            },
            'nutrient_deficiency': {
                'description': 'Nitrogen deficiency in wheat field',
                'ai_result': {'health_score': 0.75, 'nutrient': 'nitrogen_low', 'action': 'fertilize'},
                'robot_action': 'apply_fertilizer'
            },
            'ready_harvest': {
                'description': 'Ripe tomatoes ready for harvest',
                'ai_result': {'health_score': 0.90, 'ripeness': 'ready', 'action': 'harvest'},
                'robot_action': 'harvest_fruit'
            }
        }

    def setup_ros2_demo(self):
        """Setup real ROS2 integration"""
        print("🚀 Setting up ROS2 integration...")
        
        # Initialize ROS2
        rclpy.init()
        
        # Create vision node
        self.vision_node = AgriculturalVisionNode()
        
        # Create control node
        self.control_node = RobotControlNode()
        
        print("✅ ROS2 nodes initialized")

    def setup_simulation_demo(self):
        """Setup simulation mode"""
        print("🎮 Setting up simulation mode...")
        self.simulation_active = True
        self.current_scenario = 'healthy_crop'
        print("✅ Simulation initialized")

    def explain_integration(self):
        """Explain ROS2 + CV integration concepts"""
        print("\n🧠 ROS2 + Computer Vision Integration")
        print("=" * 40)
        
        print("Key Components:")
        print("  📸 cv_bridge: Converts between ROS images and OpenCV")
        print("  📡 Image Topics: /camera/image_raw, /camera/image_compressed")
        print("  🧠 Vision Nodes: Process images and extract information")
        print("  🤖 Control Nodes: Take actions based on vision results")
        print("  📊 Data Flow: Camera → Vision → Decision → Action")
        
        print("\n🌱 Agricultural Applications:")
        print("  • Plant disease detection and treatment")
        print("  • Pest identification and control")
        print("  • Crop health monitoring")
        print("  • Harvest timing optimization")
        print("  • Precision agriculture")
        
        print("\n🔄 Integration Patterns:")
        print("  1. Image Subscriber: Receives camera feed")
        print("  2. AI Processing: Analyzes images with ML models")
        print("  3. Result Publisher: Shares findings with other nodes")
        print("  4. Action Coordination: Triggers robot responses")

    def show_system_architecture(self):
        """Show the complete system architecture"""
        print("\n🏗️ Agricultural Robot System Architecture")
        print("=" * 45)
        
        print("┌─────────────────────────────────────────────────────────────────┐")
        print("│                    Agricultural Robot System                    │")
        print("├─────────────────────────────────────────────────────────────────┤")
        print("│                                                                 │")
        print("│  📹 Sensors     →    🧠 Processing    →    🤖 Actions          │")
        print("│  ┌─────────┐        ┌─────────┐          ┌─────────┐            │")
        print("│  │ Camera  │   ---> │ Vision  │   ---->  │ Motors  │            │")
        print("│  │ GPS     │        │ AI      │          │ Spray   │            │")
        print("│  │ Sensors │        │ Logic   │          │ Harvest │            │")
        print("│  └─────────┘        └─────────┘          └─────────┘            │")
        print("│                                                                 │")
        print("│  ROS2 Topics:                                                   │")
        print("│  • /camera/image_raw      • /plant_detections                   │")
        print("│  • /gps/fix              • /ai_analysis                        │")
        print("│  • /sensor_data          • /robot_actions                      │")
        print("│                                                                 │")
        print("└─────────────────────────────────────────────────────────────────┘")

    def simulate_vision_processing(self, scenario_name):
        """Simulate computer vision processing"""
        scenario = self.scenarios[scenario_name]
        
        print(f"\n🔍 Processing: {scenario['description']}")
        print("  📸 Capturing image from camera...")
        time.sleep(0.5)
        
        print("  🧠 Running AI analysis...")
        time.sleep(1)
        
        print("  📊 Extracting features...")
        time.sleep(0.5)
        
        # Simulate AI results
        result = scenario['ai_result']
        
        print(f"  ✅ Analysis complete!")
        print(f"     Health Score: {result['health_score']:.2f}")
        
        if 'disease' in result and result['disease']:
            print(f"     Disease: {result['disease']}")
        if 'pest' in result and result['pest']:
            print(f"     Pest: {result['pest']}")
        if 'nutrient' in result and result['nutrient']:
            print(f"     Nutrient Status: {result['nutrient']}")
        if 'ripeness' in result:
            print(f"     Ripeness: {result['ripeness']}")
        
        print(f"     Recommended Action: {result['action']}")
        
        return result

    def simulate_robot_response(self, vision_result, scenario_name):
        """Simulate robot response to vision analysis"""
        scenario = self.scenarios[scenario_name]
        action = scenario['robot_action']
        
        print(f"\n🤖 Robot Response: {action}")
        
        if action == 'continue_monitoring':
            print("  ✅ Plants healthy - continuing patrol")
            print("  📡 Publishing: /cmd_vel (move forward)")
            print("  📊 Logging: Healthy plants at GPS coordinates")
        
        elif action == 'spray_fungicide':
            print("  🚨 Disease detected - stopping for treatment")
            print("  📡 Publishing: /cmd_vel (stop)")
            print("  📡 Publishing: /spray_control (activate fungicide)")
            print("  📍 Logging: Disease location for follow-up")
        
        elif action == 'spray_pesticide':
            print("  🐛 Pest detected - initiating pest control")
            print("  📡 Publishing: /cmd_vel (stop)")
            print("  📡 Publishing: /spray_control (activate pesticide)")
            print("  📊 Updating: Pest monitoring database")
        
        elif action == 'apply_fertilizer':
            print("  🌱 Nutrient deficiency - applying fertilizer")
            print("  📡 Publishing: /cmd_vel (stop)")
            print("  📡 Publishing: /fertilizer_control (activate)")
            print("  📈 Scheduling: Follow-up monitoring in 7 days")
        
        elif action == 'harvest_fruit':
            print("  🍅 Harvest ready - collecting fruit")
            print("  📡 Publishing: /cmd_vel (stop)")
            print("  📡 Publishing: /harvest_control (activate)")
            print("  📊 Logging: Harvest data for yield analysis")
        
        time.sleep(2)

    def show_code_examples(self):
        """Show practical code examples"""
        print("\n💻 Code Examples: ROS2 + CV Integration")
        print("=" * 40)
        
        print("1. Basic Image Subscriber:")
        print("```python")
        print("class VisionNode(Node):")
        print("    def __init__(self):")
        print("        super().__init__('vision_node')")
        print("        self.subscription = self.create_subscription(")
        print("            Image, '/camera/image_raw', self.image_callback, 10)")
        print("        self.bridge = CvBridge()")
        print("        self.ai_model = YOLO('plant_model.pt')")
        print("")
        print("    def image_callback(self, msg):")
        print("        cv_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')")
        print("        results = self.ai_model.predict(cv_image)")
        print("        self.process_results(results)")
        print("```")
        
        print("\n2. AI Integration with ROS2:")
        print("```python")
        print("def process_results(self, ai_results):")
        print("    for result in ai_results:")
        print("        if result.confidence > 0.8:")
        print("            # High confidence detection")
        print("            detection_msg = String()")
        print("            detection_msg.data = f'Disease: {result.class_name}'")
        print("            self.detection_pub.publish(detection_msg)")
        print("            ")
        print("            # Trigger robot action")
        print("            self.trigger_treatment(result.class_name)")
        print("```")
        
        print("\n3. Robot Action Coordination:")
        print("```python")
        print("def trigger_treatment(self, disease_type):")
        print("    # Stop robot movement")
        print("    stop_msg = Twist()")
        print("    self.velocity_pub.publish(stop_msg)")
        print("    ")
        print("    # Activate treatment system")
        print("    if disease_type == 'early_blight':")
        print("        spray_msg = Bool(data=True)")
        print("        self.spray_pub.publish(spray_msg)")
        print("```")

    def interactive_scenarios(self):
        """Interactive scenario demonstration"""
        print("\n🎮 Interactive Agricultural Scenarios")
        print("=" * 40)
        
        print("Available scenarios:")
        for i, (key, scenario) in enumerate(self.scenarios.items(), 1):
            print(f"{i}. {scenario['description']}")
        
        while True:
            try:
                choice = input(f"\nSelect scenario (1-{len(self.scenarios)}) or 'q' to quit: ").strip()
                
                if choice.lower() == 'q':
                    break
                
                choice_num = int(choice)
                if 1 <= choice_num <= len(self.scenarios):
                    scenario_key = list(self.scenarios.keys())[choice_num - 1]
                    
                    print(f"\n🎬 Running Scenario: {scenario_key}")
                    print("-" * 30)
                    
                    # Simulate vision processing
                    vision_result = self.simulate_vision_processing(scenario_key)
                    
                    # Simulate robot response
                    self.simulate_robot_response(vision_result, scenario_key)
                    
                    # Show ROS2 topic activity
                    self.show_topic_activity(scenario_key)
                    
                else:
                    print("❌ Invalid choice. Please try again.")
                    
            except ValueError:
                print("❌ Please enter a valid number.")
            except KeyboardInterrupt:
                print("\n👋 Exiting scenarios...")
                break

    def show_topic_activity(self, scenario_key):
        """Show simulated ROS2 topic activity"""
        print(f"\n📡 ROS2 Topic Activity:")
        print("-" * 25)
        
        # Simulate topic messages
        topics = [
            ("/camera/image_raw", "sensor_msgs/Image", "30 Hz"),
            ("/plant_detections", "std_msgs/String", "1 Hz"),
            ("/ai_analysis", "std_msgs/String", "1 Hz"),
            ("/cmd_vel", "geometry_msgs/Twist", "10 Hz"),
            ("/spray_control", "std_msgs/Bool", "0.1 Hz"),
            ("/system_status", "std_msgs/String", "1 Hz")
        ]
        
        print(f"{'Topic':25} {'Type':20} {'Rate':10}")
        print("-" * 55)
        
        for topic, msg_type, rate in topics:
            status = "●" if random.random() > 0.3 else "○"
            print(f"{topic:25} {msg_type:20} {rate:10} {status}")

    def performance_analysis(self):
        """Show performance considerations"""
        print("\n📊 Performance Analysis")
        print("=" * 25)
        
        print("Processing Pipeline Performance:")
        print("┌─────────────────────────────────────────────────────────────┐")
        print("│  Stage              │  Time (ms)  │  CPU Usage  │  Memory │")
        print("├─────────────────────────────────────────────────────────────┤")
        print("│  Image Capture      │     5-10    │     5%      │   50MB  │")
        print("│  cv_bridge Convert  │     1-2     │     2%      │   10MB  │")
        print("│  YOLO Detection     │    50-100   │    30%      │  200MB  │")
        print("│  Gemini Analysis    │   2000-5000 │    10%      │  100MB  │")
        print("│  Decision Logic     │     1-5     │     5%      │    5MB  │")
        print("│  Robot Control      │     5-10    │    10%      │   20MB  │")
        print("└─────────────────────────────────────────────────────────────┘")
        
        print("\n💡 Optimization Tips:")
        print("  • Use CompressedImage for bandwidth efficiency")
        print("  • Process images at lower resolution for speed")
        print("  • Cache AI model results to avoid redundant processing")
        print("  • Use threading for concurrent processing")
        print("  • Implement smart triggering (motion detection)")

    def troubleshooting_guide(self):
        """Show common issues and solutions"""
        print("\n🔧 Troubleshooting Guide")
        print("=" * 25)
        
        issues = [
            {
                "problem": "Images not being received",
                "symptoms": ["No image callbacks", "Empty topic list"],
                "solutions": [
                    "Check camera connection",
                    "Verify topic names with 'ros2 topic list'",
                    "Test with 'ros2 run image_view image_view'"
                ]
            },
            {
                "problem": "cv_bridge conversion errors",
                "symptoms": ["Encoding errors", "Image format mismatches"],
                "solutions": [
                    "Check image encoding (bgr8, rgb8, mono8)",
                    "Verify image dimensions",
                    "Handle exceptions in image_callback"
                ]
            },
            {
                "problem": "AI model slow performance",
                "symptoms": ["High latency", "Dropped frames"],
                "solutions": [
                    "Reduce image resolution",
                    "Use faster model variants",
                    "Implement frame skipping"
                ]
            },
            {
                "problem": "Robot actions not responding",
                "symptoms": ["No movement", "Commands ignored"],
                "solutions": [
                    "Check topic connections",
                    "Verify message formats",
                    "Test with manual commands"
                ]
            }
        ]
        
        for i, issue in enumerate(issues, 1):
            print(f"\n{i}. {issue['problem']}")
            print(f"   Symptoms: {', '.join(issue['symptoms'])}")
            print(f"   Solutions:")
            for solution in issue['solutions']:
                print(f"     • {solution}")

    def run_complete_demo(self):
        """Run the complete demonstration"""
        print("\n🎬 Complete Agricultural Vision Demo")
        print("=" * 40)
        
        input("Press Enter to learn about ROS2 + CV integration...")
        self.explain_integration()
        
        input("\nPress Enter to see system architecture...")
        self.show_system_architecture()
        
        input("\nPress Enter to see code examples...")
        self.show_code_examples()
        
        input("\nPress Enter to try interactive scenarios...")
        self.interactive_scenarios()
        
        input("\nPress Enter to see performance analysis...")
        self.performance_analysis()
        
        input("\nPress Enter to see troubleshooting guide...")
        self.troubleshooting_guide()
        
        print("\n🎉 Demo Complete!")
        print("=" * 20)
        print("You now understand:")
        print("  ✅ ROS2 + Computer Vision integration")
        print("  ✅ cv_bridge for image conversion")
        print("  ✅ Agricultural robotics applications")
        print("  ✅ Performance optimization")
        print("  ✅ Troubleshooting common issues")
        print("\nReady for Day 2 Practice Tests! 🚀")

# ROS2 Node Classes (only created if ROS2 is available)
if ROS2_AVAILABLE:
    class AgriculturalVisionNode(Node):
        def __init__(self):
            super().__init__('agricultural_vision')
            
            # Image subscription
            self.image_subscription = self.create_subscription(
                Image, '/camera/image_raw', self.image_callback, 10)
            
            # Result publishers
            self.detection_pub = self.create_publisher(String, '/plant_detections', 10)
            self.analysis_pub = self.create_publisher(String, '/ai_analysis', 10)
            
            # CV Bridge
            self.bridge = CvBridge()
            
            self.get_logger().info('Agricultural Vision Node initialized')
        
        def image_callback(self, msg):
            try:
                # Convert ROS image to OpenCV
                cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
                
                # Process image (simplified for demo)
                self.process_image(cv_image)
                
            except Exception as e:
                self.get_logger().error(f'Image processing error: {e}')
        
        def process_image(self, cv_image):
            # Simulate AI processing
            result = {
                'timestamp': time.time(),
                'health_score': random.uniform(0.6, 0.95),
                'detection_count': random.randint(0, 5)
            }
            
            # Publish results
            self.detection_pub.publish(String(data=str(result)))
    
    class RobotControlNode(Node):
        def __init__(self):
            super().__init__('robot_control')
            
            # Subscriptions
            self.detection_sub = self.create_subscription(
                String, '/plant_detections', self.detection_callback, 10)
            
            # Publishers
            self.velocity_pub = self.create_publisher(Twist, '/cmd_vel', 10)
            self.spray_pub = self.create_publisher(Bool, '/spray_control', 10)
            
            self.get_logger().info('Robot Control Node initialized')
        
        def detection_callback(self, msg):
            # Process detection results and take action
            self.get_logger().info(f'Received detection: {msg.data}')

def main():
    """Main function"""
    demo = AgriculturalVisionDemo()
    
    try:
        while True:
            print("\n🤖 Agricultural Vision + ROS2 Demo Menu:")
            print("=" * 40)
            print("1. 🎬 Run complete demo")
            print("2. 🧠 Learn about ROS2 + CV integration")
            print("3. 🏗️ See system architecture")
            print("4. 💻 View code examples")
            print("5. 🎮 Try interactive scenarios")
            print("6. 📊 Performance analysis")
            print("7. 🔧 Troubleshooting guide")
            print("8. 🚪 Exit")
            
            choice = input("\nEnter your choice (1-8): ").strip()
            
            if choice == '1':
                demo.run_complete_demo()
            elif choice == '2':
                demo.explain_integration()
            elif choice == '3':
                demo.show_system_architecture()
            elif choice == '4':
                demo.show_code_examples()
            elif choice == '5':
                demo.interactive_scenarios()
            elif choice == '6':
                demo.performance_analysis()
            elif choice == '7':
                demo.troubleshooting_guide()
            elif choice == '8':
                print("👋 Thanks for learning about agricultural robotics!")
                break
            else:
                print("❌ Invalid choice. Please try again.")
                
    except KeyboardInterrupt:
        print("\n👋 Demo interrupted by user")
    finally:
        if ROS2_AVAILABLE:
            try:
                rclpy.shutdown()
            except:
                pass

if __name__ == '__main__':
    main() 