#!/usr/bin/env python3
"""
Day 1 Coding Exercises Runner - RAISE 2025
Practical programming challenges for ROS2 agricultural robotics.

This script provides guided coding exercises to test practical implementation skills.
"""

import os
import sys
import subprocess
import time
from datetime import datetime


class CodingExercises:
    """Interactive coding exercises for Day 1 ROS2 concepts."""
    
    def __init__(self):
        self.exercises = self.load_exercises()
        self.current_exercise = 0
        self.completed_exercises = []
        self.start_time = None
        
    def load_exercises(self):
        """Load exercise definitions."""
        return [
            {
                "title": "Exercise 1: Soil Moisture Sensor Publisher",
                "description": "Create a ROS2 node that publishes soil moisture readings",
                "file": "exercise1_soil_sensor.py",
                "template": "exercise1_template.py",
                "difficulty": "Beginner",
                "concepts": ["Publisher/Subscriber", "Sensor Simulation"],
                "time_limit": 8,
                "requirements": [
                    "Create a node named 'soil_moisture_sensor'",
                    "Publish Float64 messages to '/soil_moisture' topic",
                    "Publish readings every 2 seconds",
                    "Generate realistic values between 0-100%",
                    "Include proper logging and error handling"
                ]
            },
            {
                "title": "Exercise 2: Crop Health Data Subscriber",
                "description": "Create a subscriber that processes and analyzes crop health data",
                "file": "exercise2_crop_analyzer.py",
                "template": "exercise2_template.py",
                "difficulty": "Beginner",
                "concepts": ["Publisher/Subscriber", "Data Analysis"],
                "time_limit": 8,
                "requirements": [
                    "Subscribe to '/crop_health' topic (String messages)",
                    "Parse JSON data from messages",
                    "Calculate average health scores",
                    "Log alerts for health scores below 60",
                    "Display summary statistics every 10 seconds"
                ]
            },
            {
                "title": "Exercise 3: Simple Irrigation Service",
                "description": "Implement a basic irrigation control service",
                "file": "exercise3_irrigation_service.py",
                "template": "exercise3_template.py",
                "difficulty": "Intermediate",
                "concepts": ["Service/Client", "Control Systems"],
                "time_limit": 10,
                "requirements": [
                    "Create a service server for irrigation control",
                    "Accept start/stop irrigation requests",
                    "Track water usage and duration",
                    "Return status and confirmation messages",
                    "Handle error conditions gracefully"
                ]
            },
            {
                "title": "Exercise 4: Field Navigation Pattern",
                "description": "Create a simple field navigation pattern for turtlesim",
                "file": "exercise4_navigation.py",
                "template": "exercise4_template.py",
                "difficulty": "Intermediate",
                "concepts": ["Movement Control", "Coordinate Systems"],
                "time_limit": 10,
                "requirements": [
                    "Control turtlesim movement with velocity commands",
                    "Implement a simple square movement pattern",
                    "Track current position using pose feedback",
                    "Stop at boundaries to prevent going off-screen",
                    "Include smooth acceleration/deceleration"
                ]
            },
            {
                "title": "Exercise 5: Custom Weather Message",
                "description": "Design and implement a custom weather station message",
                "file": "exercise5_weather_station.py",
                "template": "exercise5_template.py",
                "difficulty": "Intermediate",
                "concepts": ["Custom Messages", "Data Structures"],
                "time_limit": 8,
                "requirements": [
                    "Create a weather data structure (temperature, humidity, wind)",
                    "Publish weather data using String messages with JSON",
                    "Generate realistic weather patterns",
                    "Include timestamp and location information",
                    "Demonstrate seasonal weather variations"
                ]
            },
            {
                "title": "Exercise 6: Farm System Integration",
                "description": "Integrate multiple components into a cohesive farm system",
                "file": "exercise6_farm_integration.py",
                "template": "exercise6_template.py",
                "difficulty": "Advanced",
                "concepts": ["System Integration", "Multi-component Systems"],
                "time_limit": 12,
                "requirements": [
                    "Combine sensor data from multiple sources",
                    "Implement decision-making based on sensor readings",
                    "Coordinate irrigation based on soil moisture",
                    "Display a farm dashboard with all system status",
                    "Handle multiple data streams simultaneously"
                ]
            }
        ]
    
    def display_welcome(self):
        """Display welcome message and instructions."""
        print("=" * 80)
        print("🌾 RAISE 2025 - Day 1 Coding Exercises")
        print("🛠️  Practical ROS2 Agricultural Robotics Programming")
        print("=" * 80)
        print()
        print("📋 EXERCISE INFORMATION:")
        print(f"   • Total Exercises: {len(self.exercises)}")
        print(f"   • Difficulty Levels: Beginner, Intermediate, Advanced")
        print(f"   • Total Time: ~60 minutes")
        print(f"   • Programming Language: Python 3")
        print()
        print("🛠️  EXERCISE FORMAT:")
        print("   1. Each exercise includes a template file to start with")
        print("   2. Follow the requirements to complete the implementation")
        print("   3. Test your code using the provided test commands")
        print("   4. Submit when you're satisfied with the solution")
        print()
        print("📝 ASSESSMENT CRITERIA:")
        print("   • Functionality: Does the code meet requirements?")
        print("   • Code Quality: Is it clean, readable, and documented?")
        print("   • Error Handling: Are edge cases handled properly?")
        print("   • Agricultural Context: Is it relevant to farming applications?")
        print()
        input("Press Enter to begin the exercises...")
        print()
    
    def display_exercise(self, exercise_idx):
        """Display a specific exercise."""
        exercise = self.exercises[exercise_idx]
        
        print("=" * 80)
        print(f"📝 {exercise['title']}")
        print("=" * 80)
        print()
        print(f"📖 Description: {exercise['description']}")
        print(f"⏱️  Time Limit: {exercise['time_limit']} minutes")
        print(f"🎯 Difficulty: {exercise['difficulty']}")
        print(f"🔧 Concepts: {', '.join(exercise['concepts'])}")
        print()
        print("📋 REQUIREMENTS:")
        for i, req in enumerate(exercise['requirements'], 1):
            print(f"   {i}. {req}")
        print()
        print("📁 FILES:")
        print(f"   • Template: {exercise['template']}")
        print(f"   • Your Solution: {exercise['file']}")
        print()
        
    def create_template(self, exercise_idx):
        """Create template file for the exercise."""
        exercise = self.exercises[exercise_idx]
        template_path = exercise['template']
        
        if exercise_idx == 0:  # Soil sensor template
            template_content = '''#!/usr/bin/env python3
"""
Exercise 1: Soil Moisture Sensor Publisher
TODO: Complete this implementation to meet the requirements.
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
import random
import time

class SoilMoistureSensor(Node):
    """
    Soil moisture sensor node that publishes readings.
    TODO: Complete the implementation.
    """
    
    def __init__(self):
        super().__init__('soil_moisture_sensor')
        
        # TODO: Create publisher for soil moisture data
        # Hint: Use Float64 message type and '/soil_moisture' topic
        
        # TODO: Create timer to publish data every 2 seconds
        # Hint: Use self.create_timer()
        
        self.get_logger().info("Soil moisture sensor started")
    
    def publish_moisture_reading(self):
        """
        Publish a soil moisture reading.
        TODO: Implement this method.
        """
        # TODO: Generate realistic moisture value (0-100%)
        # TODO: Create Float64 message and publish it
        # TODO: Add logging to show the published value
        pass

def main(args=None):
    """Main function."""
    # TODO: Initialize ROS2
    # TODO: Create and run the sensor node
    # TODO: Handle cleanup properly
    pass

if __name__ == '__main__':
    main()
'''
        
        elif exercise_idx == 1:  # Crop analyzer template
            template_content = '''#!/usr/bin/env python3
"""
Exercise 2: Crop Health Data Subscriber
TODO: Complete this implementation to meet the requirements.
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import json

class CropHealthAnalyzer(Node):
    """
    Crop health data analyzer that processes incoming data.
    TODO: Complete the implementation.
    """
    
    def __init__(self):
        super().__init__('crop_health_analyzer')
        
        # TODO: Create subscriber for crop health data
        # Hint: Subscribe to '/crop_health' topic with String messages
        
        # TODO: Initialize variables for tracking statistics
        
        # TODO: Create timer for summary reports every 10 seconds
        
        self.get_logger().info("Crop health analyzer started")
    
    def health_data_callback(self, msg):
        """
        Process incoming crop health data.
        TODO: Implement this callback.
        """
        # TODO: Parse JSON data from the message
        # TODO: Extract health score and other relevant data
        # TODO: Update statistics
        # TODO: Check for alerts (health < 60)
        pass
    
    def display_summary(self):
        """
        Display summary statistics.
        TODO: Implement this method.
        """
        # TODO: Calculate and display average health scores
        # TODO: Show total number of readings processed
        # TODO: Display any recent alerts
        pass

def main(args=None):
    """Main function."""
    # TODO: Initialize ROS2
    # TODO: Create and run the analyzer node
    # TODO: Handle cleanup properly
    pass

if __name__ == '__main__':
    main()
'''
        
        else:
            # Generic template for other exercises
            template_content = f'''#!/usr/bin/env python3
"""
{exercise['title']}
TODO: Complete this implementation to meet the requirements.

Requirements:
{chr(10).join(f"- {req}" for req in exercise['requirements'])}
"""

import rclpy
from rclpy.node import Node
# TODO: Add additional imports as needed

class ExerciseNode(Node):
    """
    TODO: Implement this node according to the requirements.
    """
    
    def __init__(self):
        super().__init__('exercise_node')
        
        # TODO: Initialize your node
        
        self.get_logger().info("Exercise node started")
    
    # TODO: Add your methods here

def main(args=None):
    """Main function."""
    rclpy.init(args=args)
    
    # TODO: Create and run your node
    
    try:
        # TODO: Add your main logic here
        pass
    except KeyboardInterrupt:
        pass
    finally:
        # TODO: Cleanup
        rclpy.shutdown()

if __name__ == '__main__':
    main()
'''
        
        with open(template_path, 'w') as f:
            f.write(template_content)
        
        print(f"📄 Template created: {template_path}")
        return template_path
    
    def check_solution(self, exercise_idx):
        """Basic check of the solution file."""
        exercise = self.exercises[exercise_idx]
        solution_file = exercise['file']
        
        if not os.path.exists(solution_file):
            return False, f"Solution file {solution_file} not found"
        
        try:
            # Basic syntax check
            with open(solution_file, 'r') as f:
                code = f.read()
            
            compile(code, solution_file, 'exec')
            return True, "Solution file syntax is valid"
            
        except SyntaxError as e:
            return False, f"Syntax error: {e}"
        except Exception as e:
            return False, f"Error checking solution: {e}"
    
    def run_exercise(self, exercise_idx):
        """Run a specific exercise."""
        exercise = self.exercises[exercise_idx]
        
        self.display_exercise(exercise_idx)
        
        # Create template if it doesn't exist
        if not os.path.exists(exercise['template']):
            self.create_template(exercise_idx)
        
        print("🚀 GETTING STARTED:")
        print(f"   1. Copy the template to start your solution:")
        print(f"      cp {exercise['template']} {exercise['file']}")
        print(f"   2. Edit {exercise['file']} to complete the implementation")
        print(f"   3. Test your solution by running it")
        print(f"   4. Return here when ready to submit")
        print()
        
        start_time = time.time()
        
        while True:
            print("📋 OPTIONS:")
            print("   1. Create/Copy template")
            print("   2. Check solution syntax")
            print("   3. Submit solution")
            print("   4. Skip this exercise")
            print("   5. View requirements again")
            print()
            
            choice = input("Enter your choice (1-5): ").strip()
            
            if choice == '1':
                if os.path.exists(exercise['template']):
                    subprocess.run(['cp', exercise['template'], exercise['file']])
                    print(f"✅ Template copied to {exercise['file']}")
                else:
                    self.create_template(exercise_idx)
                    subprocess.run(['cp', exercise['template'], exercise['file']])
                    print(f"✅ Template created and copied to {exercise['file']}")
                print("Now edit the file to complete the implementation.")
            
            elif choice == '2':
                is_valid, message = self.check_solution(exercise_idx)
                if is_valid:
                    print(f"✅ {message}")
                else:
                    print(f"❌ {message}")
            
            elif choice == '3':
                is_valid, message = self.check_solution(exercise_idx)
                if is_valid:
                    elapsed = time.time() - start_time
                    print(f"✅ Solution submitted!")
                    print(f"⏱️  Time taken: {elapsed/60:.1f} minutes")
                    
                    self.completed_exercises.append({
                        'exercise': exercise_idx,
                        'time': elapsed,
                        'status': 'completed'
                    })
                    return True
                else:
                    print(f"❌ Cannot submit: {message}")
                    print("Please fix the issues and try again.")
            
            elif choice == '4':
                print("⏭️  Skipping this exercise.")
                self.completed_exercises.append({
                    'exercise': exercise_idx,
                    'time': 0,
                    'status': 'skipped'
                })
                return False
            
            elif choice == '5':
                self.display_exercise(exercise_idx)
            
            else:
                print("Please enter a valid choice (1-5)")
            
            print()
    
    def display_final_results(self):
        """Display final results of all exercises."""
        print("=" * 80)
        print("📊 CODING EXERCISES RESULTS")
        print("=" * 80)
        print()
        
        completed = len([ex for ex in self.completed_exercises if ex['status'] == 'completed'])
        skipped = len([ex for ex in self.completed_exercises if ex['status'] == 'skipped'])
        
        print(f"✅ Completed: {completed}/{len(self.exercises)}")
        print(f"⏭️  Skipped: {skipped}/{len(self.exercises)}")
        
        total_time = sum(ex['time'] for ex in self.completed_exercises)
        print(f"⏱️  Total Time: {total_time/60:.1f} minutes")
        print()
        
        print("📋 EXERCISE BREAKDOWN:")
        for i, result in enumerate(self.completed_exercises):
            exercise = self.exercises[result['exercise']]
            status_emoji = "✅" if result['status'] == 'completed' else "⏭️"
            time_str = f"{result['time']/60:.1f}m" if result['time'] > 0 else "skipped"
            print(f"   {status_emoji} {exercise['title']} ({time_str})")
        
        print()
        
        # Grade calculation
        completion_rate = completed / len(self.exercises) * 100
        
        if completion_rate >= 90:
            grade = "Excellent"
            emoji = "🌟"
        elif completion_rate >= 75:
            grade = "Good"
            emoji = "👍"
        elif completion_rate >= 50:
            grade = "Satisfactory"
            emoji = "✅"
        else:
            grade = "Needs Improvement"
            emoji = "📚"
        
        print(f"{emoji} Overall Performance: {grade} ({completion_rate:.0f}%)")
        print()
        
        if completion_rate >= 75:
            print("🎉 Great job on the coding exercises!")
            print("✅ You've demonstrated solid practical ROS2 skills")
        else:
            print("📚 Consider completing more exercises to strengthen your skills")
            print("💡 Focus on the core concepts from the modules")
        
        print()
    
    def run_all_exercises(self):
        """Run all coding exercises."""
        self.display_welcome()
        self.start_time = time.time()
        
        for i in range(len(self.exercises)):
            print(f"\n🔄 Starting Exercise {i+1}/{len(self.exercises)}")
            self.run_exercise(i)
            
            if i < len(self.exercises) - 1:
                input("Press Enter to continue to the next exercise...")
        
        self.display_final_results()


def main():
    """Main function to run coding exercises."""
    print("🌾 Welcome to Day 1 Coding Exercises!")
    print()
    
    exercises = CodingExercises()
    
    try:
        exercises.run_all_exercises()
    except KeyboardInterrupt:
        print("\n\n🛑 Exercises interrupted by user.")
        if exercises.completed_exercises:
            exercises.display_final_results()
    except Exception as e:
        print(f"\n❌ An error occurred: {e}")
        print("Please try running the exercises again.")


if __name__ == '__main__':
    main() 