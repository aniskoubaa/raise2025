#!/usr/bin/env python3
"""
Day 1 Integration Challenge - RAISE 2025
Build a complete agricultural monitoring system using all Day 1 concepts.

This challenge tests your ability to integrate multiple ROS2 concepts
into a cohesive agricultural robotics system.
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Float64
import json
import time
import random
from datetime import datetime


class IntegrationChallenge:
    """Interactive integration challenge for Day 1 concepts."""
    
    def __init__(self):
        self.challenge_description = self.load_challenge()
        self.start_time = None
        self.score = 0
        
    def load_challenge(self):
        """Load the integration challenge description."""
        return {
            "title": "Smart Farm Monitoring System",
            "description": """
            Build a complete smart farm monitoring system that demonstrates
            integration of all Day 1 ROS2 concepts in a realistic agricultural scenario.
            """,
            "scenario": """
            🌾 FARM SCENARIO:
            You are developing a monitoring system for a modern smart farm that includes:
            - Multiple soil sensors across different field zones
            - Weather monitoring stations
            - Automated irrigation control system
            - Crop health assessment system
            - Mobile robot for field inspection
            
            Your system must coordinate all these components to make intelligent
            farming decisions and provide real-time monitoring capabilities.
            """,
            "requirements": [
                "Create a central farm management node",
                "Integrate sensor data from multiple sources",
                "Implement intelligent irrigation control",
                "Provide real-time system monitoring",
                "Handle system failures gracefully",
                "Generate actionable farming insights"
            ],
            "components": [
                "FarmManagerNode - Central coordination and decision making",
                "SensorNetwork - Soil and weather data collection",
                "IrrigationController - Smart watering system",
                "CropMonitor - Health assessment and alerts",
                "SystemDashboard - Real-time status display"
            ],
            "time_limit": 30,
            "difficulty": "Advanced"
        }
    
    def display_challenge(self):
        """Display the integration challenge."""
        challenge = self.challenge_description
        
        print("=" * 80)
        print(f"🌾 {challenge['title']}")
        print("=" * 80)
        print()
        print(f"📖 Description: {challenge['description']}")
        print()
        print(challenge['scenario'])
        print()
        print("📋 SYSTEM REQUIREMENTS:")
        for i, req in enumerate(challenge['requirements'], 1):
            print(f"   {i}. {req}")
        print()
        print("🏗️  SYSTEM COMPONENTS:")
        for component in challenge['components']:
            print(f"   • {component}")
        print()
        print(f"⏱️  Time Limit: {challenge['time_limit']} minutes")
        print(f"🎯 Difficulty: {challenge['difficulty']}")
        print()
        
    def display_architecture_guide(self):
        """Display system architecture guidance."""
        print("🏗️  SYSTEM ARCHITECTURE GUIDE")
        print("=" * 60)
        print()
        print("📊 DATA FLOW:")
        print("   Sensors → Farm Manager → Decision Engine → Control Systems")
        print()
        print("🔄 COMMUNICATION PATTERNS:")
        print("   • Publishers/Subscribers: Sensor data streaming")
        print("   • Services: Control system commands")
        print("   • Actions: Long-running field operations")
        print("   • Custom Messages: Structured agricultural data")
        print()
        print("🎯 KEY INTEGRATION POINTS:")
        print("   1. Sensor data aggregation and processing")
        print("   2. Decision-making based on multiple data sources")
        print("   3. Coordinated control of farm systems")
        print("   4. Real-time monitoring and alerting")
        print("   5. System health and failure management")
        print()
        
    def provide_implementation_template(self):
        """Provide a starting template for the challenge."""
        template_content = '''#!/usr/bin/env python3
"""
Smart Farm Monitoring System - Integration Challenge
TODO: Complete this implementation to meet all requirements.

Your task is to build a complete farm monitoring system that integrates
all the concepts learned in Day 1 modules.
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Float64
import json
import time
import random
from datetime import datetime

class FarmManagerNode(Node):
    """
    Central farm management node that coordinates all farm systems.
    TODO: Complete this implementation.
    """
    
    def __init__(self):
        super().__init__('farm_manager')
        
        # TODO: Set up publishers for system status and alerts
        
        # TODO: Set up subscribers for sensor data
        
        # TODO: Set up service clients for irrigation control
        
        # TODO: Initialize data storage and decision variables
        
        # TODO: Set up timers for periodic tasks
        
        self.get_logger().info("🌾 Farm Manager started")
    
    def process_sensor_data(self, sensor_data):
        """
        Process incoming sensor data and make decisions.
        TODO: Implement intelligent data processing.
        """
        # TODO: Analyze soil moisture, temperature, pH levels
        # TODO: Evaluate crop health indicators
        # TODO: Check weather conditions
        # TODO: Generate alerts for critical conditions
        # TODO: Make irrigation decisions
        pass
    
    def generate_farm_report(self):
        """
        Generate comprehensive farm status report.
        TODO: Implement reporting system.
        """
        # TODO: Aggregate all sensor data
        # TODO: Calculate system performance metrics
        # TODO: Identify areas needing attention
        # TODO: Generate actionable recommendations
        pass

class SensorNetwork(Node):
    """
    Simulated sensor network for soil and weather data.
    TODO: Complete this implementation.
    """
    
    def __init__(self):
        super().__init__('sensor_network')
        
        # TODO: Set up publishers for different sensor types
        
        # TODO: Initialize sensor simulation parameters
        
        # TODO: Set up timers for data publishing
        
        self.get_logger().info("📡 Sensor Network started")
    
    def publish_sensor_data(self):
        """
        Publish simulated sensor data.
        TODO: Implement realistic sensor simulation.
        """
        # TODO: Generate soil moisture data
        # TODO: Generate weather data
        # TODO: Generate crop health data
        # TODO: Add realistic variations and patterns
        pass

class IrrigationController(Node):
    """
    Smart irrigation control system.
    TODO: Complete this implementation.
    """
    
    def __init__(self):
        super().__init__('irrigation_controller')
        
        # TODO: Set up service server for irrigation commands
        
        # TODO: Set up publishers for irrigation status
        
        # TODO: Initialize irrigation system state
        
        self.get_logger().info("💧 Irrigation Controller started")
    
    def handle_irrigation_request(self, request, response):
        """
        Handle irrigation control requests.
        TODO: Implement irrigation logic.
        """
        # TODO: Validate irrigation parameters
        # TODO: Check system safety conditions
        # TODO: Execute irrigation commands
        # TODO: Return status and confirmation
        pass

class SystemDashboard(Node):
    """
    Real-time system monitoring dashboard.
    TODO: Complete this implementation.
    """
    
    def __init__(self):
        super().__init__('system_dashboard')
        
        # TODO: Set up subscribers for all system data
        
        # TODO: Initialize dashboard display
        
        # TODO: Set up timer for dashboard updates
        
        self.get_logger().info("📊 System Dashboard started")
    
    def update_dashboard(self):
        """
        Update and display system dashboard.
        TODO: Implement comprehensive dashboard.
        """
        # TODO: Display current sensor readings
        # TODO: Show system status and alerts
        # TODO: Display recent activities and decisions
        # TODO: Show system performance metrics
        pass

def main(args=None):
    """
    Main function to run the integrated farm system.
    TODO: Complete the system orchestration.
    """
    rclpy.init(args=args)
    
    try:
        # TODO: Create and start all system components
        # TODO: Implement proper coordination
        # TODO: Handle multi-node execution
        pass
    except KeyboardInterrupt:
        print("\\n🛑 Farm system stopped by user")
    finally:
        # TODO: Cleanup all nodes
        rclpy.shutdown()

if __name__ == '__main__':
    main()

# ASSESSMENT CRITERIA:
# 1. System Integration (25%): How well do components work together?
# 2. Data Processing (25%): Quality of sensor data analysis and decision making
# 3. Error Handling (20%): Robustness and graceful failure management  
# 4. Code Quality (20%): Clean, readable, well-documented code
# 5. Agricultural Relevance (10%): Practical applicability to real farming

# BONUS POINTS:
# - Innovative features beyond basic requirements
# - Advanced error recovery mechanisms
# - Sophisticated decision-making algorithms
# - Creative visualization or reporting features
'''
        
        with open('integration_challenge_template.py', 'w') as f:
            f.write(template_content)
        
        print("📄 Integration challenge template created: integration_challenge_template.py")
        print("💡 Copy this file to 'my_farm_system.py' and complete the implementation")
        return template_content
    
    def display_evaluation_criteria(self):
        """Display evaluation criteria for the challenge."""
        print("📊 EVALUATION CRITERIA")
        print("=" * 50)
        print()
        print("🎯 SCORING BREAKDOWN (100 points total):")
        print("   • System Integration (25 points)")
        print("     - Component coordination")
        print("     - Data flow between nodes")
        print("     - Service/topic usage")
        print()
        print("   • Data Processing (25 points)")
        print("     - Sensor data analysis")
        print("     - Decision-making logic")
        print("     - Alert generation")
        print()
        print("   • Error Handling (20 points)")
        print("     - Graceful failure handling")
        print("     - System recovery")
        print("     - Input validation")
        print()
        print("   • Code Quality (20 points)")
        print("     - Clean, readable code")
        print("     - Proper documentation")
        print("     - Good software practices")
        print()
        print("   • Agricultural Relevance (10 points)")
        print("     - Real-world applicability")
        print("     - Domain knowledge demonstration")
        print("     - Practical value")
        print()
        print("🌟 BONUS OPPORTUNITIES (+10 points):")
        print("   • Innovative features")
        print("   • Advanced algorithms")
        print("   • Creative solutions")
        print("   • Exceptional integration")
        print()
        
    def display_testing_guide(self):
        """Display testing and validation guide."""
        print("🧪 TESTING YOUR SOLUTION")
        print("=" * 50)
        print()
        print("✅ FUNCTIONALITY TESTS:")
        print("   1. All nodes start without errors")
        print("   2. Sensor data is published correctly")
        print("   3. Farm manager processes data appropriately")
        print("   4. Irrigation system responds to commands")
        print("   5. Dashboard displays current status")
        print("   6. System handles interruption gracefully")
        print()
        print("🔍 INTEGRATION TESTS:")
        print("   1. Data flows between all components")
        print("   2. Decision-making triggers appropriate actions")
        print("   3. Alerts are generated for critical conditions")
        print("   4. System maintains state consistency")
        print("   5. Performance is acceptable under load")
        print()
        print("📋 VALIDATION CHECKLIST:")
        print("   □ Code runs without syntax errors")
        print("   □ All required components are implemented")
        print("   □ System demonstrates agricultural decision-making")
        print("   □ Error handling prevents crashes")
        print("   □ Code is well-documented and readable")
        print("   □ Real-time monitoring is functional")
        print()
        
    def run_challenge(self):
        """Run the integration challenge."""
        print("🌾 Welcome to the Day 1 Integration Challenge!")
        print()
        
        self.display_challenge()
        
        while True:
            print("📋 CHALLENGE OPTIONS:")
            print("   1. View system architecture guide")
            print("   2. Get implementation template")
            print("   3. View evaluation criteria")
            print("   4. View testing guide")
            print("   5. Submit your solution")
            print("   6. Exit challenge")
            print()
            
            choice = input("Enter your choice (1-6): ").strip()
            
            if choice == '1':
                self.display_architecture_guide()
            elif choice == '2':
                self.provide_implementation_template()
            elif choice == '3':
                self.display_evaluation_criteria()
            elif choice == '4':
                self.display_testing_guide()
            elif choice == '5':
                self.submit_solution()
            elif choice == '6':
                print("👋 Exiting integration challenge. Good luck!")
                break
            else:
                print("Please enter a valid choice (1-6)")
            
            print()
            input("Press Enter to continue...")
            print()
    
    def submit_solution(self):
        """Handle solution submission."""
        print("📤 SOLUTION SUBMISSION")
        print("=" * 40)
        print()
        
        solution_file = input("Enter your solution filename (e.g., my_farm_system.py): ").strip()
        
        if not solution_file:
            print("❌ Please provide a filename")
            return
        
        try:
            import os
            if not os.path.exists(solution_file):
                print(f"❌ File {solution_file} not found")
                return
            
            # Basic syntax check
            with open(solution_file, 'r') as f:
                code = f.read()
            
            compile(code, solution_file, 'exec')
            
            print("✅ Solution file syntax is valid")
            print()
            print("📋 SUBMISSION CHECKLIST:")
            print("   Please confirm your solution includes:")
            
            checklist_items = [
                "Central farm management node",
                "Multiple sensor data sources",
                "Irrigation control system",
                "Real-time monitoring dashboard",
                "Error handling and recovery",
                "Agricultural decision-making logic"
            ]
            
            confirmed_items = 0
            for item in checklist_items:
                response = input(f"   ✓ {item}? (y/n): ").strip().lower()
                if response == 'y':
                    confirmed_items += 1
            
            completion_rate = (confirmed_items / len(checklist_items)) * 100
            
            print()
            print(f"📊 Checklist Completion: {completion_rate:.0f}%")
            
            if completion_rate >= 80:
                print("🎉 Great job! Your solution appears comprehensive.")
                print("✅ Solution submitted successfully!")
                
                # Save submission info
                timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
                submission_file = f"submission_{timestamp}.txt"
                
                with open(submission_file, 'w') as f:
                    f.write(f"RAISE 2025 - Day 1 Integration Challenge Submission\n")
                    f.write(f"Date: {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}\n")
                    f.write(f"Solution File: {solution_file}\n")
                    f.write(f"Checklist Completion: {completion_rate:.0f}%\n")
                    f.write(f"Items Confirmed: {confirmed_items}/{len(checklist_items)}\n")
                
                print(f"📄 Submission record saved: {submission_file}")
                
            else:
                print("📚 Your solution may be incomplete.")
                print("💡 Consider implementing the missing components before final submission.")
            
        except SyntaxError as e:
            print(f"❌ Syntax error in your solution: {e}")
        except Exception as e:
            print(f"❌ Error checking solution: {e}")


def main():
    """Main function to run the integration challenge."""
    challenge = IntegrationChallenge()
    
    try:
        challenge.run_challenge()
    except KeyboardInterrupt:
        print("\n\n🛑 Challenge interrupted by user.")
    except Exception as e:
        print(f"\n❌ An error occurred: {e}")


if __name__ == '__main__':
    main() 