#!/usr/bin/env python3
"""
Day 1 Knowledge Quiz - RAISE 2025
Interactive assessment of ROS2 agricultural robotics concepts.

This quiz tests understanding of all Day 1 modules through multiple-choice questions.
"""

import random
import time
from datetime import datetime


class KnowledgeQuiz:
    """Interactive knowledge quiz for Day 1 ROS2 concepts."""
    
    def __init__(self):
        self.questions = self.load_questions()
        self.score = 0
        self.total_questions = len(self.questions)
        self.start_time = None
        self.answers = []
        
    def load_questions(self):
        """Load quiz questions by category."""
        questions = []
        
        # ROS2 Fundamentals (5 questions)
        questions.extend([
            {
                "category": "ROS2 Fundamentals",
                "question": "What is a ROS2 node?",
                "options": [
                    "A) A file containing Python code",
                    "B) A process that performs computation and communicates with other nodes",
                    "C) A message type definition",
                    "D) A configuration file for ROS2"
                ],
                "correct": 1,
                "explanation": "A ROS2 node is a process that performs computation and communicates with other nodes through topics, services, and actions."
            },
            {
                "category": "ROS2 Fundamentals",
                "question": "Which communication pattern is best for continuous sensor data streaming?",
                "options": [
                    "A) Service/Client",
                    "B) Action Server/Client",
                    "C) Publisher/Subscriber",
                    "D) Parameter Server"
                ],
                "correct": 2,
                "explanation": "Publisher/Subscriber is ideal for continuous data streaming as it allows asynchronous, many-to-many communication."
            },
            {
                "category": "ROS2 Fundamentals",
                "question": "What happens when a ROS2 service call times out?",
                "options": [
                    "A) The service is automatically retried",
                    "B) An exception is raised or timeout callback is triggered",
                    "C) The node shuts down",
                    "D) The message is queued for later delivery"
                ],
                "correct": 1,
                "explanation": "Service calls can timeout, raising exceptions or triggering timeout callbacks that need to be handled properly."
            },
            {
                "category": "ROS2 Fundamentals",
                "question": "In ROS2, what is the purpose of Quality of Service (QoS)?",
                "options": [
                    "A) To measure code quality",
                    "B) To control message delivery reliability and behavior",
                    "C) To optimize CPU usage",
                    "D) To compress message data"
                ],
                "correct": 1,
                "explanation": "QoS settings control message delivery reliability, durability, and other communication behaviors."
            },
            {
                "category": "ROS2 Fundamentals",
                "question": "What is the main difference between services and actions in ROS2?",
                "options": [
                    "A) Services are faster than actions",
                    "B) Actions provide feedback during execution, services don't",
                    "C) Services use TCP, actions use UDP",
                    "D) There is no difference"
                ],
                "correct": 1,
                "explanation": "Actions provide feedback during long-running tasks and can be preempted, while services are simple request-response."
            }
        ])
        
        # Agricultural Applications (8 questions)
        questions.extend([
            {
                "category": "Agricultural Applications",
                "question": "In precision agriculture, what is the primary benefit of sensor networks?",
                "options": [
                    "A) Reducing labor costs",
                    "B) Real-time monitoring and data-driven decision making",
                    "C) Increasing crop yield automatically",
                    "D) Eliminating the need for farmers"
                ],
                "correct": 1,
                "explanation": "Sensor networks provide real-time data for informed decision making in precision agriculture."
            },
            {
                "category": "Agricultural Applications",
                "question": "Which movement pattern is most suitable for harvesting crops planted in rows?",
                "options": [
                    "A) Spiral pattern",
                    "B) Random movement",
                    "C) Row-following pattern",
                    "D) Circular pattern"
                ],
                "correct": 2,
                "explanation": "Row-following patterns are specifically designed for crops planted in organized rows."
            },
            {
                "category": "Agricultural Applications",
                "question": "What is the ideal soil pH range for most crops?",
                "options": [
                    "A) 3.0 - 4.0",
                    "B) 6.0 - 7.0",
                    "C) 8.0 - 9.0",
                    "D) 10.0 - 11.0"
                ],
                "correct": 1,
                "explanation": "Most crops thrive in slightly acidic to neutral soil with pH between 6.0 and 7.0."
            },
            {
                "category": "Agricultural Applications",
                "question": "In automated irrigation, what triggers a watering cycle?",
                "options": [
                    "A) Fixed time schedule only",
                    "B) Soil moisture levels and environmental conditions",
                    "C) Random intervals",
                    "D) Manual activation only"
                ],
                "correct": 1,
                "explanation": "Smart irrigation systems use soil moisture sensors and weather data to optimize watering schedules."
            },
            {
                "category": "Agricultural Applications",
                "question": "What is the purpose of field boundary inspection in agriculture?",
                "options": [
                    "A) Entertainment for robots",
                    "B) Monitoring fence integrity and crop edge health",
                    "C) Wasting time",
                    "D) Confusing the navigation system"
                ],
                "correct": 1,
                "explanation": "Boundary inspection helps monitor fence conditions and assess crop health at field edges."
            },
            {
                "category": "Agricultural Applications",
                "question": "Which sensor data is most critical for crop health monitoring?",
                "options": [
                    "A) Only soil moisture",
                    "B) Only temperature",
                    "C) Combination of soil, environmental, and plant health metrics",
                    "D) Only GPS coordinates"
                ],
                "correct": 2,
                "explanation": "Comprehensive crop health monitoring requires multiple sensor types for accurate assessment."
            },
            {
                "category": "Agricultural Applications",
                "question": "What is the advantage of spiral coverage patterns in field operations?",
                "options": [
                    "A) They look pretty",
                    "B) Efficient area coverage starting from center",
                    "C) They use less fuel",
                    "D) They're easier to program"
                ],
                "correct": 1,
                "explanation": "Spiral patterns provide efficient coverage while minimizing travel time to start operations."
            },
            {
                "category": "Agricultural Applications",
                "question": "In agricultural robotics, what is the primary purpose of custom message types?",
                "options": [
                    "A) To make code more complicated",
                    "B) To organize related sensor data into logical structures",
                    "C) To slow down communication",
                    "D) To increase memory usage"
                ],
                "correct": 1,
                "explanation": "Custom messages organize related agricultural data into logical, reusable structures."
            }
        ])
        
        # Programming Concepts (7 questions)
        questions.extend([
            {
                "category": "Programming Concepts",
                "question": "In ROS2 Python, what is the correct way to create a publisher?",
                "options": [
                    "A) self.publisher = Publisher(String, 'topic', 10)",
                    "B) self.publisher = self.create_publisher(String, 'topic', 10)",
                    "C) self.publisher = rclpy.publisher(String, 'topic', 10)",
                    "D) self.publisher = create_publisher(String, 'topic', 10)"
                ],
                "correct": 1,
                "explanation": "Use self.create_publisher() method within a Node class to create publishers."
            },
            {
                "category": "Programming Concepts",
                "question": "What is the purpose of rclpy.spin() in ROS2 applications?",
                "options": [
                    "A) To rotate the robot",
                    "B) To process callbacks and keep the node alive",
                    "C) To compile the code",
                    "D) To debug the application"
                ],
                "correct": 1,
                "explanation": "rclpy.spin() processes callbacks and keeps the node running to handle incoming messages."
            },
            {
                "category": "Programming Concepts",
                "question": "How should you handle exceptions in ROS2 service calls?",
                "options": [
                    "A) Ignore them",
                    "B) Use try/except blocks and proper logging",
                    "C) Crash the program",
                    "D) Return random values"
                ],
                "correct": 1,
                "explanation": "Always use try/except blocks and proper logging to handle service call exceptions gracefully."
            },
            {
                "category": "Programming Concepts",
                "question": "What is the correct way to stop a ROS2 node cleanly?",
                "options": [
                    "A) Just kill the process",
                    "B) Call destroy_node() and rclpy.shutdown()",
                    "C) Delete all variables",
                    "D) Restart the computer"
                ],
                "correct": 1,
                "explanation": "Proper cleanup involves calling destroy_node() and rclpy.shutdown() to release resources."
            },
            {
                "category": "Programming Concepts",
                "question": "In ROS2, what is the recommended way to handle timer callbacks?",
                "options": [
                    "A) Use infinite loops",
                    "B) Use self.create_timer() with callback functions",
                    "C) Use sleep() statements",
                    "D) Use threading.Timer()"
                ],
                "correct": 1,
                "explanation": "Use self.create_timer() to create timers that integrate properly with the ROS2 executor."
            },
            {
                "category": "Programming Concepts",
                "question": "What is the purpose of message queue size in ROS2 publishers?",
                "options": [
                    "A) To limit CPU usage",
                    "B) To control how many messages can be buffered",
                    "C) To set the message priority",
                    "D) To encrypt the messages"
                ],
                "correct": 1,
                "explanation": "Queue size controls how many messages can be buffered before older messages are dropped."
            },
            {
                "category": "Programming Concepts",
                "question": "How do you properly wait for a service to become available in ROS2?",
                "options": [
                    "A) Just call the service immediately",
                    "B) Use wait_for_service() with timeout",
                    "C) Keep retrying in a loop",
                    "D) Assume it's always available"
                ],
                "correct": 1,
                "explanation": "Use wait_for_service() with appropriate timeout to ensure service availability before calling."
            }
        ])
        
        # System Design (5 questions)
        questions.extend([
            {
                "category": "System Design",
                "question": "What is the best approach for designing a scalable agricultural sensor network?",
                "options": [
                    "A) Put all sensors in one giant node",
                    "B) Create separate nodes for each sensor type with clear interfaces",
                    "C) Use only one sensor",
                    "D) Avoid using ROS2 altogether"
                ],
                "correct": 1,
                "explanation": "Modular design with separate nodes for each sensor type provides scalability and maintainability."
            },
            {
                "category": "System Design",
                "question": "How should you handle system failures in agricultural robotics?",
                "options": [
                    "A) Ignore them and hope for the best",
                    "B) Implement graceful degradation and recovery mechanisms",
                    "C) Shut down the entire system",
                    "D) Replace all hardware"
                ],
                "correct": 1,
                "explanation": "Robust systems implement graceful degradation and recovery to maintain operation during failures."
            },
            {
                "category": "System Design",
                "question": "What is the advantage of using standard message types vs custom messages?",
                "options": [
                    "A) Standard messages are always better",
                    "B) Standard messages ensure interoperability, custom messages provide domain-specific structure",
                    "C) Custom messages are always better",
                    "D) There's no difference"
                ],
                "correct": 1,
                "explanation": "Standard messages ensure interoperability, while custom messages provide domain-specific organization."
            },
            {
                "category": "System Design",
                "question": "In a multi-robot agricultural system, how should robots coordinate?",
                "options": [
                    "A) Each robot works independently with no communication",
                    "B) Through centralized coordination with distributed execution",
                    "C) All robots do the same thing",
                    "D) Random coordination"
                ],
                "correct": 1,
                "explanation": "Centralized coordination with distributed execution provides efficiency while maintaining autonomy."
            },
            {
                "category": "System Design",
                "question": "What is the most important consideration for real-time agricultural systems?",
                "options": [
                    "A) Code beauty",
                    "B) Deterministic response times and reliability",
                    "C) Using the latest technology",
                    "D) Minimizing code lines"
                ],
                "correct": 1,
                "explanation": "Real-time systems require deterministic response times and high reliability for safety and effectiveness."
            }
        ])
        
        return questions
    
    def display_welcome(self):
        """Display welcome message and instructions."""
        print("=" * 80)
        print("🌾 RAISE 2025 - Day 1 Knowledge Quiz")
        print("🤖 ROS2 Agricultural Robotics Assessment")
        print("=" * 80)
        print()
        print("📋 QUIZ INFORMATION:")
        print(f"   • Total Questions: {self.total_questions}")
        print(f"   • Categories: ROS2 Fundamentals, Agricultural Applications,")
        print(f"                Programming Concepts, System Design")
        print(f"   • Time Limit: 15 minutes")
        print(f"   • Passing Score: 75%")
        print()
        print("📝 INSTRUCTIONS:")
        print("   1. Read each question carefully")
        print("   2. Select the best answer (A, B, C, or D)")
        print("   3. Type your answer and press Enter")
        print("   4. You'll see the correct answer after each question")
        print("   5. Final score will be displayed at the end")
        print()
        input("Press Enter to begin the quiz...")
        print()
    
    def ask_question(self, question_data, question_number):
        """Ask a single question and get user response."""
        print(f"Question {question_number}/{self.total_questions} - {question_data['category']}")
        print("-" * 60)
        print(question_data['question'])
        print()
        
        for option in question_data['options']:
            print(f"  {option}")
        
        print()
        
        while True:
            answer = input("Your answer (A, B, C, or D): ").strip().upper()
            if answer in ['A', 'B', 'C', 'D']:
                break
            print("Please enter A, B, C, or D")
        
        # Convert letter to index
        answer_index = ord(answer) - ord('A')
        
        # Check if correct
        is_correct = answer_index == question_data['correct']
        
        if is_correct:
            print("✅ Correct!")
            self.score += 1
        else:
            correct_letter = chr(ord('A') + question_data['correct'])
            print(f"❌ Incorrect. The correct answer is {correct_letter}")
        
        print(f"💡 Explanation: {question_data['explanation']}")
        print()
        
        # Store answer for review
        self.answers.append({
            'question': question_data['question'],
            'user_answer': answer,
            'correct_answer': chr(ord('A') + question_data['correct']),
            'is_correct': is_correct,
            'category': question_data['category']
        })
        
        input("Press Enter to continue...")
        print()
    
    def display_results(self):
        """Display final quiz results."""
        end_time = time.time()
        duration = end_time - self.start_time
        percentage = (self.score / self.total_questions) * 100
        
        print("=" * 80)
        print("📊 QUIZ RESULTS")
        print("=" * 80)
        print()
        print(f"✅ Correct Answers: {self.score}/{self.total_questions}")
        print(f"📈 Score: {percentage:.1f}%")
        print(f"⏱️  Time Taken: {duration/60:.1f} minutes")
        print()
        
        # Grade determination
        if percentage >= 90:
            grade = "Excellent"
            emoji = "🌟"
        elif percentage >= 80:
            grade = "Good"
            emoji = "👍"
        elif percentage >= 70:
            grade = "Satisfactory"
            emoji = "✅"
        else:
            grade = "Needs Improvement"
            emoji = "📚"
        
        print(f"{emoji} Grade: {grade}")
        print()
        
        # Category breakdown
        print("📊 CATEGORY BREAKDOWN:")
        categories = {}
        for answer in self.answers:
            cat = answer['category']
            if cat not in categories:
                categories[cat] = {'correct': 0, 'total': 0}
            categories[cat]['total'] += 1
            if answer['is_correct']:
                categories[cat]['correct'] += 1
        
        for category, stats in categories.items():
            cat_percentage = (stats['correct'] / stats['total']) * 100
            print(f"   {category}: {stats['correct']}/{stats['total']} ({cat_percentage:.1f}%)")
        
        print()
        
        # Pass/Fail
        if percentage >= 75:
            print("🎉 CONGRATULATIONS! You passed the quiz!")
            print("✅ You're ready to proceed to Day 2: AI and Computer Vision")
        else:
            print("📚 You need to review the material before proceeding.")
            print("💡 Focus on the areas where you scored below 70%")
        
        print()
        
        # Recommendations
        print("🔍 RECOMMENDATIONS:")
        weak_categories = [cat for cat, stats in categories.items() 
                          if (stats['correct'] / stats['total']) * 100 < 70]
        
        if weak_categories:
            print("   Review these areas:")
            for cat in weak_categories:
                print(f"   • {cat}")
        else:
            print("   • Great job! You have a solid understanding of all concepts.")
            print("   • Consider exploring advanced topics in each module.")
        
        print()
        
        # Save results
        self.save_results(percentage, duration)
    
    def save_results(self, percentage, duration):
        """Save quiz results to file."""
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        filename = f"quiz_results_{timestamp}.txt"
        
        with open(filename, 'w') as f:
            f.write(f"RAISE 2025 - Day 1 Knowledge Quiz Results\n")
            f.write(f"Date: {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}\n")
            f.write(f"Score: {self.score}/{self.total_questions} ({percentage:.1f}%)\n")
            f.write(f"Duration: {duration/60:.1f} minutes\n\n")
            
            f.write("Detailed Results:\n")
            for i, answer in enumerate(self.answers, 1):
                status = "✅" if answer['is_correct'] else "❌"
                f.write(f"{i}. [{answer['category']}] {status}\n")
                f.write(f"   Question: {answer['question']}\n")
                f.write(f"   Your Answer: {answer['user_answer']}\n")
                f.write(f"   Correct Answer: {answer['correct_answer']}\n\n")
        
        print(f"📄 Results saved to: {filename}")
    
    def run_quiz(self):
        """Run the complete quiz."""
        self.display_welcome()
        
        # Shuffle questions for variety
        random.shuffle(self.questions)
        
        self.start_time = time.time()
        
        # Ask all questions
        for i, question in enumerate(self.questions, 1):
            self.ask_question(question, i)
        
        # Display results
        self.display_results()


def main():
    """Main function to run the knowledge quiz."""
    quiz = KnowledgeQuiz()
    
    try:
        quiz.run_quiz()
    except KeyboardInterrupt:
        print("\n\n🛑 Quiz interrupted by user.")
        print("Your progress has not been saved.")
    except Exception as e:
        print(f"\n❌ An error occurred: {e}")
        print("Please try running the quiz again.")


if __name__ == '__main__':
    main() 