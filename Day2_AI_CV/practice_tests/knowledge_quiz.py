#!/usr/bin/env python3
"""
RAISE 2025 - Day 2 Knowledge Quiz
AI & Computer Vision for Agricultural Robotics

This quiz tests understanding of:
- Computer Vision fundamentals
- YOLO object detection
- Machine Learning concepts
- Large Language Models
- ROS2 + CV integration
- Agricultural AI applications

Author: RAISE 2025 Team
Date: July 2025
"""

import random
import time
from datetime import datetime, timedelta
import json

class Day2KnowledgeQuiz:
    def __init__(self):
        print("🧠 Day 2 Knowledge Quiz: AI & Computer Vision")
        print("=" * 50)
        
        # Quiz configuration
        self.total_questions = 25
        self.time_limit_minutes = 20
        self.passing_score = 0.70
        
        # Initialize quiz state
        self.current_question = 0
        self.score = 0
        self.answers = []
        self.start_time = None
        self.questions = self.load_questions()
        
        print(f"📊 Questions: {self.total_questions}")
        print(f"⏰ Time Limit: {self.time_limit_minutes} minutes")
        print(f"🎯 Passing Score: {self.passing_score*100:.0f}%")

    def load_questions(self):
        """Load quiz questions organized by topic"""
        questions = []
        
        # Module 1: YOLO Basics (5 questions)
        questions.extend([
            {
                "topic": "YOLO Basics",
                "question": "What is the main advantage of YOLO over traditional object detection methods?",
                "options": [
                    "Higher accuracy on small objects",
                    "Real-time detection in a single forward pass",
                    "Better handling of overlapping objects",
                    "Requires less training data"
                ],
                "correct": 1,
                "explanation": "YOLO's key innovation is performing object detection in a single forward pass, enabling real-time performance."
            },
            {
                "topic": "YOLO Basics",
                "question": "In agricultural applications, what does a YOLO confidence score of 0.85 indicate?",
                "options": [
                    "The model is 85% certain about the object class",
                    "The bounding box covers 85% of the object",
                    "The model found 85% of objects in the image",
                    "The training accuracy was 85%"
                ],
                "correct": 0,
                "explanation": "Confidence score represents the model's certainty about the predicted class, not geometric or performance metrics."
            },
            {
                "topic": "YOLO Basics",
                "question": "Which YOLO variant would be best for a real-time agricultural robot with limited computing power?",
                "options": [
                    "YOLOv8x (extra large)",
                    "YOLOv8l (large)", 
                    "YOLOv8m (medium)",
                    "YOLOv8n (nano)"
                ],
                "correct": 3,
                "explanation": "YOLOv8n (nano) is the smallest and fastest variant, ideal for resource-constrained real-time applications."
            },
            {
                "topic": "YOLO Basics",
                "question": "What is the difference between object detection and image classification?",
                "options": [
                    "Detection is faster than classification",
                    "Classification only works on single objects",
                    "Detection locates objects with bounding boxes, classification only identifies",
                    "Classification requires more training data"
                ],
                "correct": 2,
                "explanation": "Object detection both identifies objects AND locates them with bounding boxes, while classification only identifies the main object."
            },
            {
                "topic": "YOLO Basics",
                "question": "In a tomato greenhouse, YOLO detects 'person', 'truck', and 'potted plant'. Which has highest agricultural relevance?",
                "options": [
                    "person (workers in greenhouse)",
                    "truck (farm vehicles)",
                    "potted plant (directly agriculture-related)",
                    "All have equal relevance"
                ],
                "correct": 2,
                "explanation": "While all objects can be relevant, 'potted plant' is most directly related to agricultural monitoring and analysis."
            }
        ])
        
        # Module 2: Training Concepts (6 questions)
        questions.extend([
            {
                "topic": "Training Concepts",
                "question": "What is the typical dataset split for machine learning projects?",
                "options": [
                    "50% training, 25% validation, 25% testing",
                    "60% training, 20% validation, 20% testing", 
                    "70% training, 20% validation, 10% testing",
                    "80% training, 10% validation, 10% testing"
                ],
                "correct": 2,
                "explanation": "The standard split is 70% training, 20% validation, 10% testing to ensure sufficient training data while maintaining proper evaluation."
            },
            {
                "topic": "Training Concepts",
                "question": "What is transfer learning in the context of agricultural AI?",
                "options": [
                    "Moving data between different farms",
                    "Using a pre-trained model and fine-tuning it for agricultural tasks",
                    "Transferring AI models between different robots",
                    "Converting images between different formats"
                ],
                "correct": 1,
                "explanation": "Transfer learning uses a model pre-trained on general data (like ImageNet) and fine-tunes it for specific agricultural tasks."
            },
            {
                "topic": "Training Concepts", 
                "question": "Which data augmentation technique would be most effective for tomato disease detection?",
                "options": [
                    "Rotation and horizontal flipping",
                    "Extreme zoom and cropping",
                    "Color inversion and negative images",
                    "Adding text overlays"
                ],
                "correct": 0,
                "explanation": "Rotation and horizontal flipping simulate natural viewing angles of leaves without destroying the visual disease patterns."
            },
            {
                "topic": "Training Concepts",
                "question": "If a tomato disease model has 90% training accuracy but 65% validation accuracy, what's the problem?",
                "options": [
                    "The model is underfitting",
                    "The model is overfitting",
                    "The dataset is too small",
                    "The model architecture is wrong"
                ],
                "correct": 1,
                "explanation": "Large gap between training and validation accuracy indicates overfitting - the model memorized training data but doesn't generalize."
            },
            {
                "topic": "Training Concepts",
                "question": "What does a precision score of 0.85 mean for disease detection?",
                "options": [
                    "85% of all diseases were detected",
                    "85% of detected diseases were actually diseases",
                    "The model is 85% accurate overall",
                    "85% of images were processed correctly"
                ],
                "correct": 1,
                "explanation": "Precision = True Positives / (True Positives + False Positives), meaning 85% of positive detections were correct."
            },
            {
                "topic": "Training Concepts",
                "question": "Why is recall important for agricultural disease detection?",
                "options": [
                    "It measures model speed",
                    "It measures cost effectiveness", 
                    "It measures how many actual diseases were caught (avoiding missed diseases)",
                    "It measures data quality"
                ],
                "correct": 2,
                "explanation": "High recall means catching most actual diseases, which is critical to prevent crop losses from missed detections."
            }
        ])
        
        # Module 3: Gemini Vision API (6 questions)
        questions.extend([
            {
                "topic": "Gemini Vision",
                "question": "What makes Large Language Models like Gemini Vision different from traditional computer vision?",
                "options": [
                    "They process images faster",
                    "They can understand and explain what they see in natural language",
                    "They require less computational power",
                    "They work better in low light conditions"
                ],
                "correct": 1,
                "explanation": "LLMs can provide detailed explanations and reasoning about visual content, not just labels or classifications."
            },
            {
                "topic": "Gemini Vision",
                "question": "Which prompt would be most effective for agricultural analysis?",
                "options": [
                    "'What's this?'",
                    "'Good or bad?'",
                    "'Analyze this tomato leaf image. Identify any diseases, provide confidence level, and suggest treatment options.'",
                    "'Tell me everything about this image.'"
                ],
                "correct": 2,
                "explanation": "Specific, context-rich prompts with clear requirements produce the most useful and actionable responses."
            },
            {
                "topic": "Gemini Vision",
                "question": "What is the approximate cost per image analysis using Gemini Vision API?",
                "options": [
                    "$0.0025 per image",
                    "$0.025 per image", 
                    "$0.25 per image",
                    "$2.50 per image"
                ],
                "correct": 0,
                "explanation": "Gemini Vision API costs approximately $0.0025 per image, making it affordable for most agricultural applications."
            },
            {
                "topic": "Gemini Vision",
                "question": "What's the main limitation of using Gemini Vision for real-time agricultural robots?",
                "options": [
                    "Poor accuracy on plant images",
                    "High cost per analysis",
                    "Slower response time (2-5 seconds) compared to local models",
                    "Limited to specific plant types"
                ],
                "correct": 2,
                "explanation": "API calls take 2-5 seconds, which may be too slow for real-time robotic applications requiring instant responses."
            },
            {
                "topic": "Gemini Vision",
                "question": "How should you handle Gemini API keys in agricultural applications?",
                "options": [
                    "Hardcode them in the source code",
                    "Share them with the development team",
                    "Store them in environment variables and keep them secret",
                    "Post them in documentation for reference"
                ],
                "correct": 2,
                "explanation": "API keys should be stored securely in environment variables and never shared or committed to version control."
            },
            {
                "topic": "Gemini Vision",
                "question": "When would you choose Gemini Vision over YOLO for agricultural analysis?",
                "options": [
                    "When you need real-time processing",
                    "When you need detailed explanations and expert-level analysis", 
                    "When you have limited internet connectivity",
                    "When processing thousands of images per hour"
                ],
                "correct": 1,
                "explanation": "Gemini excels at providing detailed, expert-level analysis and explanations that go beyond simple object detection."
            }
        ])
        
        # Module 4: ROS2 + CV Integration (5 questions)
        questions.extend([
            {
                "topic": "ROS2 + CV",
                "question": "What is the purpose of cv_bridge in ROS2 computer vision applications?",
                "options": [
                    "To compress images for faster transmission",
                    "To convert between ROS image messages and OpenCV/NumPy arrays",
                    "To synchronize multiple camera feeds",
                    "To calibrate camera parameters"
                ],
                "correct": 1,
                "explanation": "cv_bridge converts between ROS2 sensor_msgs/Image format and OpenCV arrays that AI models can process."
            },
            {
                "topic": "ROS2 + CV",
                "question": "In an agricultural robot, what topic would typically carry the camera feed?",
                "options": [
                    "/cmd_vel",
                    "/camera/image_raw",
                    "/plant_detections",
                    "/spray_control"
                ],
                "correct": 1,
                "explanation": "/camera/image_raw is the standard ROS2 topic name for unprocessed camera images."
            },
            {
                "topic": "ROS2 + CV",
                "question": "What's the recommended approach for computationally expensive AI processing in ROS2?",
                "options": [
                    "Process every single frame",
                    "Use the main robot control loop",
                    "Use separate dedicated vision nodes with appropriate threading",
                    "Block all other operations until vision completes"
                ],
                "correct": 2,
                "explanation": "Separate vision nodes with proper threading prevent blocking other robot operations and allow optimization."
            },
            {
                "topic": "ROS2 + CV",
                "question": "How should an agricultural robot respond when vision detects plant disease?",
                "options": [
                    "Continue normal operations",
                    "Stop movement, activate treatment system, log location",
                    "Only log the detection",
                    "Ignore the detection and continue"
                ],
                "correct": 1,
                "explanation": "A coordinated response involves stopping, treating the problem, and logging for follow-up action."
            },
            {
                "topic": "ROS2 + CV",
                "question": "What message type would you use to command a robot to start spraying?",
                "options": [
                    "sensor_msgs/Image",
                    "geometry_msgs/Twist",
                    "std_msgs/Bool",
                    "std_msgs/String"
                ],
                "correct": 2,
                "explanation": "std_msgs/Bool is perfect for simple on/off commands like activating a spray system."
            }
        ])
        
        # Agricultural Applications (3 questions)
        questions.extend([
            {
                "topic": "Agricultural Applications",
                "question": "What's the main benefit of AI-powered precision agriculture?",
                "options": [
                    "Reduced labor costs only",
                    "Faster harvesting",
                    "Targeted treatment reducing chemical usage by up to 80%",
                    "Larger crop yields only"
                ],
                "correct": 2,
                "explanation": "Precision agriculture's key benefit is targeted treatment, dramatically reducing chemical usage while maintaining effectiveness."
            },
            {
                "topic": "Agricultural Applications", 
                "question": "In a smart greenhouse system, what sequence of operations makes most sense for disease detection?",
                "options": [
                    "Camera → AI Analysis → Alert → Human Inspection → Treatment",
                    "Treatment → Camera → AI Analysis → Alert",
                    "Human Inspection → Camera → Treatment → AI Analysis",
                    "AI Analysis → Camera → Treatment → Alert"
                ],
                "correct": 0,
                "explanation": "The logical flow starts with sensing, then analysis, then human verification, followed by treatment action."
            },
            {
                "topic": "Agricultural Applications",
                "question": "What makes computer vision particularly valuable for early disease detection?",
                "options": [
                    "It's cheaper than human inspection",
                    "It can detect diseases before visible to human eye using spectral analysis",
                    "It works faster than humans",
                    "It doesn't require training"
                ],
                "correct": 1,
                "explanation": "Advanced computer vision can detect subtle spectral changes that indicate disease before symptoms are visible to humans."
            }
        ])
        
        # Shuffle questions for each quiz attempt
        random.shuffle(questions)
        return questions[:self.total_questions]

    def start_quiz(self):
        """Start the timed quiz"""
        print(f"\n🎬 Starting Quiz - {self.total_questions} Questions")
        print("=" * 40)
        print("📋 Instructions:")
        print("  • Read each question carefully")
        print("  • Select the best answer (a, b, c, or d)")
        print("  • Type 'skip' to skip a question")
        print("  • Type 'time' to check remaining time")
        print("  • Type 'quit' to exit (progress will be lost)")
        
        input(f"\nPress Enter to start your {self.time_limit_minutes}-minute timer...")
        
        self.start_time = datetime.now()
        
        for i, question in enumerate(self.questions):
            self.current_question = i + 1
            
            # Check time limit
            if self.check_time_limit():
                print(f"\n⏰ Time limit exceeded! Quiz ended at question {self.current_question}")
                break
            
            answer = self.ask_question(question)
            self.answers.append({
                'question': question,
                'answer': answer,
                'correct': answer == question['correct'] if answer is not None else False
            })
        
        self.show_results()

    def check_time_limit(self):
        """Check if time limit has been exceeded"""
        if self.start_time is None:
            return False
        
        elapsed = datetime.now() - self.start_time
        return elapsed.total_seconds() > (self.time_limit_minutes * 60)

    def get_remaining_time(self):
        """Get remaining time in minutes and seconds"""
        if self.start_time is None:
            return self.time_limit_minutes, 0
        
        elapsed = datetime.now() - self.start_time
        remaining_seconds = (self.time_limit_minutes * 60) - elapsed.total_seconds()
        
        if remaining_seconds <= 0:
            return 0, 0
        
        minutes = int(remaining_seconds // 60)
        seconds = int(remaining_seconds % 60)
        return minutes, seconds

    def ask_question(self, question):
        """Ask a single question and get user's answer"""
        print(f"\n📝 Question {self.current_question}/{self.total_questions}")
        print(f"📚 Topic: {question['topic']}")
        
        # Show remaining time
        mins, secs = self.get_remaining_time()
        print(f"⏰ Time remaining: {mins:02d}:{secs:02d}")
        
        print(f"\n❓ {question['question']}")
        print()
        
        # Display options
        option_letters = ['a', 'b', 'c', 'd']
        for i, option in enumerate(question['options']):
            print(f"   {option_letters[i]}) {option}")
        
        # Get user input
        while True:
            answer = input(f"\nYour answer (a/b/c/d): ").strip().lower()
            
            if answer == 'quit':
                print("🚪 Quiz terminated by user.")
                exit()
            elif answer == 'time':
                mins, secs = self.get_remaining_time()
                print(f"⏰ Time remaining: {mins:02d}:{secs:02d}")
                continue
            elif answer == 'skip':
                print("⏭️ Question skipped.")
                return None
            elif answer in option_letters:
                return option_letters.index(answer)
            else:
                print("❌ Invalid input. Please enter a, b, c, d, 'skip', 'time', or 'quit'")

    def show_results(self):
        """Display quiz results and analysis"""
        print(f"\n🎉 Quiz Complete!")
        print("=" * 30)
        
        # Calculate score
        correct_answers = sum(1 for ans in self.answers if ans['correct'])
        total_answered = len([ans for ans in self.answers if ans['answer'] is not None])
        
        if total_answered > 0:
            score_percentage = correct_answers / len(self.answers)
            answered_percentage = correct_answers / total_answered
        else:
            score_percentage = 0
            answered_percentage = 0
        
        # Time analysis
        if self.start_time:
            total_time = datetime.now() - self.start_time
            avg_time_per_question = total_time.total_seconds() / len(self.answers)
        else:
            total_time = timedelta(0)
            avg_time_per_question = 0
        
        # Display overall results
        print(f"📊 Overall Results:")
        print(f"   Correct Answers: {correct_answers}/{len(self.answers)}")
        print(f"   Score: {score_percentage*100:.1f}%")
        if total_answered < len(self.answers):
            print(f"   Answered: {total_answered}/{len(self.answers)} questions")
            print(f"   Score (answered only): {answered_percentage*100:.1f}%")
        
        print(f"   Time Used: {str(total_time).split('.')[0]}")
        print(f"   Average per Question: {avg_time_per_question:.1f} seconds")
        
        # Pass/Fail determination
        if score_percentage >= self.passing_score:
            print(f"\n✅ PASSED! (Required: {self.passing_score*100:.0f}%)")
            grade = self.get_letter_grade(score_percentage)
            print(f"🏆 Grade: {grade}")
        else:
            print(f"\n❌ Not Passed (Required: {self.passing_score*100:.0f}%)")
            print("💡 Recommendation: Review the topics below and retake the quiz")
        
        # Topic analysis
        self.show_topic_analysis()
        
        # Review wrong answers
        self.show_wrong_answers()
        
        # Study recommendations
        self.show_study_recommendations(score_percentage)

    def get_letter_grade(self, score):
        """Convert percentage to letter grade"""
        if score >= 0.90:
            return "A (Excellent)"
        elif score >= 0.80:
            return "B (Good)"
        elif score >= 0.70:
            return "C (Satisfactory)"
        elif score >= 0.60:
            return "D (Needs Improvement)"
        else:
            return "F (Insufficient)"

    def show_topic_analysis(self):
        """Show performance breakdown by topic"""
        print(f"\n📈 Topic Analysis:")
        print("-" * 25)
        
        # Group answers by topic
        topic_stats = {}
        for ans in self.answers:
            topic = ans['question']['topic']
            if topic not in topic_stats:
                topic_stats[topic] = {'correct': 0, 'total': 0}
            topic_stats[topic]['total'] += 1
            if ans['correct']:
                topic_stats[topic]['correct'] += 1
        
        # Display topic performance
        for topic, stats in topic_stats.items():
            percentage = (stats['correct'] / stats['total']) * 100 if stats['total'] > 0 else 0
            status = "✅" if percentage >= 70 else "⚠️" if percentage >= 50 else "❌"
            print(f"   {status} {topic}: {stats['correct']}/{stats['total']} ({percentage:.0f}%)")

    def show_wrong_answers(self):
        """Show incorrect answers with explanations"""
        wrong_answers = [ans for ans in self.answers if not ans['correct'] and ans['answer'] is not None]
        
        if wrong_answers:
            print(f"\n📝 Review Incorrect Answers:")
            print("-" * 30)
            
            for i, ans in enumerate(wrong_answers, 1):
                question = ans['question']
                user_answer = ans['answer']
                correct_answer = question['correct']
                
                print(f"\n{i}. Topic: {question['topic']}")
                print(f"   Question: {question['question']}")
                print(f"   Your answer: {chr(97 + user_answer)}) {question['options'][user_answer]}")
                print(f"   Correct answer: {chr(97 + correct_answer)}) {question['options'][correct_answer]}")
                print(f"   Explanation: {question['explanation']}")

    def show_study_recommendations(self, score):
        """Provide study recommendations based on performance"""
        print(f"\n💡 Study Recommendations:")
        print("-" * 25)
        
        if score >= 0.90:
            print("🌟 Excellent work! You have a strong understanding of AI/CV for agriculture.")
            print("   Consider exploring advanced topics and real-world applications.")
        elif score >= 0.70:
            print("👍 Good foundation! Review the topics where you scored below 70%.")
            print("   Focus on practical applications and hands-on exercises.")
        else:
            print("📚 Recommended study plan:")
            print("   1. Re-read all module documentation")
            print("   2. Run all hands-on demos again")
            print("   3. Practice with the QUICKSTART guides")
            print("   4. Focus on topics with low scores")
            print("   5. Retake the quiz when ready")
        
        print(f"\n📖 Additional Resources:")
        print("   • Module READMEs and QUICKSTART guides")
        print("   • Hands-on demo scripts in each module")
        print("   • Practice mode: python3 knowledge_quiz.py --practice")
        print("   • Integration exercises in coding_exercises/")

def main():
    """Main function"""
    import sys
    
    practice_mode = '--practice' in sys.argv
    
    if practice_mode:
        print("🎮 Practice Mode: Unlimited time, immediate feedback")
        print("=" * 50)
    
    quiz = Day2KnowledgeQuiz()
    
    if practice_mode:
        # Practice mode modifications
        quiz.time_limit_minutes = 999  # Essentially unlimited
        print("⏰ Time limit disabled for practice")
    
    try:
        quiz.start_quiz()
    except KeyboardInterrupt:
        print(f"\n\n⏹️ Quiz interrupted by user")
        print(f"Progress: {quiz.current_question}/{quiz.total_questions} questions completed")
    except Exception as e:
        print(f"\n❌ Error occurred: {e}")
        print("Please restart the quiz or contact support")

if __name__ == "__main__":
    main() 