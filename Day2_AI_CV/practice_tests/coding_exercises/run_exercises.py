#!/usr/bin/env python3
"""
RAISE 2025 - Day 2 Coding Exercises Runner
Runs all 6 coding exercises with timing and automatic grading.

Usage:
    python3 run_exercises.py           # Timed assessment mode
    python3 run_exercises.py --practice # Practice mode (no timer)
    python3 run_exercises.py --help     # Show help
"""

import time
import sys
import os
import importlib.util
from datetime import datetime, timedelta
import traceback

class CodingExercisesRunner:
    def __init__(self, practice_mode=False):
        self.practice_mode = practice_mode
        self.time_limit_minutes = 45 if not practice_mode else 999
        self.exercises = [
            {
                'name': 'Exercise 1: YOLO Detection',
                'file': 'exercise_1_yolo_detection.py',
                'points': 5,
                'time_minutes': 8,
                'description': 'Run object detection on agricultural images'
            },
            {
                'name': 'Exercise 2: Model Evaluation',
                'file': 'exercise_2_model_evaluation.py',
                'points': 8,
                'time_minutes': 12,
                'description': 'Calculate accuracy, precision, recall metrics'
            },
            {
                'name': 'Exercise 3: Data Preparation',
                'file': 'exercise_3_data_preparation.py',
                'points': 7,
                'time_minutes': 10,
                'description': 'Implement data augmentation techniques'
            },
            {
                'name': 'Exercise 4: Prompt Engineering',
                'file': 'exercise_4_prompt_engineering.py',
                'points': 8,
                'time_minutes': 10,
                'description': 'Write effective prompts for plant analysis'
            },
            {
                'name': 'Exercise 5: Image Processing',
                'file': 'exercise_5_image_processing.py',
                'points': 7,
                'time_minutes': 8,
                'description': 'Basic OpenCV operations for agriculture'
            },
            {
                'name': 'Exercise 6: ROS2 Integration',
                'file': 'exercise_6_ros2_integration.py',
                'points': 5,
                'time_minutes': 7,
                'description': 'Create simple vision node structure'
            }
        ]
        
        self.total_points = sum(ex['points'] for ex in self.exercises)
        self.results = []
        self.start_time = None
        
    def show_introduction(self):
        """Show introduction and instructions"""
        print("💻 Day 2 Coding Exercises")
        print("=" * 40)
        print("Test your practical AI/CV implementation skills!")
        print()
        print(f"📊 Total: {len(self.exercises)} exercises, {self.total_points} points")
        print(f"⏰ Time limit: {self.time_limit_minutes} minutes")
        print("🎯 Passing score: 70% (28+ points)")
        print()
        
        if self.practice_mode:
            print("🎮 PRACTICE MODE - No time limit, get help")
        else:
            print("🏆 ASSESSMENT MODE - Timed, independent work")
        
        print("\n📋 Exercise Overview:")
        for i, ex in enumerate(self.exercises, 1):
            print(f"   {i}. {ex['name']} ({ex['points']} pts, {ex['time_minutes']} min)")
            print(f"      {ex['description']}")
        
        print("\n💡 Tips:")
        print("   • Read all exercises first (5 min)")
        print("   • Start with familiar topics")
        print("   • Test your code before submitting")
        print("   • Focus on agricultural applications")
        print("   • Don't get stuck - move on if needed")
        
    def check_time_limit(self):
        """Check if time limit has been exceeded"""
        if self.start_time is None or self.practice_mode:
            return False
        
        elapsed = datetime.now() - self.start_time
        return elapsed.total_seconds() > (self.time_limit_minutes * 60)
    
    def get_remaining_time(self):
        """Get remaining time in minutes and seconds"""
        if self.start_time is None or self.practice_mode:
            return 999, 0
        
        elapsed = datetime.now() - self.start_time
        remaining_seconds = (self.time_limit_minutes * 60) - elapsed.total_seconds()
        
        if remaining_seconds <= 0:
            return 0, 0
        
        minutes = int(remaining_seconds // 60)
        seconds = int(remaining_seconds % 60)
        return minutes, seconds
    
    def run_exercise(self, exercise):
        """Run a single exercise"""
        print(f"\n🎯 {exercise['name']}")
        print("=" * 50)
        
        # Check time limit
        if self.check_time_limit():
            print("⏰ Time limit exceeded! Skipping remaining exercises.")
            return None
        
        # Show remaining time
        mins, secs = self.get_remaining_time()
        if not self.practice_mode:
            print(f"⏰ Time remaining: {mins:02d}:{secs:02d}")
        
        print(f"📝 Task: {exercise['description']}")
        print(f"📊 Points: {exercise['points']}")
        print(f"⏱️ Suggested time: {exercise['time_minutes']} minutes")
        
        # Check if exercise file exists
        if not os.path.exists(exercise['file']):
            print(f"❌ Exercise file not found: {exercise['file']}")
            return {'completed': False, 'error': 'File not found'}
        
        # Ask if user wants to run this exercise
        if not self.practice_mode:
            choice = input(f"\nRun {exercise['name']}? (y/n/skip): ").strip().lower()
            if choice == 'n':
                return {'completed': False, 'skipped': True}
            elif choice == 'skip':
                return {'completed': False, 'skipped': True}
        
        # Run the exercise
        try:
            exercise_start = time.time()
            
            # Import and run the exercise
            spec = importlib.util.spec_from_file_location("exercise", exercise['file'])
            exercise_module = importlib.util.module_from_spec(spec)
            spec.loader.exec_module(exercise_module)
            
            # Run the main function
            if hasattr(exercise_module, 'main'):
                exercise_module.main()
            
            exercise_time = time.time() - exercise_start
            
            # Simple completion check
            completed = True
            
            return {
                'completed': completed,
                'time_taken': exercise_time,
                'error': None
            }
            
        except Exception as e:
            error_msg = f"Error running exercise: {str(e)}"
            print(f"❌ {error_msg}")
            
            if self.practice_mode:
                print("\n🔍 Error details:")
                traceback.print_exc()
            
            return {
                'completed': False,
                'error': error_msg,
                'time_taken': 0
            }
    
    def run_all_exercises(self):
        """Run all exercises"""
        print("\n🚀 Starting Coding Exercises")
        print("=" * 30)
        
        if not self.practice_mode:
            input("Press Enter to start your timer...")
        
        self.start_time = datetime.now()
        
        # Run each exercise
        for i, exercise in enumerate(self.exercises, 1):
            print(f"\n📍 Exercise {i}/{len(self.exercises)}")
            
            if self.check_time_limit():
                print("⏰ Time limit exceeded!")
                break
            
            result = self.run_exercise(exercise)
            
            if result is None:
                break
            
            self.results.append({
                'exercise': exercise,
                'result': result
            })
            
            # Show progress
            completed_count = sum(1 for r in self.results if r['result']['completed'])
            print(f"✅ Progress: {completed_count}/{len(self.exercises)} exercises completed")
        
        # Show final results
        self.show_final_results()
    
    def show_final_results(self):
        """Show final results and grading"""
        print("\n🎉 Coding Exercises Complete!")
        print("=" * 35)
        
        # Calculate scores
        total_possible = self.total_points
        points_earned = 0
        completed_exercises = 0
        
        # Time analysis
        if self.start_time:
            total_time = datetime.now() - self.start_time
            total_seconds = total_time.total_seconds()
        else:
            total_seconds = 0
        
        print("📊 Exercise Results:")
        print("-" * 20)
        
        for i, result_data in enumerate(self.results, 1):
            exercise = result_data['exercise']
            result = result_data['result']
            
            if result['completed']:
                status = "✅ COMPLETED"
                points = exercise['points']
                completed_exercises += 1
            elif result.get('skipped'):
                status = "⏭️ SKIPPED"
                points = 0
            else:
                status = "❌ FAILED"
                points = 0
            
            points_earned += points
            
            time_str = f"{result.get('time_taken', 0):.1f}s" if result.get('time_taken') else "N/A"
            print(f"   {i}. {exercise['name']}: {status} ({points}/{exercise['points']} pts, {time_str})")
        
        # Overall performance
        print(f"\n📈 Overall Performance:")
        print(f"   Completed: {completed_exercises}/{len(self.exercises)} exercises")
        print(f"   Points: {points_earned}/{total_possible}")
        
        percentage = (points_earned / total_possible) * 100 if total_possible > 0 else 0
        print(f"   Score: {percentage:.1f}%")
        
        if not self.practice_mode:
            print(f"   Time used: {str(total_time).split('.')[0]}")
        
        # Grade assignment
        if percentage >= 90:
            grade = "A (Excellent)"
            message = "Outstanding work! You have strong practical AI/CV skills for agriculture."
        elif percentage >= 80:
            grade = "B (Good)"
            message = "Good performance! You understand the key concepts well."
        elif percentage >= 70:
            grade = "C (Satisfactory)"
            message = "Satisfactory work. You meet the basic requirements."
        elif percentage >= 60:
            grade = "D (Needs Improvement)"
            message = "Some understanding shown, but more practice needed."
        else:
            grade = "F (Insufficient)"
            message = "Significant gaps in understanding. Review materials and retry."
        
        print(f"   Grade: {grade}")
        print(f"   {message}")
        
        # Pass/Fail
        if percentage >= 70:
            print("\n🎊 PASSED! You're ready for the integration challenge.")
        else:
            print("\n📚 Not passed. Review the topics and try again.")
            self.show_improvement_suggestions()
        
        # Show areas for improvement
        self.show_detailed_feedback()
    
    def show_improvement_suggestions(self):
        """Show suggestions for improvement"""
        print("\n💡 Improvement Suggestions:")
        print("-" * 25)
        
        failed_exercises = [r for r in self.results if not r['result']['completed']]
        
        if failed_exercises:
            print("Focus on these areas:")
            for result_data in failed_exercises:
                exercise = result_data['exercise']
                error = result_data['result'].get('error', 'Unknown error')
                print(f"   • {exercise['name']}: {error}")
        
        print("\n📚 Study recommendations:")
        print("   • Review module documentation")
        print("   • Practice with QUICKSTART guides")
        print("   • Run exercises in practice mode")
        print("   • Focus on agricultural applications")
        print("   • Ask for help if needed")
    
    def show_detailed_feedback(self):
        """Show detailed feedback on performance"""
        print("\n📝 Detailed Feedback:")
        print("-" * 20)
        
        # Analyze performance by topic
        topics = {
            'YOLO Detection': [0],
            'Model Evaluation': [1],
            'Data Preparation': [2],
            'Prompt Engineering': [3],
            'Image Processing': [4],
            'ROS2 Integration': [5]
        }
        
        for topic, indices in topics.items():
            topic_results = [self.results[i] for i in indices if i < len(self.results)]
            
            if topic_results:
                completed = sum(1 for r in topic_results if r['result']['completed'])
                total = len(topic_results)
                
                if completed == total:
                    status = "✅ Mastered"
                elif completed > 0:
                    status = "⚠️ Partial"
                else:
                    status = "❌ Needs work"
                
                print(f"   {topic}: {status} ({completed}/{total})")
        
        print("\n🚀 Next Steps:")
        print("   • Review any failed exercises")
        print("   • Practice weak areas")
        print("   • Move to integration challenge")
        print("   • Prepare for Day 3")

def show_help():
    """Show help information"""
    print("📖 Coding Exercises Help")
    print("=" * 25)
    print("Usage:")
    print("  python3 run_exercises.py           # Timed assessment")
    print("  python3 run_exercises.py --practice # Practice mode")
    print("  python3 run_exercises.py --help     # This help")
    print()
    print("Assessment Mode:")
    print("  • 45-minute time limit")
    print("  • Independent work")
    print("  • Graded performance")
    print()
    print("Practice Mode:")
    print("  • No time limit")
    print("  • Get help and hints")
    print("  • Detailed error messages")
    print()
    print("Exercises:")
    print("  1. YOLO Detection (5 pts)")
    print("  2. Model Evaluation (8 pts)")
    print("  3. Data Preparation (7 pts)")
    print("  4. Prompt Engineering (8 pts)")
    print("  5. Image Processing (7 pts)")
    print("  6. ROS2 Integration (5 pts)")

def main():
    """Main function"""
    if '--help' in sys.argv or '-h' in sys.argv:
        show_help()
        return
    
    practice_mode = '--practice' in sys.argv
    
    runner = CodingExercisesRunner(practice_mode)
    
    try:
        runner.show_introduction()
        
        if not practice_mode:
            print("\n⚠️  ASSESSMENT MODE:")
            print("   • You have 45 minutes total")
            print("   • Work independently")
            print("   • No external help allowed")
            print("   • Your work will be graded")
            confirm = input("\nReady to start? (y/n): ").strip().lower()
            if confirm != 'y':
                print("Assessment cancelled.")
                return
        
        runner.run_all_exercises()
        
    except KeyboardInterrupt:
        print(f"\n\n⏹️ Exercises interrupted by user")
        if runner.results:
            print(f"Progress: {len(runner.results)}/{len(runner.exercises)} exercises attempted")
    except Exception as e:
        print(f"\n❌ Error occurred: {e}")
        print("Please restart the exercises or contact support")

if __name__ == "__main__":
    main() 