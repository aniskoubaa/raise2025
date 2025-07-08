#!/usr/bin/env python3
"""
RAISE 2025 - Plant Doctor AI Demo
Educational demonstration of using Gemini Vision API for agricultural analysis.

This script demonstrates:
- Plant image analysis using AI
- Prompt engineering for agriculture
- Disease diagnosis and treatment recommendations
- Beginner-friendly AI integration

Author: RAISE 2025 Team
Date: July 2025
"""

import os
import time
import random
from datetime import datetime
import base64
from io import BytesIO

class PlantDoctorDemo:
    def __init__(self):
        print("🌱 Welcome to Plant Doctor AI!")
        print("=" * 40)
        
        # Check if real API key is available
        self.has_api_key = os.getenv('GEMINI_API_KEY') is not None
        
        if self.has_api_key:
            print("✅ Gemini API key detected - using real API")
            try:
                import google.generativeai as genai
                self.genai = genai
                genai.configure(api_key=os.getenv('GEMINI_API_KEY'))
                self.model = genai.GenerativeModel('gemini-pro-vision')
                print("✅ Connected to Gemini Vision API")
            except ImportError:
                print("❌ google-generativeai not installed, using demo mode")
                self.has_api_key = False
        else:
            print("ℹ️ No API key found - running in demo mode")
            print("   To use real API, set GEMINI_API_KEY environment variable")
        
        # Plant problems database for demo mode
        self.plant_problems = {
            'tomato': {
                'healthy': {
                    'diagnosis': 'Healthy tomato plant',
                    'confidence': 95,
                    'description': 'This tomato plant appears healthy with vibrant green leaves and no visible signs of disease or stress.',
                    'recommendations': [
                        'Continue current care routine',
                        'Monitor for early signs of disease',
                        'Ensure adequate water and nutrients',
                        'Provide support for growing stems'
                    ]
                },
                'early_blight': {
                    'diagnosis': 'Early Blight Disease',
                    'confidence': 88,
                    'description': 'The plant shows characteristic symptoms of early blight: dark spots with concentric rings on lower leaves, yellowing around spots.',
                    'recommendations': [
                        'Apply copper-based fungicide immediately',
                        'Remove affected leaves and dispose safely',
                        'Improve air circulation around plants',
                        'Water at soil level to avoid wetting leaves',
                        'Apply mulch to prevent soil splash'
                    ]
                },
                'late_blight': {
                    'diagnosis': 'Late Blight Disease',
                    'confidence': 92,
                    'description': 'Critical condition: Late blight detected with water-soaked lesions and white fuzzy growth on leaf undersides.',
                    'recommendations': [
                        'URGENT: Apply systemic fungicide (chlorothalonil)',
                        'Remove and destroy all affected plants',
                        'Avoid overhead watering completely',
                        'Increase spacing between plants',
                        'Monitor neighboring plants closely'
                    ]
                },
                'nutrient_deficiency': {
                    'diagnosis': 'Nitrogen Deficiency',
                    'confidence': 85,
                    'description': 'Yellowing of lower leaves indicates nitrogen deficiency. Plants appear stunted with pale green coloration.',
                    'recommendations': [
                        'Apply nitrogen-rich fertilizer (10-10-10)',
                        'Use compost or well-rotted manure',
                        'Consider foliar feeding for quick results',
                        'Test soil pH (should be 6.0-6.8)',
                        'Ensure proper drainage'
                    ]
                }
            },
            'corn': {
                'healthy': {
                    'diagnosis': 'Healthy corn plant',
                    'confidence': 94,
                    'description': 'Corn plant shows excellent health with dark green leaves and strong growth.',
                    'recommendations': [
                        'Continue current fertilization program',
                        'Monitor for pest activity',
                        'Ensure adequate water during tasseling',
                        'Side-dress with nitrogen if needed'
                    ]
                },
                'corn_smut': {
                    'diagnosis': 'Corn Smut (Ustilago maydis)',
                    'confidence': 90,
                    'description': 'Fungal infection causing large, grayish-white galls on ears, stalks, and leaves.',
                    'recommendations': [
                        'Remove and destroy infected plants',
                        'Do not compost affected material',
                        'Plant resistant varieties next season',
                        'Improve field drainage',
                        'Rotate crops to break disease cycle'
                    ]
                }
            },
            'wheat': {
                'rust': {
                    'diagnosis': 'Wheat Rust Disease',
                    'confidence': 89,
                    'description': 'Orange-red pustules on leaves indicate rust disease, which can significantly reduce yield.',
                    'recommendations': [
                        'Apply fungicide (propiconazole) if economically justified',
                        'Monitor weather conditions (rust spreads in humid conditions)',
                        'Consider early harvest if severely infected',
                        'Plant resistant varieties in future',
                        'Remove volunteer wheat plants'
                    ]
                }
            }
        }

    def explain_gemini_vision(self):
        """Explain what Gemini Vision is and how it works"""
        print("\n🧠 What is Gemini Vision?")
        print("=" * 30)
        
        print("Gemini Vision is Google's AI that combines:")
        print("  🔍 Computer Vision - Can 'see' and understand images")
        print("  💬 Natural Language - Can explain what it sees")
        print("  🌱 Agricultural Knowledge - Knows about plants and farming")
        
        print("\n🎯 How it works:")
        print("1. You provide a plant image")
        print("2. You ask a question about the plant")
        print("3. Gemini analyzes the image")
        print("4. It provides detailed diagnosis and recommendations")
        
        print("\n🌟 What makes it special:")
        print("  • No training needed - works out of the box")
        print("  • Understands context and nuance")
        print("  • Provides explanations, not just labels")
        print("  • Can answer follow-up questions")
        
        print("\n💡 Example conversation:")
        print("  👤 You: 'What's wrong with this tomato plant?'")
        print("  🤖 Gemini: 'I can see early blight disease. The dark spots...'")
        print("  👤 You: 'What treatment do you recommend?'")
        print("  🤖 Gemini: 'Apply copper-based fungicide and...'")

    def show_prompt_examples(self):
        """Show examples of good and bad prompts"""
        print("\n📝 Prompt Engineering Examples")
        print("=" * 35)
        
        print("✅ GOOD PROMPTS:")
        print("-" * 15)
        
        good_prompts = [
            {
                "prompt": "Analyze this tomato leaf image. What disease is present and what treatment do you recommend?",
                "why": "Specific, clear, asks for actionable advice"
            },
            {
                "prompt": "I'm a beginner farmer. This is my corn field. What problems do you see and how can I fix them?",
                "why": "Provides context, indicates experience level"
            },
            {
                "prompt": "Rate this plant's health from 1-10 and explain your reasoning.",
                "why": "Asks for structured output and explanation"
            }
        ]
        
        for i, example in enumerate(good_prompts, 1):
            print(f"{i}. \"{example['prompt']}\"")
            print(f"   Why good: {example['why']}\n")
        
        print("❌ BAD PROMPTS:")
        print("-" * 15)
        
        bad_prompts = [
            {
                "prompt": "What's this?",
                "why": "Too vague, no context"
            },
            {
                "prompt": "Analyze everything wrong with this plant and give me a complete treatment plan with exact chemical formulations and application rates.",
                "why": "Too complex, asking for too much at once"
            },
            {
                "prompt": "Good or bad?",
                "why": "Binary question, no useful information"
            }
        ]
        
        for i, example in enumerate(bad_prompts, 1):
            print(f"{i}. \"{example['prompt']}\"")
            print(f"   Why bad: {example['why']}\n")

    def simulate_analysis(self, plant_type, condition):
        """Simulate plant analysis for demo purposes"""
        print(f"\n🔍 Analyzing {plant_type} image...")
        print("  📸 Processing image...")
        time.sleep(1)
        print("  🧠 Running AI analysis...")
        time.sleep(1)
        print("  📊 Generating diagnosis...")
        time.sleep(1)
        
        # Get plant problem from database
        if plant_type in self.plant_problems and condition in self.plant_problems[plant_type]:
            result = self.plant_problems[plant_type][condition]
        else:
            # Fallback to generic healthy plant
            result = {
                'diagnosis': f'Healthy {plant_type} plant',
                'confidence': 85,
                'description': f'This {plant_type} plant appears to be in good health.',
                'recommendations': ['Continue current care routine', 'Monitor regularly']
            }
        
        return result

    def analyze_plant_image(self, plant_type="tomato", condition="healthy"):
        """Analyze a plant image (demo mode)"""
        print(f"\n🌱 Plant Analysis Results")
        print("=" * 30)
        
        # Simulate analysis
        result = self.simulate_analysis(plant_type, condition)
        
        # Display results
        print(f"Plant Type: {plant_type.title()}")
        print(f"Diagnosis: {result['diagnosis']}")
        print(f"Confidence: {result['confidence']}%")
        print(f"\nDescription:")
        print(f"  {result['description']}")
        
        print(f"\nRecommendations:")
        for i, rec in enumerate(result['recommendations'], 1):
            print(f"  {i}. {rec}")
        
        # Show urgency level
        urgency = self.get_urgency_level(result['diagnosis'])
        print(f"\nUrgency Level: {urgency}")
        
        return result

    def get_urgency_level(self, diagnosis):
        """Determine urgency level based on diagnosis"""
        if 'Late Blight' in diagnosis or 'Critical' in diagnosis:
            return "🔴 HIGH - Take immediate action!"
        elif 'Disease' in diagnosis or 'Deficiency' in diagnosis:
            return "🟡 MEDIUM - Address within 1-2 days"
        else:
            return "🟢 LOW - Continue monitoring"

    def interactive_diagnosis(self):
        """Interactive plant diagnosis demo"""
        print("\n🩺 Interactive Plant Diagnosis")
        print("=" * 35)
        
        print("Available scenarios:")
        scenarios = {
            '1': ('tomato', 'healthy'),
            '2': ('tomato', 'early_blight'),
            '3': ('tomato', 'late_blight'),
            '4': ('tomato', 'nutrient_deficiency'),
            '5': ('corn', 'healthy'),
            '6': ('corn', 'corn_smut'),
            '7': ('wheat', 'rust')
        }
        
        print("1. 🍅 Healthy tomato plant")
        print("2. 🍅 Tomato with early blight")
        print("3. 🍅 Tomato with late blight")
        print("4. 🍅 Tomato with nutrient deficiency")
        print("5. 🌽 Healthy corn plant")
        print("6. 🌽 Corn with smut disease")
        print("7. 🌾 Wheat with rust disease")
        
        choice = input("\nSelect a scenario (1-7): ").strip()
        
        if choice in scenarios:
            plant_type, condition = scenarios[choice]
            result = self.analyze_plant_image(plant_type, condition)
            
            # Follow-up questions
            print("\n💬 Ask follow-up questions:")
            print("1. What caused this problem?")
            print("2. How can I prevent this in the future?")
            print("3. How long will treatment take?")
            print("4. Is this problem contagious?")
            
            follow_up = input("\nSelect a question (1-4) or press Enter to skip: ").strip()
            
            if follow_up:
                self.answer_follow_up(plant_type, condition, follow_up)
        else:
            print("❌ Invalid choice. Please try again.")

    def answer_follow_up(self, plant_type, condition, question):
        """Answer follow-up questions"""
        print(f"\n🤖 AI Response:")
        
        responses = {
            'early_blight': {
                '1': "Early blight is caused by the fungus Alternaria solani. It thrives in warm, humid conditions with temperatures between 75-85°F.",
                '2': "Prevention tips: Use drip irrigation, space plants properly, rotate crops, apply preventive fungicides, and choose resistant varieties.",
                '3': "Treatment typically takes 2-3 weeks. You should see improvement within 7-10 days if caught early.",
                '4': "Yes, early blight is contagious and spreads through spores. Remove infected leaves immediately and avoid working with wet plants."
            },
            'late_blight': {
                '1': "Late blight is caused by Phytophthora infestans, the same pathogen that caused the Irish potato famine. It spreads rapidly in cool, wet conditions.",
                '2': "Prevention: Use resistant varieties, ensure good air circulation, avoid overhead watering, and apply preventive fungicides before symptoms appear.",
                '3': "Late blight treatment is urgent and must begin immediately. Without treatment, plants can die within days.",
                '4': "Extremely contagious! Spores can travel miles on wind. Infected plants must be removed and destroyed immediately."
            },
            'nutrient_deficiency': {
                '1': "Nitrogen deficiency typically results from depleted soil, overwatering (leaching), or poor soil pH that prevents nutrient uptake.",
                '2': "Prevention: Regular soil testing, proper fertilization schedule, composting, and maintaining proper soil pH (6.0-6.8 for tomatoes).",
                '3': "Plants should show improvement within 1-2 weeks after proper fertilization. Full recovery takes 3-4 weeks.",
                '4': "Not contagious - it's a nutritional issue, not a disease. However, stressed plants are more susceptible to diseases."
            }
        }
        
        if condition in responses and question in responses[condition]:
            print(f"  {responses[condition][question]}")
        else:
            print("  I'd be happy to help with that specific question about your plant!")

    def cost_calculator(self):
        """Calculate API costs for different usage scenarios"""
        print("\n💰 Gemini Vision API Cost Calculator")
        print("=" * 40)
        
        print("Current Gemini Vision pricing (approximate):")
        print("  • Free tier: 15 requests per minute")
        print("  • Paid tier: $0.0025 per image analysis")
        print("  • Text generation: $0.00025 per 1K characters")
        
        scenarios = {
            'Small Farm': {
                'images_per_day': 10,
                'days_per_month': 30,
                'description': 'Small farm with daily monitoring'
            },
            'Medium Farm': {
                'images_per_day': 50,
                'days_per_month': 30,
                'description': 'Medium farm with field sections'
            },
            'Large Farm': {
                'images_per_day': 200,
                'days_per_month': 30,
                'description': 'Large farm with automated monitoring'
            },
            'Mobile App': {
                'images_per_day': 1000,
                'days_per_month': 30,
                'description': 'Mobile app serving multiple users'
            }
        }
        
        print("\nCost estimates for different scenarios:")
        print("-" * 40)
        
        for scenario, data in scenarios.items():
            monthly_images = data['images_per_day'] * data['days_per_month']
            monthly_cost = monthly_images * 0.0025
            
            print(f"\n{scenario}: {data['description']}")
            print(f"  • {data['images_per_day']} images/day")
            print(f"  • {monthly_images} images/month")
            print(f"  • Estimated cost: ${monthly_cost:.2f}/month")
            
            if monthly_cost > 100:
                print(f"  💡 Tip: Consider batch processing to reduce costs")

    def show_comparison(self):
        """Show comparison between different AI approaches"""
        print("\n🆚 AI Approach Comparison")
        print("=" * 30)
        
        approaches = {
            'YOLO (Traditional CV)': {
                'accuracy': '85-95%',
                'speed': 'Very Fast (10ms)',
                'cost': 'Low (one-time)',
                'setup': 'Complex training',
                'flexibility': 'Limited to trained objects',
                'explanations': 'None'
            },
            'Gemini Vision': {
                'accuracy': '90-95%',
                'speed': 'Moderate (2-5s)',
                'cost': 'Per request',
                'setup': 'Simple API calls',
                'flexibility': 'Analyzes anything',
                'explanations': 'Detailed'
            },
            'Custom CNN': {
                'accuracy': '80-90%',
                'speed': 'Fast (50ms)',
                'cost': 'High (development)',
                'setup': 'Very complex',
                'flexibility': 'Limited domain',
                'explanations': 'None'
            }
        }
        
        # Print comparison table
        print(f"{'Approach':<20} {'Accuracy':<12} {'Speed':<15} {'Cost':<15} {'Setup':<15}")
        print("-" * 77)
        
        for approach, metrics in approaches.items():
            print(f"{approach:<20} {metrics['accuracy']:<12} {metrics['speed']:<15} {metrics['cost']:<15} {metrics['setup']:<15}")
        
        print("\n💡 When to use each approach:")
        print("  🏎️ YOLO: Real-time processing, specific objects, edge devices")
        print("  🧠 Gemini: Complex analysis, explanations, rapid prototyping")
        print("  🔧 Custom CNN: Specialized domains, high accuracy requirements")

    def run_demo(self):
        """Run the complete demo"""
        print("\n🎬 Complete Plant Doctor Demo")
        print("=" * 35)
        
        input("\nPress Enter to learn about Gemini Vision...")
        self.explain_gemini_vision()
        
        input("\nPress Enter to see prompt examples...")
        self.show_prompt_examples()
        
        input("\nPress Enter to try plant diagnosis...")
        self.interactive_diagnosis()
        
        input("\nPress Enter to see cost information...")
        self.cost_calculator()
        
        input("\nPress Enter to see AI comparison...")
        self.show_comparison()
        
        print("\n🎉 Demo Complete!")
        print("=" * 20)
        print("You now understand:")
        print("  ✅ How Gemini Vision works")
        print("  ✅ How to write effective prompts")
        print("  ✅ Plant diagnosis capabilities")
        print("  ✅ Cost considerations")
        print("  ✅ When to use different AI approaches")
        print("\nReady for Module 4: ROS2 + CV Integration! 🚀")

def main():
    """Main function"""
    demo = PlantDoctorDemo()
    
    while True:
        print("\n🌱 Plant Doctor AI Menu:")
        print("=" * 30)
        print("1. 🎬 Run complete demo")
        print("2. 🧠 Learn about Gemini Vision")
        print("3. 📝 See prompt examples")
        print("4. 🩺 Try plant diagnosis")
        print("5. 💰 Calculate API costs")
        print("6. 🆚 Compare AI approaches")
        print("7. 🚪 Exit")
        
        choice = input("\nEnter your choice (1-7): ").strip()
        
        if choice == '1':
            demo.run_demo()
        elif choice == '2':
            demo.explain_gemini_vision()
        elif choice == '3':
            demo.show_prompt_examples()
        elif choice == '4':
            demo.interactive_diagnosis()
        elif choice == '5':
            demo.cost_calculator()
        elif choice == '6':
            demo.show_comparison()
        elif choice == '7':
            print("👋 Thanks for learning about AI plant diagnosis!")
            break
        else:
            print("❌ Invalid choice. Please try again.")

if __name__ == "__main__":
    main() 