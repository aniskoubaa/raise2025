# QUICKSTART: Gemini Vision API

## ⚡ 5-Minute Plant Doctor Demo

### Step 1: Setup
```bash
# Navigate to the module directory
cd RAISE2025/Day2_AI_CV/03_gemini_vision_api/

# Install required packages (optional for demo)
pip install google-generativeai

# Run the demo (works without API key)
python3 plant_doctor_demo.py
```

### Step 2: Try the Demo
```bash
# Interactive menu will appear:
# 1. 🎬 Run complete demo (recommended)
# 2. 🧠 Learn about Gemini Vision
# 3. 📝 See prompt examples
# 4. 🩺 Try plant diagnosis
# 5. 💰 Calculate API costs
# 6. 🆚 Compare AI approaches
# 7. 🚪 Exit

# Start with option 1 for full experience
```

### Step 3: Optional - Use Real API
```bash
# Get free API key from: https://makersuite.google.com/app/apikey
# Set your API key:
export GEMINI_API_KEY="your_api_key_here"

# Run with real API
python3 plant_doctor_demo.py
```

## 🤖 What You'll Learn

### 🧠 Core Concepts
- **Large Language Models (LLMs)**: AI that understands both text and images
- **Prompt Engineering**: How to ask AI the right questions
- **Multi-modal Analysis**: Combining vision and language
- **Agricultural AI**: Applying AI to farming problems

### 🌱 Plant Analysis Demo
Experience diagnosing these conditions:
1. **Healthy tomato plant** - Learn normal characteristics
2. **Early blight disease** - See disease symptoms
3. **Late blight disease** - Understand urgency levels
4. **Nutrient deficiency** - Identify nutritional problems
5. **Corn smut** - Different crop, different disease
6. **Wheat rust** - Grain crop diseases

## 📝 Prompt Engineering Examples

### ✅ Good Prompts
```
"Analyze this tomato leaf image. What disease is present and what treatment do you recommend?"
→ Specific, clear, asks for actionable advice

"I'm a beginner farmer. This is my corn field. What problems do you see and how can I fix them?"
→ Provides context, indicates experience level

"Rate this plant's health from 1-10 and explain your reasoning."
→ Asks for structured output and explanation
```

### ❌ Bad Prompts
```
"What's this?"
→ Too vague, no context

"Analyze everything wrong with this plant..."
→ Too complex, asking for too much

"Good or bad?"
→ Binary question, no useful information
```

## 🆚 AI Approach Comparison

| Feature | YOLO (Traditional) | Gemini Vision | Custom CNN |
|---------|-------------------|---------------|------------|
| **Setup** | Complex training | Simple API | Very complex |
| **Speed** | Very Fast (10ms) | Moderate (2-5s) | Fast (50ms) |
| **Flexibility** | Limited objects | Analyzes anything | Limited domain |
| **Explanations** | None | Detailed | None |
| **Cost** | One-time training | Per request | High development |

## 💰 Cost Information

### Free Tier
- **15 requests per minute** (perfect for learning)
- **No credit card required**
- **Great for small farms**

### Paid Tier
- **$0.0025 per image** (very affordable)
- **Small farm**: ~$7.50/month (10 images/day)
- **Medium farm**: ~$37.50/month (50 images/day)
- **Large operation**: ~$150/month (200 images/day)

## 🎯 Demo Scenarios

### 1. Healthy Plant Analysis
```
Plant: Tomato
Condition: Healthy
Confidence: 95%
Recommendations: Continue current care, monitor regularly
```

### 2. Disease Diagnosis
```
Plant: Tomato
Condition: Early Blight
Confidence: 88%
Urgency: 🟡 MEDIUM
Treatment: Copper fungicide, remove affected leaves
```

### 3. Critical Condition
```
Plant: Tomato
Condition: Late Blight
Confidence: 92%
Urgency: 🔴 HIGH - Immediate action required!
Treatment: Systemic fungicide, destroy affected plants
```

## 🔧 Interactive Features

### 🩺 Plant Diagnosis
Choose from 7 different plant scenarios:
- Experience different diagnostic challenges
- See confidence levels and urgency ratings
- Get detailed treatment recommendations
- Learn about disease progression

### 💬 Follow-up Questions
Ask about:
1. **What caused this problem?**
2. **How can I prevent this in the future?**
3. **How long will treatment take?**
4. **Is this problem contagious?**

### 💰 Cost Calculator
Calculate API costs for:
- Small farm operations
- Medium farm operations
- Large commercial farms
- Mobile app development

## 🚨 Important Notes

### 🔒 API Key Security
- ✅ Keep your API key secret
- ✅ Use environment variables
- ✅ Don't commit to version control
- ✅ Monitor usage and costs

### 🧠 AI Limitations
- ❌ Don't rely solely on AI
- ❌ Always verify with experts
- ❌ Question low confidence results
- ❌ Test in real field conditions

## 🎬 Demo Flow

1. **Start** → Choose option 1 (complete demo)
2. **Learn** → Understand how Gemini Vision works
3. **Examples** → See good vs bad prompts
4. **Diagnose** → Try plant analysis scenarios
5. **Costs** → Understand pricing structure
6. **Compare** → See different AI approaches
7. **Complete** → Ready for Module 4!

## 📚 Real-World Applications

### 🚜 Farm Management
- **Field monitoring**: Regular crop health checks
- **Disease early warning**: Detect problems before they spread
- **Treatment optimization**: Get specific recommendations
- **Yield prediction**: Assess crop potential

### 📱 Mobile Integration
- **Farmer apps**: Instant diagnosis in the field
- **Extension services**: Remote expert assistance
- **Quality control**: Post-harvest assessment
- **Training tools**: Educational applications

### 🏭 Commercial Uses
- **Greenhouse monitoring**: Automated plant health
- **Supply chain**: Quality assessment
- **Research**: Agricultural studies
- **Insurance**: Crop damage assessment

## 🎯 Learning Outcomes

After this demo, you'll understand:
- ✅ How LLMs work for agriculture
- ✅ Writing effective prompts
- ✅ Plant diagnosis with AI
- ✅ Cost considerations
- ✅ When to use different AI approaches

## 🚀 Next Steps

After completing this module:
1. **Try with your own images** (if you have API key)
2. **Experiment with different prompts**
3. **Think about applications for your farm**
4. **Move to Module 4**: ROS2 + Computer Vision Integration

## 🔄 Quick Commands

```bash
# Run demo
python3 plant_doctor_demo.py

# Check if API key is set
echo $GEMINI_API_KEY

# Quick plant diagnosis
python3 -c "
from plant_doctor_demo import PlantDoctorDemo
demo = PlantDoctorDemo()
demo.interactive_diagnosis()
"

# See prompt examples
python3 -c "
from plant_doctor_demo import PlantDoctorDemo
demo = PlantDoctorDemo()
demo.show_prompt_examples()
"
```

Happy analyzing! 🌱🤖 