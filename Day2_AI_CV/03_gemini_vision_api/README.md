# Module 3: Gemini Vision API for Plant Analysis

## 🎯 Learning Objectives

By the end of this module, you will understand:
- ✅ What Large Language Models (LLMs) are and how they work
- ✅ How to use Gemini Vision API for plant analysis
- ✅ How to write effective prompts for agricultural AI
- ✅ How to combine text and image analysis
- ✅ Practical applications in agricultural diagnostics

**Duration:** 45 minutes  
**Difficulty:** Beginner  
**Prerequisites:** Modules 1 & 2 (YOLO and Training basics)

## 🤖 What is Gemini Vision API?

Gemini Vision is Google's AI that can:
- **See and understand images** like a human expert
- **Answer questions** about what it sees
- **Provide detailed explanations** in natural language
- **Give recommendations** based on visual analysis

### 🌱 **Traditional AI vs Gemini Vision**

| Traditional Computer Vision | Gemini Vision |
|----------------------------|---------------|
| Can only detect pre-trained objects | Can analyze any image |
| Gives simple labels | Provides detailed descriptions |
| Limited to specific tasks | Can answer any question |
| Requires training data | Uses general knowledge |
| Says "tomato leaf" | Says "This tomato leaf shows early blight disease with characteristic dark spots. Recommend fungicide treatment." |

## 🧠 How Large Language Models Work

### 🔄 **The Magic Behind LLMs**

Think of LLMs like having a super-smart agricultural expert who:
1. **Has read every agricultural textbook** 📚
2. **Seen millions of plant images** 🌱
3. **Knows about diseases, treatments, and best practices** 🔬
4. **Can explain complex concepts simply** 💡

### 🎯 **Vision + Language = Powerful Analysis**

```
Step 1: You show an image
"Here's a photo of my tomato plant"

Step 2: You ask a question
"What's wrong with this plant?"

Step 3: Gemini analyzes and responds
"I can see signs of early blight disease. The dark spots with concentric rings are characteristic. I recommend applying copper-based fungicide and improving air circulation around the plants."
```

## 🌿 Agricultural Applications

### 🔍 **Disease Diagnosis**
```python
# Example interaction
prompt = "Analyze this tomato leaf image. What disease is present and what treatment do you recommend?"

response = "I can see early blight disease on this tomato leaf. The symptoms include:
- Dark brown spots with concentric rings
- Yellowing around the spots
- Lower leaves are most affected

Recommended treatment:
- Apply copper-based fungicide
- Remove affected leaves
- Improve air circulation
- Water at soil level to avoid wetting leaves"
```

### 🌾 **Crop Health Assessment**
```python
# Field analysis
prompt = "Assess the overall health of this corn field. What issues do you see?"

response = "This corn field shows signs of nitrogen deficiency:
- Yellowing leaves (chlorosis)
- Stunted growth
- V-shaped yellowing pattern

Recommendations:
- Apply nitrogen fertilizer (150-200 lbs/acre)
- Soil test to confirm nutrient levels
- Consider split application for better uptake"
```

### 🍅 **Ripeness Detection**
```python
# Harvest timing
prompt = "Are these tomatoes ready for harvest? What stage of ripeness are they?"

response = "These tomatoes are at the 'breaker' stage:
- Light green color with pink blush
- Full size but not fully ripe
- Perfect for commercial harvest and shipping
- Will ripen during transport
- Harvest now for best quality and shelf life"
```

## 🎨 Prompt Engineering for Agriculture

### 📝 **Writing Effective Prompts**

The key to getting great results is asking the right questions:

#### ✅ **Good Prompts**
```python
# Specific and detailed
"Analyze this tomato leaf image. Identify any diseases, provide confidence level, and suggest treatment options."

# Context-rich
"I'm a beginner farmer. This is my first tomato crop. What's wrong with this plant and how can I fix it simply?"

# Action-oriented
"What immediate steps should I take to treat this plant disease? List in order of priority."
```

#### ❌ **Poor Prompts**
```python
# Too vague
"What's this?"

# Too complex
"Analyze this image for diseases, pests, nutrient deficiencies, soil problems, watering issues, and provide a complete treatment plan with chemical formulations."

# Without context
"Is this good or bad?"
```

### 🏗️ **Prompt Template for Plant Analysis**

```python
# Standard template
prompt = f"""
Analyze this {crop_type} image as an agricultural expert.

Please provide:
1. Overall plant health assessment
2. Any diseases or problems identified
3. Confidence level (1-10)
4. Immediate treatment recommendations
5. Prevention tips for the future

Keep the language simple for beginner farmers.
"""
```

## 🔧 Hands-On Exercise: Plant Doctor AI

### 🚀 **Building Your AI Plant Doctor**

We'll create a simple AI that can:
1. **Analyze plant images**
2. **Diagnose problems**
3. **Provide treatment recommendations**
4. **Give beginner-friendly advice**

### 📱 **Interactive Demo**

```bash
# Navigate to module directory
cd RAISE2025/Day2_AI_CV/03_gemini_vision_api/

# Run the plant doctor demo
python3 plant_doctor_demo.py
```

## 🔑 API Key Setup

### 🛠️ **Getting Your Gemini API Key**

1. **Go to Google AI Studio**: https://makersuite.google.com/app/apikey
2. **Sign in** with your Google account
3. **Create new API key**
4. **Copy the key** (keep it secret!)

### 🔒 **Setting Up Your Key**

```bash
# Option 1: Environment variable (recommended)
export GEMINI_API_KEY="your_api_key_here"

# Option 2: In the code (for demo only)
api_key = "your_api_key_here"
```

### 🚨 **API Key Security**
- ✅ **Never share your API key**
- ✅ **Don't commit it to version control**
- ✅ **Use environment variables**
- ✅ **Rotate keys regularly**

## 💰 Cost Management

### 💳 **Understanding API Costs**

Gemini Vision API pricing (approximate):
- **Free tier**: 15 requests per minute
- **Paid tier**: $0.00025 per 1K characters
- **Image analysis**: ~$0.0025 per image

### 📊 **Cost Optimization Tips**

```python
# Efficient prompts
prompt = "Diagnose this plant disease" # Short and focused

# Batch analysis
images = [img1, img2, img3]
results = analyze_batch(images)  # More efficient

# Caching results
if image_hash in cache:
    return cache[image_hash]  # Avoid duplicate API calls
```

## 🌟 Advanced Features

### 🔍 **Multi-Modal Analysis**

```python
# Combine image and text input
prompt = f"""
Image: {plant_image}
Context: This is a tomato plant in greenhouse, day 45 after planting.
Weather: Hot and humid, 85°F, 90% humidity
Previous treatments: None

What's the diagnosis and treatment plan?
"""
```

### 📊 **Structured Responses**

```python
# Request structured output
prompt = """
Analyze this plant image and respond in JSON format:
{
    "plant_type": "...",
    "health_status": "healthy/diseased/stressed",
    "confidence": 0-100,
    "issues": ["list", "of", "problems"],
    "treatments": ["list", "of", "solutions"],
    "urgency": "low/medium/high"
}
"""
```

### 🔄 **Conversation Flow**

```python
# Follow-up questions
conversation = [
    {"role": "user", "content": "What's wrong with this plant?"},
    {"role": "assistant", "content": "I see early blight disease..."},
    {"role": "user", "content": "What fungicide should I use?"},
    {"role": "assistant", "content": "For early blight, I recommend..."}
]
```

## 🌐 Real-World Applications

### 🚜 **Farm Management System**

```python
# Integrated farm analysis
def analyze_field_section(image, location, crop_type):
    prompt = f"""
    Analyze this {crop_type} field image from {location}.
    
    Provide:
    1. Overall field health score (1-10)
    2. Problem areas requiring attention
    3. Recommended actions with priority
    4. Estimated yield impact
    """
    
    return gemini_analyze(image, prompt)
```

### 📱 **Mobile App Integration**

```python
# Farmer's smartphone app
def mobile_plant_diagnosis(photo):
    prompt = """
    I'm a farmer who needs help. Analyze this plant photo and:
    1. Tell me what's wrong (if anything)
    2. Explain in simple terms
    3. Give me step-by-step treatment
    4. Tell me how urgent this is
    """
    
    return gemini_analyze(photo, prompt)
```

### 🏭 **Quality Control**

```python
# Post-harvest quality assessment
def quality_check(produce_image):
    prompt = """
    Assess this produce for quality control:
    1. Grade (A, B, C, or reject)
    2. Defects identified
    3. Shelf life estimate
    4. Storage recommendations
    """
    
    return gemini_analyze(produce_image, prompt)
```

## 🔬 Comparison with Traditional Methods

### 🆚 **Gemini vs Traditional Computer Vision**

| Aspect | Traditional CV | Gemini Vision |
|--------|---------------|---------------|
| **Training** | Requires thousands of labeled images | No training needed |
| **Flexibility** | Fixed set of detectable objects | Can analyze anything |
| **Explanations** | Just labels and confidence scores | Detailed explanations |
| **Context** | No understanding of context | Understands agricultural context |
| **Updates** | Requires retraining | Automatically improves |
| **Cost** | High initial setup cost | Pay per use |

### 🎯 **When to Use Each**

| Use Case | Traditional CV | Gemini Vision |
|----------|---------------|---------------|
| **High-speed processing** | ✅ Better (milliseconds) | ❌ Slower (seconds) |
| **Batch processing** | ✅ Efficient | ❌ API limits |
| **Detailed analysis** | ❌ Limited | ✅ Excellent |
| **Novel problems** | ❌ Can't handle | ✅ Adaptable |
| **Expert explanations** | ❌ Not available | ✅ Detailed |

## 💡 Best Practices

### ✅ **Do's**
1. **Be specific** in your prompts
2. **Provide context** about the farm/crop
3. **Ask for explanations** to understand the reasoning
4. **Use structured prompts** for consistent results
5. **Validate results** with agricultural experts

### ❌ **Don'ts**
1. **Don't rely solely on AI** - combine with human expertise
2. **Don't ignore confidence levels** - question low-confidence results
3. **Don't use for life-critical decisions** without verification
4. **Don't share sensitive farm data** without proper security
5. **Don't forget to monitor API usage** and costs

## 🚀 Getting Started

### 📦 **Installation**
```bash
pip install google-generativeai pillow requests
```

### 🔑 **API Key Setup**
```bash
export GEMINI_API_KEY="your_api_key_here"
```

### 🎮 **Run the Demo**
```bash
python3 plant_doctor_demo.py
```

## 🎯 Learning Outcomes

After completing this module, you will:
- ✅ Understand how LLMs work for agricultural analysis
- ✅ Be able to write effective prompts for plant diagnosis
- ✅ Know how to integrate Gemini Vision into agricultural workflows
- ✅ Understand the benefits and limitations of AI-powered analysis
- ✅ Be ready to build your own agricultural AI applications

## 📚 What's Next?

Next module: **ROS2 + Computer Vision Integration** - combining all the AI techniques with robotic systems for complete agricultural automation! 