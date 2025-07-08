#!/usr/bin/env python3
"""
RAISE 2025 - Tomato Disease Training Demo
Educational simulation of training a tomato disease detection model.

This script demonstrates:
- Data preparation concepts
- Training simulation
- Performance evaluation
- Model improvement techniques

Author: RAISE 2025 Team
Date: July 2025
"""

import random
import time
import matplotlib.pyplot as plt
from collections import defaultdict
import numpy as np

class TomatoTrainingDemo:
    def __init__(self):
        print("🍅 Welcome to Tomato Disease Training Demo!")
        print("=" * 50)
        
        # Define tomato diseases
        self.diseases = {
            'Healthy': {'symptoms': 'Green, vibrant leaves', 'impact': 'Normal growth'},
            'Early Blight': {'symptoms': 'Dark spots with rings', 'impact': 'Reduces yield 20-30%'},
            'Late Blight': {'symptoms': 'Water-soaked spots', 'impact': 'Can destroy crops'},
            'Leaf Mold': {'symptoms': 'Yellow spots turning brown', 'impact': 'Affects fruit quality'},
            'Bacterial Spot': {'symptoms': 'Small dark spots', 'impact': 'Spreads rapidly'}
        }
        
        # Simulate dataset
        self.dataset_size = 1000
        self.train_split = 0.7
        self.val_split = 0.2
        self.test_split = 0.1
        
        # Training parameters
        self.epochs = 10
        self.initial_accuracy = 0.3  # Start with random guessing
        self.target_accuracy = 0.92  # Target accuracy
        
        print(f"📊 Dataset: {self.dataset_size} images")
        print(f"🦠 Diseases: {len(self.diseases)} classes")
        print(f"🎯 Target accuracy: {self.target_accuracy*100:.1f}%")

    def explain_diseases(self):
        """Explain different tomato diseases"""
        print("\n🦠 Tomato Diseases Overview:")
        print("=" * 40)
        
        for disease, info in self.diseases.items():
            print(f"\n{disease}:")
            print(f"  Symptoms: {info['symptoms']}")
            print(f"  Impact: {info['impact']}")
        
        print("\n💡 Why AI Detection is Important:")
        print("  • Early detection saves crops")
        print("  • Consistent diagnosis")
        print("  • Faster than human inspection")
        print("  • Can detect subtle patterns")

    def prepare_dataset(self):
        """Simulate dataset preparation"""
        print("\n📚 Step 1: Preparing Dataset")
        print("=" * 30)
        
        # Calculate splits
        train_size = int(self.dataset_size * self.train_split)
        val_size = int(self.dataset_size * self.val_split)
        test_size = self.dataset_size - train_size - val_size
        
        print(f"Training set: {train_size} images (70%)")
        print(f"Validation set: {val_size} images (20%)")
        print(f"Test set: {test_size} images (10%)")
        
        # Simulate data balance
        images_per_class = train_size // len(self.diseases)
        print(f"\nImages per disease class: {images_per_class}")
        
        # Show data distribution
        print("\n📊 Data Distribution:")
        for disease in self.diseases:
            print(f"  {disease}: {images_per_class} images")
        
        # Simulate data augmentation
        print("\n🎨 Data Augmentation:")
        print("  • Rotation: ±15 degrees")
        print("  • Horizontal flip: 50% chance")
        print("  • Brightness: ±20%")
        print("  • Contrast: ±10%")
        print(f"  • Effective dataset size: {train_size * 4} images")
        
        time.sleep(2)

    def simulate_training(self):
        """Simulate the training process"""
        print("\n🚀 Step 2: Training Model")
        print("=" * 25)
        
        print("Starting with pre-trained ImageNet model...")
        print("Fine-tuning for tomato disease detection...")
        
        # Training metrics
        train_accuracies = []
        val_accuracies = []
        losses = []
        
        for epoch in range(self.epochs):
            print(f"\n--- Epoch {epoch + 1}/{self.epochs} ---")
            
            # Simulate training progress
            progress = (epoch + 1) / self.epochs
            
            # Training accuracy improves with some noise
            train_acc = self.initial_accuracy + (self.target_accuracy - self.initial_accuracy) * progress
            train_acc += random.uniform(-0.05, 0.05)  # Add some noise
            train_acc = max(0, min(1, train_acc))  # Clamp to [0, 1]
            
            # Validation accuracy lags behind slightly
            val_acc = train_acc - random.uniform(0.02, 0.08)
            val_acc = max(0, min(1, val_acc))
            
            # Loss decreases
            loss = 2.0 * (1 - progress) + random.uniform(-0.1, 0.1)
            loss = max(0.1, loss)
            
            train_accuracies.append(train_acc)
            val_accuracies.append(val_acc)
            losses.append(loss)
            
            # Show progress
            print(f"Training accuracy: {train_acc:.3f} ({train_acc*100:.1f}%)")
            print(f"Validation accuracy: {val_acc:.3f} ({val_acc*100:.1f}%)")
            print(f"Loss: {loss:.3f}")
            
            # Simulate training time
            time.sleep(0.5)
            
            # Show improvement tips
            if epoch == 3:
                print("\n💡 Tip: Model is learning basic patterns!")
            elif epoch == 7:
                print("\n💡 Tip: Fine-tuning disease-specific features!")
        
        # Plot training progress
        self.plot_training_progress(train_accuracies, val_accuracies, losses)
        
        return train_accuracies, val_accuracies, losses

    def plot_training_progress(self, train_acc, val_acc, losses):
        """Plot training progress"""
        fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(12, 5))
        
        epochs = range(1, len(train_acc) + 1)
        
        # Accuracy plot
        ax1.plot(epochs, train_acc, 'b-', label='Training Accuracy', linewidth=2)
        ax1.plot(epochs, val_acc, 'r-', label='Validation Accuracy', linewidth=2)
        ax1.set_xlabel('Epoch')
        ax1.set_ylabel('Accuracy')
        ax1.set_title('Training Progress: Accuracy')
        ax1.legend()
        ax1.grid(True)
        ax1.set_ylim(0, 1)
        
        # Loss plot
        ax2.plot(epochs, losses, 'g-', label='Loss', linewidth=2)
        ax2.set_xlabel('Epoch')
        ax2.set_ylabel('Loss')
        ax2.set_title('Training Progress: Loss')
        ax2.legend()
        ax2.grid(True)
        
        plt.tight_layout()
        plt.show()

    def evaluate_model(self):
        """Simulate model evaluation"""
        print("\n📊 Step 3: Evaluating Model")
        print("=" * 30)
        
        # Simulate test results
        test_accuracy = random.uniform(0.88, 0.94)
        
        print(f"Test Accuracy: {test_accuracy:.3f} ({test_accuracy*100:.1f}%)")
        
        # Simulate confusion matrix
        print("\n🎯 Confusion Matrix:")
        print("=" * 20)
        
        # Create a simple confusion matrix
        diseases_list = list(self.diseases.keys())
        matrix = np.zeros((len(diseases_list), len(diseases_list)))
        
        for i in range(len(diseases_list)):
            for j in range(len(diseases_list)):
                if i == j:  # Correct predictions
                    matrix[i][j] = random.randint(15, 20)
                else:  # Incorrect predictions
                    matrix[i][j] = random.randint(0, 3)
        
        # Print confusion matrix
        print(f"{'':15}", end="")
        for disease in diseases_list:
            print(f"{disease[:8]:>8}", end="")
        print()
        
        for i, disease in enumerate(diseases_list):
            print(f"{disease[:15]:15}", end="")
            for j in range(len(diseases_list)):
                print(f"{int(matrix[i][j]):8}", end="")
            print()
        
        # Calculate metrics
        precision = {}
        recall = {}
        f1_score = {}
        
        for i, disease in enumerate(diseases_list):
            # Precision = TP / (TP + FP)
            tp = matrix[i][i]
            fp = sum(matrix[j][i] for j in range(len(diseases_list)) if j != i)
            precision[disease] = tp / (tp + fp) if (tp + fp) > 0 else 0
            
            # Recall = TP / (TP + FN)
            fn = sum(matrix[i][j] for j in range(len(diseases_list)) if j != i)
            recall[disease] = tp / (tp + fn) if (tp + fn) > 0 else 0
            
            # F1 Score = 2 * (precision * recall) / (precision + recall)
            p, r = precision[disease], recall[disease]
            f1_score[disease] = 2 * p * r / (p + r) if (p + r) > 0 else 0
        
        # Print metrics
        print("\n📈 Performance Metrics:")
        print("=" * 25)
        print(f"{'Disease':15} {'Precision':>10} {'Recall':>10} {'F1-Score':>10}")
        print("-" * 50)
        
        for disease in diseases_list:
            print(f"{disease[:15]:15} {precision[disease]:>9.3f} {recall[disease]:>9.3f} {f1_score[disease]:>9.3f}")
        
        # Overall metrics
        avg_precision = sum(precision.values()) / len(precision)
        avg_recall = sum(recall.values()) / len(recall)
        avg_f1 = sum(f1_score.values()) / len(f1_score)
        
        print("-" * 50)
        print(f"{'Average':15} {avg_precision:>9.3f} {avg_recall:>9.3f} {avg_f1:>9.3f}")
        
        return test_accuracy, precision, recall, f1_score

    def make_predictions(self):
        """Simulate making predictions on new images"""
        print("\n🔮 Step 4: Making Predictions")
        print("=" * 30)
        
        sample_predictions = [
            {"image": "tomato_leaf_001.jpg", "actual": "Healthy", "predicted": "Healthy", "confidence": 0.94},
            {"image": "tomato_leaf_002.jpg", "actual": "Early Blight", "predicted": "Early Blight", "confidence": 0.89},
            {"image": "tomato_leaf_003.jpg", "actual": "Late Blight", "predicted": "Late Blight", "confidence": 0.92},
            {"image": "tomato_leaf_004.jpg", "actual": "Leaf Mold", "predicted": "Healthy", "confidence": 0.67},
            {"image": "tomato_leaf_005.jpg", "actual": "Bacterial Spot", "predicted": "Bacterial Spot", "confidence": 0.85}
        ]
        
        print("Sample Predictions:")
        print("-" * 60)
        print(f"{'Image':20} {'Actual':15} {'Predicted':15} {'Confidence':>10}")
        print("-" * 60)
        
        correct = 0
        for pred in sample_predictions:
            status = "✅" if pred["actual"] == pred["predicted"] else "❌"
            print(f"{pred['image']:20} {pred['actual']:15} {pred['predicted']:15} {pred['confidence']:>9.2f} {status}")
            
            if pred["actual"] == pred["predicted"]:
                correct += 1
        
        accuracy = correct / len(sample_predictions)
        print("-" * 60)
        print(f"Sample Accuracy: {accuracy:.2f} ({accuracy*100:.1f}%)")
        
        # Explain confidence scores
        print("\n🎯 Understanding Confidence Scores:")
        for pred in sample_predictions:
            conf = pred["confidence"]
            if conf >= 0.9:
                level = "Very High"
            elif conf >= 0.7:
                level = "High"
            elif conf >= 0.5:
                level = "Medium"
            else:
                level = "Low"
            
            print(f"  {pred['image']}: {conf:.2f} - {level} confidence")

    def show_improvement_tips(self):
        """Show tips for improving model performance"""
        print("\n💡 Tips for Improving Model Performance:")
        print("=" * 40)
        
        tips = [
            "📸 Collect more diverse training data",
            "🎨 Use data augmentation techniques",
            "⚖️ Balance the dataset (equal samples per class)",
            "🔍 Remove low-quality or mislabeled images",
            "🎛️ Adjust learning rate and training parameters",
            "⏱️ Train for more epochs (but watch for overfitting)",
            "🏗️ Try different model architectures",
            "📊 Use ensemble methods (combine multiple models)",
            "🌍 Test on real-world field conditions",
            "👥 Get expert validation from plant pathologists"
        ]
        
        for tip in tips:
            print(f"  {tip}")
        
        print("\n🚨 Common Mistakes to Avoid:")
        mistakes = [
            "❌ Using too few training samples",
            "❌ Ignoring data quality",
            "❌ Overfitting to training data",
            "❌ Not testing in real conditions",
            "❌ Forgetting to validate with experts"
        ]
        
        for mistake in mistakes:
            print(f"  {mistake}")

    def run_demo(self):
        """Run the complete training demo"""
        print("\n🎬 Starting Complete Training Demo")
        print("=" * 40)
        
        # Step 1: Explain diseases
        input("\nPress Enter to learn about tomato diseases...")
        self.explain_diseases()
        
        # Step 2: Prepare dataset
        input("\nPress Enter to prepare the dataset...")
        self.prepare_dataset()
        
        # Step 3: Train model
        input("\nPress Enter to start training...")
        train_acc, val_acc, losses = self.simulate_training()
        
        # Step 4: Evaluate model
        input("\nPress Enter to evaluate the model...")
        test_acc, precision, recall, f1 = self.evaluate_model()
        
        # Step 5: Make predictions
        input("\nPress Enter to see sample predictions...")
        self.make_predictions()
        
        # Step 6: Show improvement tips
        input("\nPress Enter to see improvement tips...")
        self.show_improvement_tips()
        
        # Summary
        print("\n🎉 Training Demo Complete!")
        print("=" * 30)
        print(f"Final test accuracy: {test_acc:.3f} ({test_acc*100:.1f}%)")
        print("\nYou now understand:")
        print("  ✅ How AI models learn from data")
        print("  ✅ The importance of good training data")
        print("  ✅ How to evaluate model performance")
        print("  ✅ Techniques for improving accuracy")
        print("\nReady for Module 3: Gemini Vision API! 🚀")


def main():
    """Main function to run the demo"""
    demo = TomatoTrainingDemo()
    
    while True:
        print("\n🍅 Tomato Training Demo Menu:")
        print("=" * 30)
        print("1. 🎬 Run complete demo")
        print("2. 🦠 Learn about diseases")
        print("3. 📚 Understand dataset preparation")
        print("4. 🚀 See training simulation")
        print("5. 📊 See evaluation metrics")
        print("6. 🔮 See sample predictions")
        print("7. 💡 Get improvement tips")
        print("8. 🚪 Exit")
        
        choice = input("\nEnter your choice (1-8): ").strip()
        
        if choice == '1':
            demo.run_demo()
        elif choice == '2':
            demo.explain_diseases()
        elif choice == '3':
            demo.prepare_dataset()
        elif choice == '4':
            demo.simulate_training()
        elif choice == '5':
            demo.evaluate_model()
        elif choice == '6':
            demo.make_predictions()
        elif choice == '7':
            demo.show_improvement_tips()
        elif choice == '8':
            print("👋 Thanks for learning about tomato disease detection!")
            break
        else:
            print("❌ Invalid choice. Please try again.")


if __name__ == "__main__":
    main() 