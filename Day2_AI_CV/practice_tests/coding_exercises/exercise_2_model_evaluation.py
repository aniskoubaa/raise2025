#!/usr/bin/env python3
"""
Exercise 2: Model Evaluation (8 points)
Test your ability to calculate and interpret performance metrics for agricultural AI models.

Task: Calculate accuracy, precision, recall, and F1-score metrics for a plant disease detection model.

Skills Tested:
- Performance metrics calculation
- Confusion matrix analysis
- Precision vs recall tradeoffs
- Agricultural context interpretation

Time: 12 minutes
"""

import numpy as np
import matplotlib.pyplot as plt
from sklearn.metrics import confusion_matrix, classification_report
import seaborn as sns

class ModelEvaluationExercise:
    def __init__(self):
        # Sample data for tomato disease detection
        self.disease_classes = ['Healthy', 'Early_Blight', 'Late_Blight', 'Leaf_Mold', 'Bacterial_Spot']
        
        # Sample predictions and ground truth
        self.sample_data = self.generate_sample_data()
        
    def generate_sample_data(self):
        """Generate sample prediction data for evaluation"""
        np.random.seed(42)  # For reproducible results
        
        # Ground truth labels (100 samples)
        true_labels = np.random.choice(self.disease_classes, 100, 
                                     p=[0.4, 0.15, 0.15, 0.15, 0.15])  # More healthy samples
        
        # Simulate model predictions with some errors
        pred_labels = []
        for true_label in true_labels:
            if np.random.random() < 0.85:  # 85% accuracy
                pred_labels.append(true_label)
            else:
                # Random wrong prediction
                wrong_choices = [c for c in self.disease_classes if c != true_label]
                pred_labels.append(np.random.choice(wrong_choices))
        
        return {
            'true_labels': true_labels,
            'pred_labels': pred_labels
        }
    
    def calculate_confusion_matrix(self, y_true, y_pred):
        """
        TODO: Calculate confusion matrix for the predictions
        
        Args:
            y_true: True labels
            y_pred: Predicted labels
            
        Returns:
            numpy array: Confusion matrix
        """
        
        # YOUR CODE HERE
        # Hint: Use sklearn.metrics.confusion_matrix
        # Make sure to use the correct labels parameter
        
        cm = confusion_matrix(y_true, y_pred, labels=self.disease_classes)
        return cm
    
    def calculate_accuracy(self, y_true, y_pred):
        """
        TODO: Calculate overall accuracy
        
        Args:
            y_true: True labels
            y_pred: Predicted labels
            
        Returns:
            float: Accuracy score (0-1)
        """
        
        # YOUR CODE HERE
        # Accuracy = (Correct Predictions) / (Total Predictions)
        
        correct = sum(1 for true, pred in zip(y_true, y_pred) if true == pred)
        total = len(y_true)
        accuracy = correct / total
        
        return accuracy
    
    def calculate_precision_recall_f1(self, y_true, y_pred, class_name):
        """
        TODO: Calculate precision, recall, and F1-score for a specific class
        
        Args:
            y_true: True labels
            y_pred: Predicted labels
            class_name: Name of the class to evaluate
            
        Returns:
            dict: {'precision': float, 'recall': float, 'f1': float}
        """
        
        # YOUR CODE HERE
        # Precision = TP / (TP + FP)
        # Recall = TP / (TP + FN)
        # F1 = 2 * (precision * recall) / (precision + recall)
        
        # Calculate True Positives, False Positives, False Negatives
        tp = sum(1 for true, pred in zip(y_true, y_pred) if true == class_name and pred == class_name)
        fp = sum(1 for true, pred in zip(y_true, y_pred) if true != class_name and pred == class_name)
        fn = sum(1 for true, pred in zip(y_true, y_pred) if true == class_name and pred != class_name)
        
        # Calculate metrics
        precision = tp / (tp + fp) if (tp + fp) > 0 else 0
        recall = tp / (tp + fn) if (tp + fn) > 0 else 0
        f1 = 2 * (precision * recall) / (precision + recall) if (precision + recall) > 0 else 0
        
        return {
            'precision': precision,
            'recall': recall,
            'f1': f1
        }
    
    def calculate_macro_metrics(self, y_true, y_pred):
        """
        TODO: Calculate macro-averaged precision, recall, and F1-score
        
        Args:
            y_true: True labels
            y_pred: Predicted labels
            
        Returns:
            dict: {'macro_precision': float, 'macro_recall': float, 'macro_f1': float}
        """
        
        # YOUR CODE HERE
        # Macro average = average of per-class metrics
        
        all_metrics = []
        for class_name in self.disease_classes:
            class_metrics = self.calculate_precision_recall_f1(y_true, y_pred, class_name)
            all_metrics.append(class_metrics)
        
        # Calculate macro averages
        macro_precision = np.mean([m['precision'] for m in all_metrics])
        macro_recall = np.mean([m['recall'] for m in all_metrics])
        macro_f1 = np.mean([m['f1'] for m in all_metrics])
        
        return {
            'macro_precision': macro_precision,
            'macro_recall': macro_recall,
            'macro_f1': macro_f1
        }
    
    def plot_confusion_matrix(self, cm, title="Confusion Matrix"):
        """
        TODO: Plot a confusion matrix heatmap
        
        Args:
            cm: Confusion matrix array
            title: Plot title
        """
        
        # YOUR CODE HERE
        # Use matplotlib and seaborn to create a nice heatmap
        
        plt.figure(figsize=(10, 8))
        sns.heatmap(cm, annot=True, fmt='d', cmap='Blues', 
                    xticklabels=self.disease_classes, 
                    yticklabels=self.disease_classes)
        plt.title(title)
        plt.xlabel('Predicted Label')
        plt.ylabel('True Label')
        plt.tight_layout()
        plt.savefig('confusion_matrix.png', dpi=300, bbox_inches='tight')
        plt.show()
    
    def analyze_class_performance(self, y_true, y_pred):
        """
        TODO: Analyze performance for each disease class
        
        Args:
            y_true: True labels
            y_pred: Predicted labels
            
        Returns:
            dict: Per-class performance metrics
        """
        
        # YOUR CODE HERE
        # Calculate metrics for each class and identify strengths/weaknesses
        
        class_performance = {}
        
        for class_name in self.disease_classes:
            metrics = self.calculate_precision_recall_f1(y_true, y_pred, class_name)
            
            # Count samples for this class
            class_count = sum(1 for label in y_true if label == class_name)
            
            class_performance[class_name] = {
                'precision': metrics['precision'],
                'recall': metrics['recall'],
                'f1': metrics['f1'],
                'sample_count': class_count
            }
        
        return class_performance
    
    def agricultural_interpretation(self, class_performance):
        """
        TODO: Provide agricultural interpretation of the results
        
        Args:
            class_performance: Per-class performance metrics
            
        Returns:
            dict: Agricultural insights and recommendations
        """
        
        # YOUR CODE HERE
        # Analyze what these metrics mean for farmers
        # Consider the cost of false positives vs false negatives for each disease
        
        insights = {
            'high_precision_classes': [],
            'high_recall_classes': [],
            'concerning_classes': [],
            'recommendations': []
        }
        
        for class_name, metrics in class_performance.items():
            precision = metrics['precision']
            recall = metrics['recall']
            
            if precision > 0.9:
                insights['high_precision_classes'].append(class_name)
            
            if recall > 0.9:
                insights['high_recall_classes'].append(class_name)
            
            if recall < 0.7:  # Missing many actual diseases
                insights['concerning_classes'].append(class_name)
                insights['recommendations'].append(
                    f"Improve {class_name} detection - missing {(1-recall)*100:.1f}% of cases"
                )
        
        # General recommendations
        if 'Healthy' in insights['concerning_classes']:
            insights['recommendations'].append(
                "Low recall for healthy plants may lead to unnecessary treatments"
            )
        
        for disease in ['Early_Blight', 'Late_Blight', 'Leaf_Mold', 'Bacterial_Spot']:
            if disease in insights['concerning_classes']:
                insights['recommendations'].append(
                    f"Critical: {disease} detection needs improvement to prevent crop losses"
                )
        
        return insights
    
    def run_exercise(self):
        """Run the complete exercise"""
        print("🎯 Exercise 2: Model Evaluation for Agriculture")
        print("=" * 45)
        
        # Load sample data
        y_true = self.sample_data['true_labels']
        y_pred = self.sample_data['pred_labels']
        
        print(f"📊 Evaluating model on {len(y_true)} samples")
        print(f"   Classes: {', '.join(self.disease_classes)}")
        
        # Calculate confusion matrix
        print("\n🔍 Calculating confusion matrix...")
        cm = self.calculate_confusion_matrix(y_true, y_pred)
        
        # Calculate overall accuracy
        print("\n📈 Calculating overall metrics...")
        accuracy = self.calculate_accuracy(y_true, y_pred)
        print(f"   Overall Accuracy: {accuracy:.3f} ({accuracy*100:.1f}%)")
        
        # Calculate macro metrics
        macro_metrics = self.calculate_macro_metrics(y_true, y_pred)
        print(f"   Macro Precision: {macro_metrics['macro_precision']:.3f}")
        print(f"   Macro Recall: {macro_metrics['macro_recall']:.3f}")
        print(f"   Macro F1-Score: {macro_metrics['macro_f1']:.3f}")
        
        # Per-class analysis
        print("\n📋 Per-class Performance:")
        class_performance = self.analyze_class_performance(y_true, y_pred)
        
        for class_name, metrics in class_performance.items():
            print(f"   {class_name:15} - P:{metrics['precision']:.3f} R:{metrics['recall']:.3f} F1:{metrics['f1']:.3f} (n={metrics['sample_count']})")
        
        # Agricultural interpretation
        print("\n🌱 Agricultural Interpretation:")
        insights = self.agricultural_interpretation(class_performance)
        
        if insights['high_precision_classes']:
            print(f"   ✅ High precision: {', '.join(insights['high_precision_classes'])}")
        
        if insights['high_recall_classes']:
            print(f"   ✅ High recall: {', '.join(insights['high_recall_classes'])}")
        
        if insights['concerning_classes']:
            print(f"   ⚠️  Concerning: {', '.join(insights['concerning_classes'])}")
        
        print("\n💡 Recommendations:")
        for rec in insights['recommendations']:
            print(f"   • {rec}")
        
        # Plot confusion matrix
        print("\n📊 Plotting confusion matrix...")
        self.plot_confusion_matrix(cm, "Tomato Disease Detection - Confusion Matrix")
        
        # Exercise questions
        self.ask_questions(accuracy, macro_metrics, class_performance)
        
        return {
            'accuracy': accuracy,
            'macro_metrics': macro_metrics,
            'class_performance': class_performance,
            'insights': insights
        }
    
    def ask_questions(self, accuracy, macro_metrics, class_performance):
        """Ask questions about the results"""
        print("\n❓ Exercise Questions:")
        print("=" * 25)
        
        print("1. For disease detection, which metric is MORE important?")
        print("   a) Precision (fewer false alarms)")
        print("   b) Recall (catch all actual diseases)")
        print("   c) Accuracy (overall correctness)")
        print("   d) F1-score (balance of precision and recall)")
        
        print("\n2. What does a low recall for 'Healthy' class mean?")
        print("   a) Model often misses healthy plants")
        print("   b) Model often treats healthy plants as diseased")
        print("   c) Model is very accurate for healthy plants")
        print("   d) Model cannot detect healthy plants")
        
        print(f"\n3. Your model's macro F1-score is {macro_metrics['macro_f1']:.3f}. This is:")
        if macro_metrics['macro_f1'] > 0.8:
            print("   Excellent performance for agricultural deployment")
        elif macro_metrics['macro_f1'] > 0.7:
            print("   Good performance but may need some improvement")
        elif macro_metrics['macro_f1'] > 0.6:
            print("   Moderate performance - significant improvement needed")
        else:
            print("   Poor performance - major revision required")
        
        print("\n4. Which disease class has the lowest recall in your results?")
        lowest_recall_class = min(class_performance.keys(), 
                                 key=lambda x: class_performance[x]['recall'])
        print(f"   Answer: {lowest_recall_class} (recall: {class_performance[lowest_recall_class]['recall']:.3f})")
        
        print("\n💡 Think about:")
        print("   • How would you improve the model's weak areas?")
        print("   • What's the cost of false positives vs false negatives in farming?")
        print("   • How do class imbalances affect your metrics?")
        print("   • What confidence threshold would you use for deployment?")

def main():
    """Main function to run the exercise"""
    exercise = ModelEvaluationExercise()
    
    try:
        results = exercise.run_exercise()
        print("\n🎉 Exercise 2 Complete!")
        print("📝 Understanding model evaluation is crucial for agricultural AI deployment.")
        
    except Exception as e:
        print(f"\n❌ Error in exercise: {e}")
        print("💡 Check your implementation and try again.")

if __name__ == "__main__":
    main() 