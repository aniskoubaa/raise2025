# CropHealth.msg - Simple crop health monitoring data
# Educational example for RAISE 2025

string crop_type              # e.g., "tomato", "wheat", "corn"
string growth_stage           # e.g., "seedling", "flowering", "mature"
float64 health_score          # Health score (0-100, higher is better)
bool disease_detected         # True if disease found
string disease_type           # Disease name if detected (empty if none)
string location              # Field location identifier
int64 timestamp              # Unix timestamp when assessment was made 