# SoilSensor.msg - Simple soil sensor data structure
# Educational example for RAISE 2025

float64 ph                    # pH level (0-14)
float64 moisture_percent      # Moisture percentage (0-100)
float64 temperature_celsius   # Temperature in Celsius
string location              # Location identifier (e.g., "field_A_zone_3")
int64 timestamp              # Unix timestamp when measurement was taken 