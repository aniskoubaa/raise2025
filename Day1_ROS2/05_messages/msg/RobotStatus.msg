# RobotStatus.msg - Simple robot status data
# Educational example for RAISE 2025

string robot_id               # Unique robot identifier
string current_task           # e.g., "navigating", "harvesting", "idle"
float64 battery_percentage    # Battery level (0-100)
float64 position_x            # X coordinate in field
float64 position_y            # Y coordinate in field
string[] alerts              # List of current alerts
bool emergency_stop          # Emergency stop status
int64 timestamp              # Unix timestamp of status update 