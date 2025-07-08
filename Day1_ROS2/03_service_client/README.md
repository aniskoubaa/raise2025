# Module 3: Service/Client Architecture

## Learning Objectives
By the end of this module, you will be able to:
- Understand the difference between services and topics in ROS2
- Implement service servers and clients
- Handle synchronous request-response communication
- Apply service/client pattern to agricultural robotics scenarios

## Agricultural Context: Smart Irrigation System
In smart farming, irrigation systems need to respond to specific requests:
- **Irrigation Control Service**: Turn on/off water zones, set duration, check status
- **Soil Analysis Service**: Request soil moisture and pH readings
- **Weather Service**: Get weather forecasts for irrigation planning

## Service vs Topic: When to Use Which?

### Topics (Publish/Subscribe)
- **Continuous data streams**: sensor readings, robot positions
- **One-to-many communication**: broadcasting sensor data
- **Asynchronous**: fire-and-forget messages
- **Example**: Soil moisture sensor publishing readings every second

### Services (Request/Response)
- **On-demand operations**: start irrigation, calibrate sensors
- **Synchronous communication**: wait for confirmation
- **Point-to-point**: direct client-server interaction
- **Example**: Requesting irrigation system to water specific zone

## Service Interface Definition

ROS2 services use `.srv` files to define the interface:

```srv
# Request
string zone_id
float64 duration_minutes
bool emergency_stop
---
# Response
bool success
string message
float64 actual_duration
int32 water_flow_rate
```

## Key Concepts

### Service Server
- **Provides functionality**: implements the service callback
- **Waits for requests**: listens for incoming service calls
- **Processes and responds**: handles the request and sends response

### Service Client
- **Requests functionality**: calls services when needed
- **Blocks until response**: waits for server to complete
- **Handles responses**: processes the returned data

## Implementation Overview

### 1. Irrigation Control Service
- **Purpose**: Control water flow to different agricultural zones
- **Request**: Zone ID, duration, emergency stop flag
- **Response**: Success status, actual duration, flow rate

### 2. Soil Analysis Service
- **Purpose**: Provide on-demand soil condition analysis
- **Request**: Zone ID, analysis type (moisture, pH, nutrients)
- **Response**: Measurement values, recommendations

### 3. Multi-Client Farm Manager
- **Purpose**: Coordinate multiple farm operations
- **Functionality**: Calls irrigation and soil analysis services
- **Intelligence**: Makes decisions based on service responses

## Files in this Module

1. **`irrigation_service.py`** - Service server for irrigation control
2. **`soil_analysis_service.py`** - Service server for soil analysis
3. **`farm_manager_client.py`** - Client that coordinates farm operations
4. **`irrigation_client_simple.py`** - Simple client example
5. **`QUICKSTART.md`** - Step-by-step guide to run the examples

## Common Service Patterns

### Synchronous Call
```python
# Client waits for response
response = await client.call_async(request)
```

### Asynchronous Call with Callback
```python
# Client continues, handles response later
future = client.call_async(request)
future.add_done_callback(handle_response)
```

### Timeout Handling
```python
# Prevent infinite waiting
response = await asyncio.wait_for(
    client.call_async(request), 
    timeout=5.0
)
```

## Error Handling in Services

### Server-Side
- Validate request parameters
- Handle hardware failures gracefully
- Provide meaningful error messages
- Log service calls for debugging

### Client-Side
- Handle service unavailability
- Implement retry logic
- Timeout management
- Validate response data

## Agricultural Applications

### Precision Irrigation
- **Zone-specific watering**: Request irrigation for specific field zones
- **Duration control**: Set precise watering times
- **Emergency stops**: Immediately halt irrigation if needed

### Soil Health Monitoring
- **On-demand analysis**: Request soil tests when needed
- **Multi-parameter testing**: pH, moisture, nutrients in one call
- **Recommendations**: Get actionable advice from analysis

### Farm Coordination
- **Central management**: One client coordinating multiple services
- **Decision making**: Use service responses to make farm decisions
- **Scheduling**: Coordinate timing of different operations

## Best Practices

### Service Design
1. **Keep services stateless**: Each call should be independent
2. **Use appropriate timeouts**: Prevent clients from waiting forever
3. **Provide detailed responses**: Include status, messages, and data
4. **Handle errors gracefully**: Always respond, even on failure

### Client Implementation
1. **Check service availability**: Verify server is running before calling
2. **Implement retry logic**: Handle temporary failures
3. **Use async calls**: Don't block the main thread
4. **Validate responses**: Check success flags and data validity

## Integration with Other ROS2 Concepts

### With Topics
- Services can publish status updates to topics
- Clients can subscribe to topics for real-time updates
- Topics for continuous data, services for commands

### With Parameters
- Services can read/write ROS2 parameters
- Configure service behavior through parameters
- Dynamic reconfiguration of service settings

### With Actions
- Services for quick operations (< 1 second)
- Actions for long-running tasks (> 1 second)
- Services can start/stop actions

## Troubleshooting

### Common Issues

1. **Service not found**
   - Check if service server is running
   - Verify service name spelling
   - Use `ros2 service list` to see available services

2. **Request timeout**
   - Increase timeout duration
   - Check server processing time
   - Verify network connectivity

3. **Invalid request format**
   - Check service interface definition
   - Verify request parameter types
   - Use `ros2 service type <service_name>` to see interface

### Debug Commands
```bash
# List all services
ros2 service list

# Show service type
ros2 service type /irrigation_control

# Call service manually
ros2 service call /irrigation_control example_interfaces/srv/IrrigationControl "{zone_id: 'zone_1', duration_minutes: 5.0, emergency_stop: false}"

# Echo service calls
ros2 service echo /irrigation_control
```

## Next Steps
After completing this module, you'll understand:
- How to implement request-response patterns
- When to use services vs topics
- Error handling in distributed systems
- Synchronous communication in ROS2

Continue to **Module 4: Actions** to learn about long-running tasks and preemptable operations. 