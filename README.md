# Vehicle SafeStop AI (Team 4)

## Overview
This repository contains the ROS2 implementation for the "Vehicle SafeStop AI" project.
It includes nodes for fault detection, emergency path planning, safe stop control, V2X communication, and HMI alerts.

## Requirements
- ROS2 (Humble or Foxy recommended)
- `rclpy`
- `nav2_msgs`
- `paho-mqtt` (`pip install paho-mqtt`)

## Installation
1. Clone this repository into your ROS2 workspace `src` folder.
2. Build the package:
   ```bash
   colcon build --packages-select safestop_ai
   ```
3. Source the setup:
   ```bash
   source install/setup.bash
   ```

## Usage
Run the entire system with the launch file:
```bash
ros2 launch safestop_ai safestop.launch.py
```

## Nodes
- **health_monitor**: Checks sensor timeouts and publishes `/fault_trigger`.
- **emergency_planner**: Listens for faults and sends emergency goals to Nav2.
- **safe_stop_control**: Intercepts `/cmd_vel` to enforce jerk-limited stops.
- **v2x_bridge**: Publishes fault alerts to MQTT (`test.mosquitto.org`).
- **hmi_alert**: Visualizes faults in RViz and logs to console.

## Simulation
To test fault injection:
1. Run the launch file.
2. Manually stop a sensor (e.g., kill the camera node or block the laser).
3. Or manually publish a fault:
   ```bash
   ros2 topic pub /fault_trigger std_msgs/msg/String "data: '{\"type\": \"TEST_FAULT\", \"detail\": \"Manual Trigger\"}'" -1
   ```

## Verification (Sim-First)
To validate the logic without hardware or Gazebo, use the verification launch file.
This runs a `mock_sensors` node and a `test_scenario_runner` that automatically injects a fault and checks if the system detects it.

```bash
ros2 launch safestop_ai verification.launch.py
```
**Expected Output:**
1. System starts.
2. `test_scenario_runner` waits 5s.
3. Injects `fail_lidar`.
4. `health_monitor` detects `SENSOR_LOSS_LIDAR`.
5. `test_scenario_runner` prints `TEST PASSED`.

## Investor Dashboard (Web GUI)
For a premium visual demonstration:

1.  **Install ROS Bridge**:
    ```bash
    sudo apt install ros-humble-rosbridge-suite
    ```
2.  **Run the System**:
    ```bash
    ros2 launch safestop_ai verification.launch.py
    ```
3.  **Start the Dashboard**:
    ```bash
    cd safestop_dashboard
    npm install
    npm run dev
    ```
4.  **Open Browser**: Go to `http://localhost:5173`.
    - You will see the "Connected" status.
    - Click "Inject LiDAR Loss" to see the system react in real-time!
