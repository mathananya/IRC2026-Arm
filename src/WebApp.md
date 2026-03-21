# CRISS Arm Web Integration Dashboard

This folder contains a modern Web Dashboard (powered by Vanilla JS, CSS, and HTML) and a realtime FastAPI backend that connects directly to the ROS 2 nodes for controlling the robotic arm.

The dashboard allows you to:
- Instantly monitor **live telemetry** (IK state, Arm busyness, Encoder angles, and estimated XYZ coordinates) at 20Hz via WebSockets.
- Dispatch absolute targets `(X, Z)`.
- Dispatch relative incremental movements `(±100 in X or Z)` based dynamically on your live encoder layout.

## Setup Instructions

### 1. Setup the Python Environment
Since you are mixing a modern ASGI web server with ROS 2, it is highly recommended to isolate them in a virtual environment inside your package to prevent system conflicts.
```bash
cd joystick_control/joystick_control
# Create a virtual environment named 'ros2_venv'
python3 -m venv ros2_venv 

# Activate it
source ros2_venv/bin/activate

# Install the backend requirements
pip install -r requirements.txt
```

### 2. Start the Backend Server
Whenever you want to use the dashboard, simply activate that virtual environment and run the backend script. Note: since the backend interacts with ROS 2, you must ensure your underlying ROS 2 workspace is sourced as usual!
```bash
# 1. Source your main ROS 2 installation & workspace (e.g. Humble/Jazzy)
source /opt/ros/humble/setup.bash
# Alternatively source your local workspace install if you haven't natively included paths
source ../../install/setup.bash

# 2. Activate the python environment and run the server
cd joystick_control/joystick_control
source ros2_venv/bin/activate
python3 web_backend.py
```

### 3. Open the Dashboard
Once the Uvicorn server is running, the frontend is instantly hosted as static content alongside the python API.
Open any modern web browser and navigate to:
**[http://localhost:8000/ui/index.html](http://localhost:8000/ui/index.html)**

### 4. Hardware Sidenote
The Web Dashboard relies purely on the ROS topics `/arm_encoder_data`, `/arm_target_states`, and `/arm_pid_busy`. You can start your `arm_controller_refactor2` and encoder conversion nodes completely independently in other terminals just like you always do. The web backend will automatically detect them bridging data via the ROS network!
