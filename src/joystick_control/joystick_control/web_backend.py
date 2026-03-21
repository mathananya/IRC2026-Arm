import os
import sys
import math
import threading
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32MultiArray, Int32, Bool

from fastapi import FastAPI, HTTPException, WebSocket, WebSocketDisconnect
from fastapi.middleware.cors import CORSMiddleware
from pydantic import BaseModel
import uvicorn

# Import local scripts directly so this can be run standalone
from IK_Func import calculateIK, check_lim, FKVerify
from calculate_encoders import angle_lower_to_encoder, angle_upper_to_encoder

# Data Models
class MoveTarget(BaseModel):
    X: float
    Z: float

class MoveRelative(BaseModel):
    delta_X: float
    delta_Z: float

class WebBridgeNode(Node):
    def __init__(self):
        super().__init__('web_backend_bridge')
        
        # Publishers
        self.target_pub = self.create_publisher(Int32MultiArray, 'arm_target_states', 10)
        self.ik_toggle_pub = self.create_publisher(Int32, 'ik_toggle_state', 10)

        # Subscribers
        self.encoder_sub = self.create_subscription(Int32MultiArray, 'arm_encoder_data', self.encoder_callback, 10)
        self.busy_sub = self.create_subscription(Bool, 'arm_pid_busy', self.busy_callback, 10)
        self.toggle_sub = self.create_subscription(Int32, 'ik_toggle_state', self.ik_callback, 10)
        
        # State vars
        self.current_lower_angle = 0.0
        self.current_upper_angle = 0.0
        self.current_X = 0.0
        self.current_Z = 0.0
        self.is_busy = False
        self.ik_toggle = 0

        # Run startup initialization
        self.init_timer = self.create_timer(2.0, self.startup_init)

    def startup_init(self):
        # Fire once
        if self.init_timer:
            self.init_timer.cancel()
            self.init_timer = None
        self.get_logger().info("Initializing arm to 0, 90 degrees")
        lower_enc = angle_lower_to_encoder(0.0)
        upper_enc = angle_upper_to_encoder(90.0)
        self.publish_target_encoders(int(lower_enc), int(upper_enc))

    def encoder_to_angle_lower(self, encoder_value):
        return 0.05689 * encoder_value - 87.22924

    def encoder_to_angle_upper(self, encoder_value):
        return -0.02863 * encoder_value + 101.58735

    def encoder_callback(self, msg):
        if len(msg.data) >= 2:
            lower_enc = msg.data[0]
            upper_enc = msg.data[1]
            
            # Convert to angles
            alpha = self.encoder_to_angle_lower(lower_enc)
            beta = self.encoder_to_angle_upper(upper_enc)
            
            self.current_lower_angle = alpha
            self.current_upper_angle = beta
            
            # Convert angles to current X and Z using FKVerify
            # FKVerify expects angles in radians
            alpha_rad = math.radians(alpha)
            beta_rad = math.radians(beta)
            X, Z = FKVerify(alpha_rad, beta_rad)
            
            self.current_X = X
            self.current_Z = Z

    def busy_callback(self, msg):
        self.is_busy = msg.data

    def ik_callback(self, msg):
        self.ik_toggle = msg.data
        
    def publish_target_encoders(self, lower_enc, upper_enc):
        msg = Int32MultiArray()
        msg.data = [lower_enc, upper_enc, 0, 0, 0]
        self.target_pub.publish(msg)
        
    def toggle_ik(self):
        self.ik_toggle = 1 if self.ik_toggle == 0 else 0
        msg = Int32()
        msg.data = self.ik_toggle
        self.ik_toggle_pub.publish(msg)

from fastapi.staticfiles import StaticFiles

# ... existing code up to app ...
ros_node: WebBridgeNode = None
app = FastAPI(title="Robotic Arm Control API")

# Add CORS Middleware to allow requests from the web dashboard
app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],  # Adjust in production
    allow_methods=["*"],
    allow_headers=["*"],
)

frontend_path = os.path.join(os.path.dirname(os.path.dirname(os.path.dirname(__file__))), "arm_frontend")
if os.path.exists(frontend_path):
    app.mount("/ui", StaticFiles(directory=frontend_path, html=True), name="ui")


@app.websocket("/ws")
async def websocket_endpoint(websocket: WebSocket):
    import asyncio
    await websocket.accept()
    try:
        while True:
            if ros_node:
                data = {
                    "alpha": round(ros_node.current_lower_angle, 2),
                    "beta": round(ros_node.current_upper_angle, 2),
                    "X": round(ros_node.current_X, 2),
                    "Z": round(ros_node.current_Z, 2),
                    "is_busy": ros_node.is_busy,
                    "ik_toggle": ros_node.ik_toggle == 1
                }
                await websocket.send_json(data)
            await asyncio.sleep(0.05) # 20 Hz updates
    except WebSocketDisconnect:
        pass

@app.get("/state")
def get_state():
    if not ros_node:
        raise HTTPException(status_code=503, detail="ROS Node not initialized")
    return {
        "alpha": round(ros_node.current_lower_angle, 2),
        "beta": round(ros_node.current_upper_angle, 2),
        "X": round(ros_node.current_X, 2),
        "Z": round(ros_node.current_Z, 2),
        "is_busy": ros_node.is_busy,
        "ik_toggle": ros_node.ik_toggle == 1
    }

@app.post("/ik-toggle")
def toggle_ik():
    if not ros_node:
        raise HTTPException(status_code=503, detail="ROS Node not initialized")
    ros_node.toggle_ik()
    return {"status": "success", "ik_toggle": ros_node.ik_toggle == 1}

def resolve_ik_and_publish(target_X: float, target_Z: float):
    # 1. Calculate IK solutions
    solutions = calculateIK(target_X, target_Z)
    if not solutions:
        raise HTTPException(status_code=400, detail="Target is out of reach.")
        
    sol1, sol2 = solutions
    
    # 2. Check limits for both solutions
    sol1_valid = check_lim(*sol1)
    sol2_valid = check_lim(*sol2)
    
    if not sol1_valid and not sol2_valid:
        raise HTTPException(status_code=400, detail="Both solutions exceed joint limits.")
        
    # 3. Choose the optimal (quicker to reach) solution
    best_sol = None
    curr_alpha = ros_node.current_lower_angle
    curr_beta = ros_node.current_upper_angle
    
    def angle_diff(sol):
        return abs(sol[0] - curr_alpha) + abs(sol[1] - curr_beta)
        
    if sol1_valid and sol2_valid:
        best_sol = sol1 if angle_diff(sol1) < angle_diff(sol2) else sol2
    elif sol1_valid:
        best_sol = sol1
    else:
        best_sol = sol2
        
    # 4. Convert chosen angles to encoders
    lower_angle, upper_angle = best_sol
    lower_encoder = angle_lower_to_encoder(lower_angle)
    upper_encoder = angle_upper_to_encoder(upper_angle)
    
    # 5. Publish target to ROS
    ros_node.publish_target_encoders(int(lower_encoder), int(upper_encoder))
    return {
        "status": "success", 
        "target_X": target_X, 
        "target_Z": target_Z, 
        "chosen_angles": best_sol, 
        "target_encoders": [lower_encoder, upper_encoder]
    }

@app.post("/move/ik")
def move_ik(target: MoveTarget):
    if not ros_node:
        raise HTTPException(status_code=503, detail="ROS Node not initialized")
    if ros_node.is_busy:
        raise HTTPException(status_code=400, detail="Arm is currently busy.")
        
    return resolve_ik_and_publish(target.X, target.Z)

@app.post("/move/relative")
def move_relative(target: MoveRelative):
    if not ros_node:
        raise HTTPException(status_code=503, detail="ROS Node not initialized")
    if ros_node.is_busy:
        raise HTTPException(status_code=400, detail="Arm is currently busy.")
        
    new_X = ros_node.current_X + target.delta_X
    new_Z = ros_node.current_Z + target.delta_Z
    
    return resolve_ik_and_publish(new_X, new_Z)

def start_ros():
    rclpy.init()
    global ros_node
    ros_node = WebBridgeNode()
    rclpy.spin(ros_node)
    ros_node.destroy_node()
    rclpy.shutdown()

def main():
    # Start ROS 2 loop in a background thread
    ros_thread = threading.Thread(target=start_ros, daemon=True)
    ros_thread.start()
    
    # Start FastAPI server on the main thread
    uvicorn.run(app, host="0.0.0.0", port=8000)

if __name__ == "__main__":
    main()
