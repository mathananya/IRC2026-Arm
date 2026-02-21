"""
ROS 2 Inverse Kinematics GUI Application

A Tkinter-based GUI for computing and publishing inverse kinematics solutions
for a robotic arm. Converts Cartesian (x, z) coordinates to encoder values
and publishes them via ROS 2.

Author: ROS 2 Engineering Team
License: Apache 2.0
"""

import threading
import tkinter as tk
from tkinter import messagebox
from typing import List, Tuple
from poseToPWM import pose_to_encoder


import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32MultiArray

class IKPublisherNode(Node):
    """ROS 2 node for publishing inverse kinematics encoder values."""
    
    def __init__(self):
        """Initialize the IK publisher node."""
        super().__init__('ik_gui_publisher')
        
        self.publisher = self.create_publisher(
            Int32MultiArray,
            'arm_target_states',
            10
        )
        
        self.get_logger().info('IK Publisher Node initialized')
    
    def publish_encoder_values(self, encoder_values: List[int]) -> None:
        """
        Publish encoder values to the arm_target_states topic.
        
        Args:
            encoder_values: List of encoder values to publish
        """
        msg = Int32MultiArray()
        if(encoder_values[1] <= 230 or encoder_values[1] >= 610):
            self.get_logger().info("Bhang bhosda value")
            self.get_logger().info(f'Recieved encoder values: {encoder_values[1]}, {encoder_values[0]}')
            # msg.data = [0, encoder_values[0],0,0,0]
            # self.publisher.publish(msg)
            # self.get_logger().info(f"Published vals : 0, {encoder_values[0]}")
            

        elif(encoder_values[0] <= 55 or encoder_values[0] >= 520):
            self.get_logger().info("Bhang bhosda value")
            self.get_logger().info(f'Recieved encoder values: {encoder_values[1]}, {encoder_values[0]}')
            # msg.data =  [encoder_values[1],0,0,0,0]
            # self.publisher.publish(msg)
            # self.get_logger().info(f"Published vals : {encoder_values[1]}, 0")


        else:
            msg.data = [encoder_values[1], encoder_values[0], 0, 0, 0]
            self.publisher.publish(msg)
            self.get_logger().info(f"Published vals : {encoder_values[1]}, {encoder_values[0]}")


# =============================================================================
# Tkinter GUI Application
# =============================================================================

class IKGUIApplication:
    """Main GUI application for inverse kinematics control."""
    
    def __init__(self, root: tk.Tk, ros_node: IKPublisherNode):
        """
        Initialize the GUI application.
        
        Args:
            root: Tkinter root window
            ros_node: ROS 2 node instance for publishing
        """
        self.root = root
        self.ros_node = ros_node
        
        # Configure main window
        self.root.title("ROS 2 Inverse Kinematics Control")
        self.root.geometry("450x350")
        self.root.resizable(False, False)
        
        # Build GUI components
        self._create_widgets()
        
        # Handle window close event
        self.root.protocol("WM_DELETE_WINDOW", self._on_closing)
    
    def _create_widgets(self) -> None:
        """Create and layout all GUI widgets."""
        
        # Main frame with padding
        main_frame = tk.Frame(self.root, padx=20, pady=20)
        main_frame.pack(fill=tk.BOTH, expand=True)
        
        # Title label
        title_label = tk.Label(
            main_frame,
            text="Inverse Kinematics Calculator",
            font=("Arial", 16, "bold")
        )
        title_label.grid(row=0, column=0, columnspan=2, pady=(0, 20))
        
        # Input section
        input_frame = tk.LabelFrame(
            main_frame,
            text="Cartesian Coordinates",
            padx=15,
            pady=15,
            font=("Arial", 10, "bold")
        )
        input_frame.grid(row=1, column=0, columnspan=2, sticky="ew", pady=(0, 15))
        
        # X coordinate input
        tk.Label(input_frame, text="X Coordinate:", font=("Arial", 10)).grid(
            row=0, column=0, sticky="w", pady=5
        )
        self.x_entry = tk.Entry(input_frame, width=20, font=("Arial", 10))
        self.x_entry.grid(row=0, column=1, padx=(10, 0), pady=5)
        self.x_entry.insert(0, "0.0")
        
        # Z coordinate input
        tk.Label(input_frame, text="Z Coordinate:", font=("Arial", 10)).grid(
            row=1, column=0, sticky="w", pady=5
        )
        self.z_entry = tk.Entry(input_frame, width=20, font=("Arial", 10))
        self.z_entry.grid(row=1, column=1, padx=(10, 0), pady=5)
        self.z_entry.insert(0, "0.0")
        
        # Compute and publish button
        self.compute_button = tk.Button(
            main_frame,
            text="Compute IK & Publish",
            command=self._on_compute_clicked,
            font=("Arial", 11, "bold"),
            bg="#0000EC",
            fg="white",
            padx=20,
            pady=10,
            cursor="hand2"
        )
        self.compute_button.grid(row=2, column=0, columnspan=2, pady=(0, 15))
        
        # Output section
        output_frame = tk.LabelFrame(
            main_frame,
            text="Published Encoder Values",
            padx=15,
            pady=15,
            font=("Arial", 10, "bold")
        )
        output_frame.grid(row=3, column=0, columnspan=2, sticky="ew")
        
        # Encoder values display (read-only)
        self.encoder_display = tk.Text(
            output_frame,
            height=3,
            width=40,
            font=("Courier", 10),
            state=tk.DISABLED,
            bg="#f0f0f0"
        )
        self.encoder_display.pack()
        
        # Status label
        self.status_label = tk.Label(
            main_frame,
            text="Ready",
            font=("Arial", 9),
            fg="gray"
        )
        self.status_label.grid(row=4, column=0, columnspan=2, pady=(10, 0))
    
    def _on_compute_clicked(self) -> None:
        """Handle compute button click event."""
        try:
            # Get and validate input values
            x, z = self._get_validated_inputs()
            
            # Update status
            self._update_status("Computing IK...", "blue")
            self.root.update_idletasks()
            
            # Compute encoder values
            encoder_values = list(pose_to_encoder(x, z))
            
            # Publish to ROS topic
            self.ros_node.publish_encoder_values(encoder_values)
            
            # Update display
            self._update_encoder_display(encoder_values)
            self._update_status("Published successfully", "green")
            
        except ValueError as e:
            # Handle invalid input or unreachable pose
            messagebox.showerror("Input Error", str(e))
            self._update_status("Error: Invalid input", "red")
        
        except Exception as e:
            # Handle unexpected errors
            messagebox.showerror("Computation Error", f"An error occurred: {str(e)}")
            self._update_status("Error: Computation failed", "red")
    
    def _get_validated_inputs(self) -> Tuple[float, float]:
        """
        Retrieve and validate input coordinates.
        
        Returns:
            Tuple of (x, z) float coordinates
        
        Raises:
            ValueError: If inputs are not valid numbers
        """
        try:
            x = float(self.x_entry.get().strip())
            z = float(self.z_entry.get().strip())
            return x, z
        except ValueError:
            raise ValueError("Please enter valid numeric values for X and Z coordinates")
    
    def _update_encoder_display(self, encoder_values: List[int]) -> None:
        """
        Update the encoder values display.
        
        Args:
            encoder_values: List of encoder values to display
        """
        self.encoder_display.config(state=tk.NORMAL)
        self.encoder_display.delete(1.0, tk.END)
        
        display_text = f"Encoder Values: {encoder_values[1]}, {encoder_values[0]}\n"
        display_text += f"Number of Joints: {len(encoder_values)}\n"
        # display_text += f"Values: {', '.join(map(str, encoder_values))}"
        
        self.encoder_display.insert(1.0, display_text)
        self.encoder_display.config(state=tk.DISABLED)
    
    def _update_status(self, message: str, color: str = "gray") -> None:
        """
        Update the status label.
        
        Args:
            message: Status message to display
            color: Text color for the status
        """
        self.status_label.config(text=message, fg=color)
    
    def _on_closing(self) -> None:
        """Handle window close event with proper ROS shutdown."""
        if messagebox.askokcancel("Quit", "Do you want to quit the application?"):
            self.root.quit()


# =============================================================================
# Main Application Entry Point
# =============================================================================

def ros_spin_thread(node: Node, stop_event: threading.Event) -> None:
    """
    ROS 2 spin thread to keep the node responsive.
    
    Args:
        node: ROS 2 node to spin
        stop_event: Threading event to signal shutdown
    """
    while not stop_event.is_set() and rclpy.ok():
        rclpy.spin_once(node, timeout_sec=0.1)


def main():
    """Main entry point for the application."""
    
    # Initialize ROS 2
    rclpy.init()
    
    # Create ROS node
    ik_node = IKPublisherNode()
    
    # Create Tkinter root window
    root = tk.Tk()
    
    # Create GUI application
    app = IKGUIApplication(root, ik_node)
    
    # Setup ROS spinning in separate thread
    stop_event = threading.Event()
    ros_thread = threading.Thread(
        target=ros_spin_thread,
        args=(ik_node, stop_event),
        daemon=True
    )
    ros_thread.start()
    
    try:
        # Start Tkinter main loop
        root.mainloop()
    
    finally:
        # Clean shutdown
        stop_event.set()
        ros_thread.join(timeout=2.0)
        ik_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()