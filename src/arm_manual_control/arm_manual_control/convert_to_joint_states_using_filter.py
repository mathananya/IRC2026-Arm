import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32MultiArray
from std_msgs.msg import Float32MultiArray
import array

class EncoderToJointAnglesNode(Node):
    def __init__(self):
        super().__init__('encoder_to_joint_angles_node')
        self.LA1_factor=7.0
        self.LA2_factor=4.0
        # Subscriber to encoder topic
        self.subscription = self.create_subscription(
            Int32MultiArray,  # Message type
            'arm_encoder_data',          # Topic name
            self.encoder_callback,  # Callback function
            10                  # QoS
        )

        # Publisher for joint angles
        self.publisher = self.create_publisher(
            Float32MultiArray,  # Message type
            'arm_joint_states',     # Topic name
            10                  # QoS
        )

        # Parameters for filtering
        self.alpha = 0.5  # Smoothing factor for exponential moving average
        self.filtered_values = []

        self.get_logger().info("Encoder to Joint Angles Node Initialized")

    def encoder_callback(self, msg):
        # Extract potentiometer values from the message
        potentiometer_values = msg.data

        self.get_logger().info(f"Received potentiometer values: {potentiometer_values}")

        # Apply exponential moving average filter
        if not self.filtered_values:
            # Initialize filtered values with the first set of readings
            self.filtered_values = potentiometer_values[:2]
        else:
            # Update filtered values using exponential moving average
            self.filtered_values = [
                self.alpha * value + (1 - self.alpha) * filtered
                for value, filtered in zip(potentiometer_values[:2], self.filtered_values)
            ]

        self.get_logger().info(f"Filtered potentiometer values: {self.filtered_values}")

        # Convert filtered potentiometer values to joint angles (example calculation)
        # Assume potentiometer values are in the range [0, 1023] and joint angles are in radians
        #joint_angles = [(self.filtered_values[0] / self.LA1_factor) * 3.14159, (self.filtered_values[1] / self.LA2_factor) * 3.14159]+list(potentiometer_values[2:])
        #joint_angles = [(self.filtered_values[0] / self.LA1_factor) , (self.filtered_values[1] / self.LA2_factor) ]
        joint_angles = [(self.filtered_values[0]) , (self.filtered_values[1]) ]
        self.get_logger().info(f"Calculated joint angles: {joint_angles}")

        # Publish the joint angles
        joint_angles_msg = Float32MultiArray()
        print(type(joint_angles_msg),joint_angles)
        joint_angles_msg.data = array.array('f',joint_angles)
        self.publisher.publish(joint_angles_msg)
        
        self.get_logger().info("Published joint angles")

def main(args=None):
    rclpy.init(args=args)

    node = EncoderToJointAnglesNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Node interrupted, shutting down")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
