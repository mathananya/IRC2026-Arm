import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from std_msgs.msg import Int32MultiArray
from std_msgs.msg import Float32MultiArray
import math

class JoySubscriber(Node):
    def __init__(self):
        super().__init__('joy_subscriber')
        self.subscription = self.create_subscription(
            Joy,
            'joy',  # Replace with the topic name you are subscribing to
            self.joy_callback,
            10  # QoS depth
        )
        self.current_state_subscriber = self.create_subscription(
            Float32MultiArray,
            'arm_gripper_coordinates',
            self.current_state_callback,
            10
        )
        self.publisher = self.create_publisher(
            Int32MultiArray,
            'arm_target_states',  # Replace with the topic name you want to publish to
            10  # QoS depth
        )
        self.subscription  # Prevent unused variable warning
        self.current_x =0.0
        self.current_y =0.0
        
        self.move_x = False
        self.move_y = False
        self.last_pressed =0 #0 for none ; 1 for x; 2 for y

        self.x_state_to_move=0
        self.y_state_to_move=0

    def current_state_callback(self, msg):
        # Extract lower and upper actuator states from the encoder data
        self.current_x = msg.data[0]
        self.current_y = msg.data[1]

    def joy_callback(self, msg):
        
        # self.get_logger().info(f'Received joystick data:')
        # self.get_logger().info(f'Timestamp: {msg.header.stamp.sec}s {msg.header.stamp.nanosec}ns')
        # self.get_logger().info(f'Frame ID: {msg.header.frame_id}')
        # self.get_logger().info(f'Axes: {msg.axes}')
        # self.get_logger().info(f'Buttons: {msg.buttons}')

        # Process the joystick data and create an Int32MultiArray message
        # data_from_joystick = Float32MultiArray()
        # processed_data = Float32MultiArray()
        # pwm_publish = Int32MultiArray()
        x_flag , y_flag = msg.buttons[:2]
        # bisep_state_to_move = self.current_states[0]
        # tricep_state_to_move = self.current_states[1]
        if(x_flag == 1 and y_flag==0):
            self.move_x = True
            self.move_y = False
            self.last_pressed =1

        if(x_flag == 0 and y_flag==1):
            self.move_x = False
            self.move_y = True
            self.last_pressed =2
        
        if(x_flag == 1 and  y_flag==1):
            if(self.last_pressed == 0):
                self.move_x = False
                self.move_y = False
            elif(self.last_pressed == 1):
                self.move_x = True
                self.move_y = False
            elif(self.last_pressed == 2):
                self.move_x = False
                self.move_y = True

        if(y_flag==0 and y_flag ==0):
            self.move_x = False
            self.move_y = False
            self.last_pressed =0

        x_y_analog_value = msg.axes[1]
        #print(pwm_analog_value)

        if(self.move_x):
            x_current_state = self.current_x
            self.x_state_to_move = int(x_current_state + (x_y_analog_value*30))
        
        elif(self.move_y):
            y_current_state = self.current_y
            self.y_state_to_move = int(y_current_state + (x_y_analog_value*30))
        
        LA1_length = 50  # cm
        LA2_length = 64  # cm
        distance = math.sqrt(self.x_state_to_move**2 + self.y_state_to_move**2)
        
        # Check if the target is reachable
        if distance > (LA1_length + LA2_length) or distance < abs(LA1_length - LA2_length):
            raise ValueError("Target is out of reach")
        
        # Apply the law of cosines to calculate the required lengths of LA1 and LA2
        # For a 2-DOF arm with linear actuators, we can use the triangle formed by LA1, LA2, and the target point (x, y)
        
        # Cosine rule for LA2
        angle_LA2 = math.acos((distance**2 - LA1_length**2 - LA2_length**2) / (-2 * LA1_length * LA2_length))
        
        # Angle for LA1 (elbow angle) can be calculated using the law of cosines again
        angle_LA1 = math.atan2(y, x) - math.acos((LA1_length**2 + distance**2 - LA2_length**2) / (2 * LA1_length * distance))


        coordinate_target = Int32MultiArray()
        if(self.move_x or self.move_y):
            coordinate_target.data=[int(angle_LA1*7),int(angle_LA2*7),0,0,0]
            print(coordinate_target)
            self.publisher.publish(coordinate_target)
def main(args=None):
    rclpy.init(args=args)
    joy_subscriber = JoySubscriber()

    try:
        rclpy.spin(joy_subscriber)
    except KeyboardInterrupt:
        pass
    finally:
        joy_subscriber.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
