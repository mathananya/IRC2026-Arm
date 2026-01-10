import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from std_msgs.msg import Int32MultiArray
from std_msgs.msg import Float32MultiArray

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
            Int32MultiArray,
            'arm_encoder_data',
            self.current_state_callback,
            10
        )
        self.publisher = self.create_publisher(
            Int32MultiArray,
            'arm_target_states',  # Replace with the topic name you want to publish to
            10  # QoS depth
        )
        self.subscription  # Prevent unused variable warning
        self.current_states=[0,0]
        
        self.move_bisep = False
        self.move_tricep = False
        self.last_pressed =0 #0 for none ; 1 for bisep; 2 for tricep

        self.bisep_state_to_move=0
        self.tricep_state_to_move=0
    def current_state_callback(self, msg):
        # Extract lower and upper actuator states from the encoder data
        self.current_states = msg.data[:2]

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
        bisep_flag , tricep_flag = msg.buttons[:2]
    
        # bisep_state_to_move = self.current_states[0]
        # tricep_state_to_move = self.current_states[1]
        if(bisep_flag == 1 and tricep_flag==0):
            self.move_bisep = True
            self.move_tricep = False
            self.last_pressed =1

        if(bisep_flag == 0 and tricep_flag==1):
            self.move_bisep = False
            self.move_tricep = True
            self.last_pressed =2
        
        if(tricep_flag == 1 and  tricep_flag==1):
            if(self.last_pressed == 0):
                self.move_bisep = False
                self.move_tricep = False
            elif(self.last_pressed == 1):
                self.move_bisep = True
                self.move_tricep = False
            elif(self.last_pressed == 2):
                self.move_bisep = False
                self.move_tricep = True

        if(tricep_flag==0 and bisep_flag ==0):
            self.move_bisep = False
            self.move_tricep = False
            self.last_pressed =0

        pwm_analog_value = msg.axes[1]
        #print(pwm_analog_value)

        if(self.move_bisep):
            bisep_current_state = self.current_states[0]
            self.bisep_state_to_move = int(bisep_current_state + (pwm_analog_value*40))
        
        elif(self.move_tricep):
            tricep_current_state = self.current_states[1]
            self.tricep_state_to_move = int(tricep_current_state + (pwm_analog_value*40))
        # else:
        #     self.tricep_state_to_move=0
        #     self.bisep_state_to_move=0
        joystick_target = Int32MultiArray()
        if(self.move_bisep or self.move_tricep):
            joystick_target.data=[self.bisep_state_to_move,self.tricep_state_to_move,0,0,0]
            print(joystick_target)
            self.publisher.publish(joystick_target)
        elif(self.move_bisep == False and self.move_tricep == False):
            joystick_target.data=[self.current_states[0],self.current_states[1],0,0,0]
            print(joystick_target)
            self.publisher.publish(joystick_target)
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
