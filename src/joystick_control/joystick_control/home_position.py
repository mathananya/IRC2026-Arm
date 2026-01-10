import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from std_msgs.msg import Int32MultiArray
from std_msgs.msg import Float32MultiArray

class HomeSubscriber(Node):
    def __init__(self):
        super().__init__('home_subscriber')
        self.subscription = self.create_subscription(
            Joy,
            'joy',  # Replace with the topic name you are subscribing to
            self.joy_callback,
            10  # QoS depth
        )
        # self.current_state_subscriber = self.create_subscription(
        #     Int32MultiArray,
        #     'arm_encoder_data',
        #     self.current_state_callback,
        #     10
        # )
        self.publisher = self.create_publisher(
            Int32MultiArray,
            'arm_target_states',  # Replace with the topic name you want to publish to
            10  # QoS depth
        )
        self.subscription  # Prevent unused variable warning
        # self.current_states=[0,0]
        self.publish_flag = False
        # self.move_bisep = False
        # self.move_tricep = False
        # self.last_pressed =0 #0 for none ; 1 for bisep; 2 for tricep

    #     self.bisep_state_to_move=0
    #     self.tricep_state_to_move=0
    # def current_state_callback(self, msg):
    #     # Extract lower and upper actuator states from the encoder data
    #     self.current_states = msg.data[:2]

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
        go_home , go_90, go_drop,go_pick ,go_extend= msg.buttons[7],msg.buttons[8],msg.buttons[9],msg.buttons[10],msg.buttons[11]
        # bisep_state_to_move = self.current_states[0]
        # tricep_state_to_move = self.current_states[1]
        if(go_home == 0 and go_90==0 and go_drop==1 and go_pick==0 and go_extend==0):
            target_state =[315,820,0,0,0]
            self.publish_flag=True
        elif(go_home == 0 and go_90==1 and go_drop==0 and go_pick==0 and go_extend==0):
            target_state =[650,250,0,0,0]
            self.publish_flag=True
        elif(go_home == 1 and go_90==0 and go_drop==0 and go_pick==0 and go_extend==0):
            target_state =[365,785,0,0,0]
            self.publish_flag=True
        elif(go_home == 0 and go_90==0 and go_drop==0 and go_pick==1 and go_extend==0):
            #target_state =[490,420,0,0,0]
            target_state =[590,380,0,0,0]
            self.publish_flag=True
        elif(go_home == 0 and go_90==0 and go_drop==0 and go_pick==0 and go_extend==1):
            #target_state =[490,420,0,0,0]
            target_state =[850,410,0,0,0]
            self.publish_flag=True
        joystick_target = Int32MultiArray()
        if(self.publish_flag):
            joystick_target.data=target_state
            print(joystick_target)
            self.publish_flag=False
            self.publisher.publish(joystick_target)
def main(args=None):
    rclpy.init(args=args)
    joy_subscriber = HomeSubscriber()

    try:
        rclpy.spin(joy_subscriber)
    except KeyboardInterrupt:
        pass
    finally:
        joy_subscriber.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
