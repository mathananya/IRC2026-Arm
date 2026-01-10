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
        self.publisher = self.create_publisher(
            Int32MultiArray,
            'arm_pwm_commands',  # Replace with the topic name you want to publish to
            10  # QoS depth
        )
        self.subscription  # Prevent unused variable warning

        self.wrist_pwm_left =0
        self.wrist_pwm_right =0
    def joy_callback(self, msg):
        # self.get_logger().info(f'Received joystick data:')
        # self.get_logger().info(f'Timestamp: {msg.header.stamp.sec}s {msg.header.stamp.nanosec}ns')
        # self.get_logger().info(f'Frame ID: {msg.header.frame_id}')
        # self.get_logger().info(f'Buttons: {msg.buttons}')
        # self.get_logger().info(f'Axes: {msg.axes}')

        # Process the joystick data and create an Int32MultiArray message
        # data_from_joystick = Float32MultiArray()
        # processed_data = Float32MultiArray()
        pwm_publish = Int32MultiArray()
        
        wrist_data = msg.axes[-2:]
        print(wrist_data)
        gripper_data = msg.axes[3]
        gripper_pwm = int(gripper_data*100)
        #calculating wrist pwm
        if(wrist_data[0]==0.0 and wrist_data[1]==1.0): #wrist_down
            self.wrist_pwm_left,self.wrist_pwm_right = [-100,-100]
        elif(wrist_data[0]==0.0 and wrist_data[1]==-1.0):#wrist_up
            self.wrist_pwm_left,self.wrist_pwm_right=[100,100]
        elif(wrist_data[0]==1.0 and wrist_data[1]==0.0):#rotate_left
            self.wrist_pwm_left,self.wrist_pwm_right=[100,-100]
        elif(wrist_data[0]==-1.0 and wrist_data[1]==0.0):#rotate_right
            self.wrist_pwm_left,self.wrist_pwm_right=[-100,100]
        elif(wrist_data[0]==0.0 and wrist_data[1]==0.0):#rotate_right
            self.wrist_pwm_left,self.wrist_pwm_right=[0,0]    
        print(self.wrist_pwm_left,self.wrist_pwm_right)
        #calculating gripper pwm

        pwm_publish.data=[0,0,self.wrist_pwm_left,self.wrist_pwm_right,gripper_pwm]
        print(self.wrist_pwm_left,self.wrist_pwm_right)
        # Publish the processed data
        print(pwm_publish.data)
        print(self.wrist_pwm_left,self.wrist_pwm_right)

        self.publisher.publish(pwm_publish)
        #self.get_logger().info(f'Published processed data: {processed_data.data}')

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
