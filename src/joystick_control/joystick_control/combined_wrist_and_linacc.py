import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from std_msgs.msg import Int32MultiArray
from std_msgs.msg import Float32MultiArray,Float64
import termios
import tty
import sys
import select

def get_key(): 
    """Read a single character from the keyboard."""
    settings = termios.tcgetattr(sys.stdin)
    try:
        tty.setraw(sys.stdin)
        if select.select([sys.stdin], [], [], 0.1)[0]:
            key = sys.stdin.read(1)
        else:
            key = None
    finally:
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key

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
        self.publisher_wrist = self.create_publisher(
            Int32MultiArray,
            'arm_pwm_commands',  # Replace with the topic name you want to publish to
            10  # QoS depth
        )

        self.subscription  # Prevent unused variable warning
        self.current_states=[0,0]
        
        self.move_bisep = False
        self.move_tricep = False
        self.last_pressed =0 #0 for none ; 1 for bisep; 2 for tricep

        self.bisep_state_to_move=0
        self.tricep_state_to_move=0

        self.publish_flag = False
        self.subscription  # Prevent unused variable warning

        self.wrist_pwm_left =0
        self.wrist_pwm_right =0
        self.last_pressed =0 #0 for none ; 1 for bisep; 2 for tricep

        self.previous_key = None
        self.data_array = [0, 0, 0, 0, 0]
        self.timer1 = self.create_timer(0.01, self.key_callback)
    

    def current_state_callback(self, msg):
        # Extract lower and upper actuator states from the encoder data
        self.current_states = msg.data[:2]
    def key_callback(self):
        VAL = 50
        key = get_key()
        if key is not None:
                self.data_array=[0,0,0,0,0]
                if key == 'w' and self.previous_key != 'w':
                    self.data_array[0] += VAL  # Increase first value
                elif key == 's' and self.previous_key != 's':
                    self.data_array[0] -= VAL  # Decrease first value
                elif key == 'e' and self.previous_key != 'e':
                    self.data_array[1] += VAL  # Increase second value
                elif key == 'd' and self.previous_key != 'd':
                    self.data_array[1] -= VAL #0ecrease second value
                elif key == 'r' and self.previous_key != 'r':
                    self.data_array[2] += VAL# Increase third value
                elif key == 'f' and self.previous_key != 'f':
                    self.data_array[2] -= VAL  # Decrease third value
                elif key == 't' and self.previous_key != 't':
                    self.data_array[3] += VAL  # Increase fourth value
                elif key == 'g' and self.previous_key != 'g':
                    self.data_array[3] -= VAL  # Decrease fourth value
                elif key == 'y' and self.previous_key != 'y':
                    self.data_array[4] += VAL  # Increase fifth value
                elif key == 'h' and self.previous_key != 'h':
                    self.data_array[4] -= VAL  # Decrease fifth value
                elif key == 'q':  # Quit the loop
                    raise KeyboardInterrupt
                    self.get_logger().info("Quitting...") 
                msg = Int32MultiArray()
                msg.data = self.data_array

                # Publish the message to the topic
                self.publisher_wrist.publish(msg)
                self.get_logger().info(f'Published array: {msg.data}')
                self.data_array=[0,0,0,0,0]
    def joy_callback(self, msg):
        self.move_x = False
        self.move_y = False
        x_flag , y_flag = [msg.buttons[5],msg.buttons[3]]
    
        # bisep_state_to_move = self.current_states[0]
        # tricep_state_to_move = self.current_states[1]
        if(x_flag == 1 and y_flag==0 and self.last_pressed!=1):
            self.move_x = True
            self.move_y = False
            self.last_pressed =1
            if(msg.axes[1]!=0):
                pwm_analog_value=msg.axes[1]/abs(msg.axes[1])
            else:
                pwm_analog_value=0.0

            joystick_target = Int32MultiArray()

            bisep_current_state = self.current_states[0]
            tricep_current_state = self.current_states[1]
            self.tricep_state_to_move = int(tricep_current_state - (pwm_analog_value*16*5.9))
            self.bisep_state_to_move = int(bisep_current_state - (pwm_analog_value*16*2.5))
            print("moving X", (pwm_analog_value*20),bisep_current_state,self.bisep_state_to_move)
            joystick_target.data=[self.bisep_state_to_move,self.tricep_state_to_move,0,0,0]
            self.publisher.publish(joystick_target)
        if(x_flag == 0 and y_flag==1 and self.last_pressed!=2):
            self.move_bisep = False
            self.move_tricep = True
            self.last_pressed =2
            if(msg.axes[1]!=0):
                pwm_analog_value=msg.axes[1]/abs(msg.axes[1])
            else:
                pwm_analog_value=0.0
            joystick_target = Int32MultiArray()

            bisep_current_state = self.current_states[0]
            tricep_current_state = self.current_states[1]
            self.tricep_state_to_move = int(tricep_current_state + (pwm_analog_value*16*5.9))
            self.bisep_state_to_move = int(bisep_current_state + (pwm_analog_value*16*2.5))
            print("moving Y", (pwm_analog_value*20),tricep_current_state,self.tricep_state_to_move)
            joystick_target.data=[self.bisep_state_to_move,self.tricep_state_to_move,0,0,0]
            self.publisher.publish(joystick_target)
            
        if(x_flag == 1 and  y_flag==1):
                self.move_bisep = False
                self.move_tricep = False

        if(x_flag==0 and y_flag ==0):
            self.move_bisep = False
            self.move_tricep = False
            self.last_pressed =0

        # VAL = 100
        # key = get_key()
        # if key is not None:
        #         if key == 'w' and self.previous_key != 'w':
        #             self.data_array[0] += VAL  # Increase first value
        #         elif key == 's' and self.previous_key != 's':
        #             self.data_array[0] -= VAL  # Decrease first value
        #         elif key == 'e' and self.previous_key != 'e':
        #             self.data_array[1] += VAL  # Increase second value
        #         elif key == 'd' and self.previous_key != 'd':
        #             self.data_array[1] -= VAL #0ecrease second value
        #         elif key == 'r' and self.previous_key != 'r':
        #             self.data_array[2] += VAL# Increase third value
        #         elif key == 'f' and self.previous_key != 'f':
        #             self.data_array[2] -= VAL  # Decrease third value
        #         elif key == 't' and self.previous_key != 't':
        #             self.data_array[3] += VAL  # Increase fourth value
        #         elif key == 'g' and self.previous_key != 'g':
        #             self.data_array[3] -= VAL  # Decrease fourth value
        #         elif key == 'y' and self.previous_key != 'y':
        #             self.data_array[4] += VAL  # Increase fifth value
        #         elif key == 'h' and self.previous_key != 'h':
        #             self.data_array[4] -= VAL  # Decrease fifth value
        #         elif key == 'q':  # Quit the loop
        #             self.get_logger().info("Quitting...")
        #         msg = Int32MultiArray()
        #         msg.data = self.data_array

        #         # Publish the message to the topic
        #         self.publisher_wrist.publish(msg)
        #         self.get_logger().info(f'Published array: {msg.data}')
        # else:
        #     self.data_array = [0, 0, 0, 0, 0]
        
        #     # Reset values when the key is released
        # if not key and self.previous_key is not None:
        #     self.data_array = [0, 0, 0, 0, 0]
        #     self.publish_array()
        #     self.previous_key = None

        #     self.previous_key = key

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
        joystick_target_home = Int32MultiArray()
        if(self.publish_flag):
            joystick_target_home.data=target_state
            print(joystick_target_home)
            self.publish_flag=False
            self.publisher.publish(joystick_target_home)

        # analog_value =abs(int(msg.axes[3]*80))
        # print(analog_value)
        # self.get_logger().info(f'Received joystick data:')
        # self.get_logger().info(f'Timestamp: {msg.header.stamp.sec}s {msg.header.stamp.nanosec}ns')
        # self.get_logger().info(f'Frame ID: {msg.header.frame_id}')
        # self.get_logger().info(f'Axes: {msg.axes}')
        # self.get_logger().info(f'Buttons: {msg.buttons}')

        # Process the joystick data and create an Int32MultiArray message
        # data_from_joystick = Float32MultiArray()
        # processed_data = Float32MultiArray()
        # pwm_publish = Int32MultiArray()
        bisep_flag , tricep_flag = [msg.buttons[2],msg.buttons[4]]
    
        # bisep_state_to_move = self.current_states[0]
        # tricep_state_to_move = self.current_states[1]
        if(bisep_flag == 1 and tricep_flag==0 and self.last_pressed!=1):
            self.move_bisep = True
            self.move_tricep = False
            self.last_pressed =1
            if(msg.axes[1]!=0):
                pwm_analog_value=msg.axes[1]/abs(msg.axes[1])
            else:
                pwm_analog_value=0.0

            joystick_target = Int32MultiArray()

            bisep_current_state = self.current_states[0]
            self.tricep_state_to_move = self.current_states[1]
            self.bisep_state_to_move = int(bisep_current_state + (pwm_analog_value*40))
            print("moving bisep", (pwm_analog_value*20),bisep_current_state,self.bisep_state_to_move)
            joystick_target.data=[self.bisep_state_to_move,self.tricep_state_to_move,0,0,0]
            self.publisher.publish(joystick_target)
        if(bisep_flag == 0 and tricep_flag==1 and self.last_pressed!=2):
            self.move_bisep = False
            self.move_tricep = True
            self.last_pressed =2
            if(msg.axes[1]!=0):
                pwm_analog_value=msg.axes[1]/abs(msg.axes[1])
            else:
                pwm_analog_value=0.0
            joystick_target = Int32MultiArray()

            tricep_current_state = self.current_states[1]
            self.bisep_state_to_move = self.current_states[0]
            self.tricep_state_to_move = int(tricep_current_state + (pwm_analog_value*40))
            print("moving tricep", (pwm_analog_value*20),tricep_current_state,self.tricep_state_to_move)
            joystick_target.data=[self.bisep_state_to_move,self.tricep_state_to_move,0,0,0]
            self.publisher.publish(joystick_target)
            
        if(bisep_flag == 1 and  tricep_flag==1):
                self.move_bisep = False
                self.move_tricep = False

        if(tricep_flag==0 and bisep_flag ==0):
            self.move_bisep = False
            self.move_tricep = False
            self.last_pressed =0
        #print("flag",self.move_bisep,self.move_tricep)
        # pwm_analog_value = msg.axes[1]
        # if(msg.axes[1]!=0):
        #     pwm_analog_value=msg.axes[1]/abs(msg.axes[1])
        # else:
        #     pwm_analog_value=0.0
        #print(pwm_analog_value)
        # print("flag",self.move_bisep,self.move_tricep,pwm_analog_value)
        
        # joystick_target = Int32MultiArray()

        # if(self.move_bisep==1):
        #     bisep_current_state = self.current_states[0]
        #     tricep_current_state = self.current_states[1]
        #     self.bisep_state_to_move = int(bisep_current_state + (pwm_analog_value*80))
        #     print("moving bisep", (pwm_analog_value*80))
        #     joystick_target.data=[self.bisep_state_to_move,self.tricep_state_to_move,0,0,0]
        #     self.publisher.publish(joystick_target)

        # elif(self.move_tricep==1):
        #     tricep_current_state = self.current_states[1]
        #     bisep_current_state = self.current_states[0]
        #     self.tricep_state_to_move = int(tricep_current_state + (pwm_analog_value*80))
        #     print("moving tricep", (pwm_analog_value*80))
        #     joystick_target.data=[self.bisep_state_to_move,self.tricep_state_to_move,0,0,0]
        #     self.publisher.publish(joystick_target)

        # else:
        #     self.tricep_state_to_move=0
        #     self.bisep_state_to_move=0
        # if(self.move_bisep or self.move_tricep):
        #     print(joystick_target)
        # elif(self.move_bisep == False and self.move_tricep == False):
        #     joystick_target.data=[self.current_states[0],self.current_states[1],0,0,0]
        #     print(joystick_target)
        #     self.publisher.publish(joystick_target)
        if(self.move_bisep==False and self.move_tricep==False and msg.buttons[6]==1):
            pwm_publish_wrist = Int32MultiArray()

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

            pwm_publish_wrist.data=[0,0,self.wrist_pwm_left,self.wrist_pwm_right,gripper_pwm]
            # Publish the processed data
            print(pwm_publish_wrist.data)

            self.publisher_wrist.publish(pwm_publish_wrist)

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
