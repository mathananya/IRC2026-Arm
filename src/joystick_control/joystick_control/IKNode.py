import rclpy
from rclpy.node import Node
from poseToPWM import pose_to_encoder
from std_msgs.msg import Int32MultiArray


class IKNode(Node):

    def __init__(self):
        super().__init__('IK_Node')
        self.publisher_ = self.create_publisher(Int32MultiArray, 'arm_target_states', 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):
        if(rclpy.ok()):
            print("Enter values of x and z where you want to go, comma seperated")
            formatStr = input().strip()
            x, z = formatStr.split(",")
            x = int(x)
            z = int(z)

            lower_encoder, upper_encoder = pose_to_encoder(x, z)
            pubMsg = Int32MultiArray()
            pubMsg.data = [int(lower_encoder), int(upper_encoder), 0, 0, 0]
            self.publisher_.publish(pubMsg)
            self.get_logger().info(f"Published IK Message : {pubMsg.data}")



def main(args=None):
    rclpy.init(args=args)

    ik_node = IKNode()

    rclpy.spin(ik_node)

    ik_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()