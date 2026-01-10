import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy

class JoySubscriber(Node):
    def __init__(self):
        super().__init__('joy_subscriber')
        self.subscription = self.create_subscription(
            Joy,
            'joy',  # Replace with the topic name you are subscribing to
            self.joy_callback,
            10  # QoS depth
        )
        self.subscription  # Prevent unused variable warning

    def joy_callback(self, msg):
        self.get_logger().info(f'Received joystick data:')
        self.get_logger().info(f'Timestamp: {msg.header.stamp.sec}s {msg.header.stamp.nanosec}ns')
        self.get_logger().info(f'Frame ID: {msg.header.frame_id}')
        self.get_logger().info(f'Axes: {msg.axes}')
        self.get_logger().info(f'Buttons: {msg.buttons}')

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
