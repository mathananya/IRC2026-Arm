import rclpy
from rclpy.node import Node
import cv2
import time
from std_msgs.msg import Int32MultiArray

CAMERA_INDEX = 0
SAVE_DIR = "~/pano"

class PanoWriter(Node):

    def __init__(self):
        super().__init__('pano_writer')

        self.cap = cv2.VideoCapture(CAMERA_INDEX)
        if not self.cap.isOpened():
            self.get_logger().error("Camera not opened")

        self.images = []
        self.last_angle = None

        self.subscriber = self.create_subscription(
            Int32MultiArray,
            '/ServoMotorCommands',
            self.listener_callback,
            10
        )

    def listener_callback(self, msg):
        angle = msg.data[1]

        if angle % 30 == 0 and angle != self.last_angle:
            self.capture(angle)

        if angle >= 170:
            self.stitch()

        self.last_angle = angle

    def capture(self, angle):
        ret, frame = self.cap.read()
        if not ret:
            self.get_logger().error("Failed to capture frame")
            return

        filename = f"{SAVE_DIR}/img_{angle}_{int(time.time())}.jpg"
        cv2.imwrite(filename, frame)
        self.images.append(frame)

        self.get_logger().info(f"Captured image at {angle}°")

    def stitch(self):
        if len(self.images) < 2:
            self.get_logger().warn("Not enough images to stitch")
            return

        self.get_logger().info("Stitching images...")

        stitcher = cv2.Stitcher_create(cv2.Stitcher_PANORAMA)
        status, pano = stitcher.stitch(self.images)

        if status == cv2.Stitcher_OK:
            cv2.imwrite("panorama.jpg", pano)
            self.get_logger().info("Panorama saved as panorama.jpg")
        else:
            self.get_logger().error(f"Stitching failed with code {status}")

        self.images.clear()

