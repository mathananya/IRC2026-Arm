import serial
import time
import rclpy
from rclpy.node import Node
from gps_msgs.msg import GPSFix
from std_msgs.msg import String

import csv
import os
from datetime import datetime

# Configuration
PORT1 = "/dev/ttyACM0"
PORT2 = "/dev/ttyACM1"
BAUD_RATE = 115200
ROS_TOPIC_GPS = "/gps/fix"
CSV_FILE = "pne_data.csv"


class SerialReaderNode(Node):
    def __init__(self):
        super().__init__('serial_reader_node')
        self.gps_pub = self.create_publisher(
            GPSFix, ROS_TOPIC_GPS, 10)

        self.keySub = self.create_subscription(
            String,
            'keyboard',
            self.key_callback,
            10)
        self.keySub  # prevent unused variable warning

        self.inside_pne = False
        self.inside_gps = False

        self.ser = None
        self.connect_serial()

        self.init_csv()

        self.create_timer(0.1, self.read_serial)

    def key_callback(self, msg):
        self.get_logger().info(f"Received key input: {msg.data.encode('utf-8')}")
        self.ser.write(msg.data.encode('utf-8'))

    def init_csv(self):
        if not os.path.exists(CSV_FILE):
            with open(CSV_FILE, mode="w", newline="") as file:
                writer = csv.writer(file)
                writer.writerow(["PNE_data"])
            self.get_logger().info("CSV file created.")
        else:
            self.get_logger().info("CSV file already exists. Appending data.")

    def write_to_csv(self, line):
        with open(CSV_FILE, mode="a", newline="") as file:
            writer = csv.writer(file)
            writer.writerow([line])

    def connect_serial(self):
        try:
            self.ser = serial.Serial(PORT1, BAUD_RATE)
            time.sleep(2)
            self.get_logger().info(f"Connected to Arduino on {PORT1}")
        except Exception as e:
            self.get_logger().error(
                f"Error: Could not connect to Arduino - {e}")
            self.get_logger().info(f"Trying port - {PORT2}")

        try:
            self.ser = serial.Serial(PORT2, BAUD_RATE)
            time.sleep(2)
            self.get_logger().info(f"Connected to Arduino on {PORT2}")
        except Exception as e:
            self.get_logger().error(
                f"Error: Could not connect to Arduino - {e}")
            self.get_logger().info("Failed to connect to both ports. Exiting.")
            exit(1)

    def read_serial(self):
        try:
            if self.ser.in_waiting:
                line = self.ser.readline().decode("utf-8").strip()
                if not line:
                    return

                if line == "PNE_START":
                    self.inside_pne = True
                    self.get_logger().info("PNE data start detected.")
                    timestamp = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
                    self.write_to_csv(
                        f"--- PNE Data Start: {timestamp} ---")

                elif line == "PNE_END":
                    self.inside_pne = False
                    self.get_logger().info("PNE data end detected.")
                    self.write_to_csv("-----------------------------------")

                elif self.inside_pne:
                    self.write_to_csv(line)

                elif line == "GPS_START":
                    self.inside_gps = True
                    self.get_logger().info("GPS data detected.")

                elif line == "GPS_END":
                    self.inside_gps = False
                    self.get_logger().info("GPS data end detected.")

                elif self.inside_gps:
                    gps_msg = self.gps_msg_generator(line)
                    self.gps_pub.publish(gps_msg)
                    self.inside_gps = False

        except Exception as e:
            self.get_logger().error(f"Error reading serial: {e}")

    def gps_msg_generator(self, line):
        gps_msg = GPSFix()
        if ("," in line):
            gps_msg.track = float(line.split(',')[0])
            gps_msg.latitude = float(line.split(',')[1])
            gps_msg.longitude = float(line.split(',')[2])
        return gps_msg

    def destroy_node(self):
        self.ser.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = SerialReaderNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
