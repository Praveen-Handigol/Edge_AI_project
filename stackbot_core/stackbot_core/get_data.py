import rclpy
from rclpy.node import Node
import os
from geometry_msgs.msg import Twist
import cv2
import csv

"""
Saves image and csv vlaue at every 0.25 seconds
"""

class TwistSubscriber(Node):

    def __init__(self):
        super().__init__('twist_subscriber')
        self.subscription = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.twist_callback,
            10  # QoS profile depth
        )

        self.cap = cv2.VideoCapture(0)
        self.i = 1
        self.linear_x = 0
        self.angular_z = 0

        self.image_path = "/home/STACKBOT/stackbot_ws/April29_images_1"
        if not os.path.exists(self.image_path):
            os.makedirs(self.image_path)

        self.csv_path = "/home/STACKBOT/stackbot_ws/output.csv"
        if not os.path.exists(self.csv_path):
            with open(self.csv_path, 'w', newline='') as csvfile:
                # Write header or initial content if needed
                csv_writer = csv.writer(csvfile)
                # Write header
                csv_writer.writerow(['image_path', 'linear_x', 'angular_z'])
        self.timer = self.create_timer(0.25, self.capture_image_callback)
        print("done initializing get_data")

    def capture_image_callback(self):
        if self.linear_x != 0 or self.angular_z != 0:
            ret, frame = self.cap.read()
            file_name = "/image_"+str(self.i)+".jpg"
            cv2.imwrite(self.image_path + file_name, frame)
            self.i += 1
            with open(self.csv_path, 'a', newline='') as csvfile:
                csv_writer = csv.writer(csvfile)
                csv_writer.writerow(["/content/April29_images_1" + file_name, self.linear_x, self.angular_z])

    def twist_callback(self, msg):
        self.linear_x = msg.linear.x
        self.angular_z = msg.angular.z               
            



def main(args=None):
    rclpy.init(args=args)
    twist_subscriber = TwistSubscriber()
    rclpy.spin(twist_subscriber)
    twist_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
