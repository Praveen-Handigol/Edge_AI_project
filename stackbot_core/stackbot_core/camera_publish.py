import cv2
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import String
from cv_bridge import CvBridge

"""
Publishes camera images to "camera_iamge/compressed" topic using "CompressedImage" message type
"""

class CameraPublisher(Node):
    def __init__(self):
        super().__init__('camera_publisher')
        self.publisher_ = self.create_publisher(CompressedImage, 'camera_image/compressed', 10)
        self.publish_count = self.create_publisher(String, 'camera_count', 10)
        self.timer = self.create_timer(0.1, self.publish_image)
        self.bridge = CvBridge()
        self.cap = cv2.VideoCapture(0)
        self.i = 0

    def publish_image(self):
        ret, frame = self.cap.read()
        if ret:
            img_msg = self.bridge.cv2_to_compressed_imgmsg(frame)
            self.publisher_.publish(img_msg)
            self.msg = String()
            self.msg.data = str(self.i)
            self.publish_count.publish(self.msg)
            self.i +=1
            print("publishing images")
        
def main(args=None):
    rclpy.init(args=args)
    camera_publisher = CameraPublisher()
    try:
        rclpy.spin(camera_publisher)
    except KeyboardInterrupt:
        camera_publisher.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()