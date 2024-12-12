import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, HistoryPolicy, ReliabilityPolicy, DurabilityPolicy
from std_msgs.msg import String
import cv2
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
import time
import os

class Turtlebot(Node):

    def __init__(self):
        super().__init__('turtlebot')
        
        timer_period = 2.0
        
        print("== Starting Turtlebot Node ==")
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )
        # qos_profile = QoSProfile(history=HistoryPolicy.KEEP_LAST, depth = 1)
        self.publisher = self.create_publisher(Image, 'video_frames', qos_profile)
        self.count = 0

        self.cam = cv2.VideoCapture('/dev/video0', cv2.CAP_V4L)
        if not self.cam.isOpened():
            raise Exception("Camera open failed!")
        self.br = CvBridge()
        self.timer = self.create_timer(timer_period, self.publish_images)
        print("== Waiting ==")

    def publish_images(self):
        print("Reading images")
        ret, frame = self.cam.read()
        if not ret:
            self.get_logger().info("Image read failed!")
            print("Image read failed!")
        else:
            image_name = f"images/img{self.count}.jpg"
            print(f"writing file {image_name}")
            cv2.imwrite(image_name, frame)
            self.publisher.publish(self.br.cv2_to_imgmsg(frame, encoding="bgr8"))
            self.count += 1

def main(args=None):
    rclpy.init(args=args)
    node = Turtlebot()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Keyboard Interrupt (SIGINT)')
    finally:
        node.cam.release()
        node.destroy_node()
        rclpy.shutdown()
if __name__ == '__main__':
    main()