import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, HistoryPolicy
import cv2
from cv_bridge import CvBridge
from std_msgs.msg import String
from sensor_msgs.msg import Image
import time
import os

class Turtlebot(Node):

    def __init__(self):
        super().__init__('turtlebot')
        
        timer_period = 1
        
        print("== Starting Turtlebot Node ==")
        self.flag = "Enable"
        qos_profile = QoSProfile(history=HistoryPolicy.KEEP_LAST, depth = 1)
        self.publisher = self.create_publisher(Image, 'video_frames', qos_profile)
        self.flag_subscriber = self.create_subscription(
            String,
            'working_flag',
            self.switch_flag,
            qos_profile)
        self.count = 0

        self.cam = cv2.VideoCapture('/dev/video0', cv2.CAP_V4L)
        if not self.cam.isOpened():
            raise Exception("Camera open failed!")
        self.br = CvBridge()
        self.latest_frame = None
        self.timer = self.create_timer(timer_period, self.publish_images)
        self.frame_timer = self.create_timer(0.5, self.get_latest_frame)

        print("== Waiting ==")
            
    def publish_images(self):
        if self.flag == "Disable":
            return
        
        if self.latest_frame is None:
            return
        self.get_logger().info('***********image published***********')
        
        self.publisher.publish(self.br.cv2_to_imgmsg(self.latest_frame, encoding="bgr8"))
    
    
    def get_latest_frame(self):
        self.get_logger().info("Acquire images")
        self.cam.set(cv2.CAP_PROP_POS_FRAMES, self.cam.get(cv2.CAP_PROP_FRAME_COUNT) - 1)
        ret, frame = self.cam.read()

        if not ret:
            self.get_logger().info("Image read failed!")
            print("Image read failed!")
            self.latest_frame = None
        else:
            self.latest_frame = frame
    
    def switch_flag(self, data):
        self.get_logger().info('Switch message: {0}'.format(data.data))
        self.flag = data.data

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