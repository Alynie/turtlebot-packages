import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from std_msgs.msg import String, Bool
from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge

class ImageSubscriber(Node):
    
    def __init__(self):
        super().__init__('Image_subscriber')
        qos_profile = QoSProfile(depth=1)
        self.image_subscriber = self.create_subscription(
            Image,
            'video_frames',
            self.listener_callback,
            qos_profile)
        self.br = CvBridge()
        self.count = 0
    
    def listener_callback(self, data):
        msg = String()
        self.count+=1 
        current_frame = self.br.imgmsg_to_cv2(data)
        cv2.imwrite("images/img" + str(self.count)+".jpg", current_frame)
        print('== Finish Detected ==')

def main(args=None):
    print('Starting Subscriber Node')
    rclpy.init(args=args)
    node = ImageSubscriber()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Keyboard Interrupt (SIGINT)')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()