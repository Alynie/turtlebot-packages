import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, HistoryPolicy
from std_msgs.msg import String, Bool
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
import cv2
from cv_bridge import CvBridge

class Navigation(Node):
    
    def __init__(self):
        super().__init__('Navigation')
        qos_profile = QoSProfile(history=HistoryPolicy.KEEP_LAST,depth=1)
        self.image_subscriber = self.create_subscription(
            Image,
            'video_frames',
            self.image_callback,
            qos_profile)
        self.nav_publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.br = CvBridge()
        self.count = 0
        
    def save_image(self, data):
        image_name = f"images/img{self.count}.jpg"
        msg = String()
        self.count+=1 
        current_frame = self.br.imgmsg_to_cv2(data)
        cv2.imwrite(image_name, current_frame)
        print(f'== Saved {image_name} ==')
        
    def forward(self):
        vel_msg = Twist()
        
        vel_msg.linear.x = 0.5 
        vel_msg.angular.z = 0.0
        
        self.nav_publisher.publish(vel_msg)
        self.get_logger().info('Move Forward')
    
    def image_callback(self, data):
        self.save_image(data)
        # predict
        # map gesture to movement
        # publish movement
        self.forward()

def main(args=None):
    print('Starting Navigation Node')
    rclpy.init(args=args)
    node = Navigation()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Keyboard Interrupt (SIGINT)')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()