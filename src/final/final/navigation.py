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
        reset_timer_frequency = 2.0 #Hz
        
        self.image_subscriber = self.create_subscription(
            Image,
            'video_frames',
            self.image_callback,
            qos_profile)
        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.br = CvBridge()
        self.count = 0
        self.timer = self.create_timer(reset_timer_frequency, self.reset)
        self.vel_msg = Twist()
        self.gesture = "stop"
        
    def save_image(self, data):
        image_name = f"images/img{self.count}.jpg"
        msg = String()
        self.count+=1 
        current_frame = self.br.imgmsg_to_cv2(data)
        cv2.imwrite(image_name, current_frame)
        print(f'== Saved {image_name} ==')
        
    def forward(self):
        self.vel_msg.linear.x = 0.1 
        self.vel_msg.angular.z = 0.0
        self.get_logger().info('Move Forward')
    
    def backward(self):
        self.vel_msg.linear.x = -0.1 
        self.vel_msg.angular.z = 0.0
        self.get_logger().info('Move Backward')
        
    def left(self):
        self.vel_msg.linear.x = 0.0
        self.vel_msg.angular.z = 0.1
        self.get_logger().info('Turn Left')
        
    def right(self):
        self.vel_msg.linear.x = 0.0 
        self.vel_msg.angular.z = -0.1
        self.get_logger().info('Turn Right')
    
    def stop(self):
        self.vel_msg.linear.x = 0.0 
        self.vel_msg.angular.z = 0.0
        self.get_logger().info('Stop')
    
    def move(self):
        self.publisher.publish(self.vel_msg)
    
    def reset(self):
        self.stop()
        self.move()
    
    def image_callback(self, data):
        self.save_image(data)
        # predict
        # map gesture to movement
        if self.gesture == "forward":
            self.forward()
        elif self.gesture == "backward":
            self.backward()
        elif self.gesture == "left":
            self.left()
        elif self.gesture == "right":
            self.right()
        elif self.gesture == "stop":
            self.stop()
        else: 
            self.stop()
        # publish movement
        self.move()

def main(args=None):
    print('Starting Navigation Node')
    rclpy.init(args=args)
    node = Navigation()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Keyboard Interrupt (SIGINT)')
    finally:
        node.reset()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()