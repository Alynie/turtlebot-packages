import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, HistoryPolicy
from std_msgs.msg import String
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
import cv2
from cv_bridge import CvBridge
from final.detect_gesture import Gesture
import time
import numpy as np

class Navigation(Node):
    
    def __init__(self):
        super().__init__('Navigation')
        qos_profile = QoSProfile(history=HistoryPolicy.KEEP_LAST,depth=1)
        self.sleep_time = 2
        
        self.image_subscriber = self.create_subscription(
            Image,
            'video_frames',
            self.image_callback,
            qos_profile)
        self.camera_flag_publisher = self.create_publisher(String, 'working_flag', qos_profile)
        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.br = CvBridge()
        self.count = 0
        self.vel_msg = Twist()
        self.gesture = Gesture()
        self.result = None
        
    def save_image(self, data):
        image_name = f"images/img{self.count}.jpg"
        msg = String()
        self.count+=1 
        current_frame = self.br.imgmsg_to_cv2(data)
        height, width = current_frame.shape[:2]
        if width != 640 and height != 480:
            current_frame = cv2.resize(current_frame, (640, 480), interpolation=cv2.INTER_AREA)
            print("Resized image")

        gray = cv2.cvtColor(current_frame, cv2.COLOR_BGR2GRAY)
        img_gray = np.zeros_like(current_frame)
        img_gray[:,:,0] = gray
        img_gray[:,:,1] = gray
        img_gray[:,:,2] = gray
        print(img_gray.shape)
        cv2.imwrite(image_name, img_gray) #comment if you don't want to save file
        print(f'== Saved {image_name} ==')
        self.result = self.gesture.detect_gesture(img_gray,self.count)
        
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
        msg = String()
        msg.data = "Disable"
        self.camera_flag_publisher.publish(msg)
        # predict
        self.save_image(data)
        # map gesture to movement
        if self.result[0] == "ok":
            self.forward()
        elif self.result[0] == "fist":
            self.backward()
        elif self.result[0] == "tnf":
            self.backward()
        elif self.result[0] == "one_finger_left":
            self.left()
        elif self.result[0] == "one_finger_right":
            self.right()
        elif self.result[0] == "palm":
            self.stop()
        elif self.result[0] == "no_gesture":
            self.stop()
        else: 
            self.stop()
        
        print("Moving robot...")
        self.move()
        time.sleep(self.sleep_time)
        msg.data = "Enable"
        self.camera_flag_publisher.publish(msg)
        self.reset()

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