import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from std_msgs.msg import String
import cv2
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
import time
import os

class ImagePublisher(Node):

    def __init__(self):
        super().__init__('image_publisher')
        print("init")
        qos_profile = QoSProfile(depth = 1)
        self.image_publisher = self.create_publisher(Image, 'video_frames', qos_profile)
        self.count = 0

        self.cam = cv2.VideoCapture('/dev/video0', cv2.CAP_V4L)
        if not self.cam.isOpened():
            raise Exception("Camera open failed!")
        self.br = CvBridge()
        # self.publish_timer = self.create_timer(30, self.publish_images)
        while True:
            self.publish_images
        print("init end")

    def publish_images(self):
        print("inside publish_images")
        ret, frame = self.cam.read()
        if not ret:
            self.get_logger().info("Image read failed!")
            print("Image read failed!")
        else:
            name = "img" + str(self.count)+".jpg"
            print('== Press s to save or esc to exit ==')
            cv2.imshow('image',frame)
            k = cv2.waitKey(0)
            if k == 27:         # wait for ESC key to exit
                cv2.destroyAllWindows()
                print('== Exited without saving ==')
            elif k == ord('s'): # wait for 's' key to save and exit
                print("writing file")
                cv2.imwrite("images/img"+str(self.count)+".jpg", frame)
                self.image_publisher.publish(self.br.cv2_to_imgmsg(frame, encoding="bgr8"))
                self.count += 1
                cv2.destroyAllWindows()
                print(f"== {name} saved ==")

def main(args=None):
    print('== Publisher Started ==')
    rclpy.init(args=args)
    node = ImagePublisher()
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
