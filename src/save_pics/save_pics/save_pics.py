import rclpy
from std_msgs.msg import String, Bool
from sensor_msgs.msg import Image
import cv2
import numpy as np

def main(args=None):
    path = "images/"
    try:
        while True:
            count = 0
            cam = cv2.VideoCapture('/dev/video0', cv2.CAP_V4L)
            if not cam.isOpened():
                raise Exception("Camera open failed!")
            ret, frame = cam.read()
            if not ret:
                print("Image read failed!")
            else:
                name = f"{path}img{count}.jpg"
                print('== Press s to save or esc to exit ==')
                cv2.imshow('image', frame)
                k = cv2.waitKey(0)
                if k == 27:         # wait for ESC key to exit
                    cv2.destroyAllWindows()
                    print('== Exited without saving ==')
                elif k == ord('s'): # wait for 's' key to save and exit
                    cv2.imwrite(name, frame)
                    cv2.destroyAllWindows()
                    count+=1
                    print(f"== {name} saved ==")
    except KeyboardInterrupt:
        print('Keyboard Interrupt (SIGINT)')
    finally:
        cv2.destroyAllWindows()

if __name__ == '__main__':
    main()
