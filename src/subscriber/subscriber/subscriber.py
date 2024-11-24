import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class MinimalSubscriber(Node):
    def __init__(self):
        super().__init__('minimal_subscriber')
        print("init")
        self.subscription = self.create_subscription(String, 'topic', self.listener_callback, 10)
        self.subscription #prevent unused variable warning
        print("init end")

    def listener_callback(self, msg):
        print("callback")
        self.get_logger().info('I heard: "%s"' % msg.data)
        print('I heard: "%s"' % msg.data)

def main(args=None):
    rclpy.init(args=args)

    minimal_subscriber = MinimalSubscriber()
    rclpy.spin(minimal_subscriber)

    #Destroy the node explicitly so that garbage collector doesn't need to do it
    minimal_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()