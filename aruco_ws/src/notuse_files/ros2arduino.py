import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32, String

class PublisherSubscriber(Node):

    def __init__(self):
        super().__init__('publisher_subscriber')
        self.publisher_ = self.create_publisher(Int32, 'micro_ros_arduino_subscriber', 10)
        self.subscription = self.create_subscription(
            String,
            'arduino_response',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning
        timer_period = 5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 1

    def timer_callback(self):
        msg = Int32()
        msg.data = self.i
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing: "%d"' % msg.data)

    def listener_callback(self, msg):
        self.get_logger().info('Received response: "%s"' % msg.data)

def main(args=None):
    rclpy.init(args=args)

    node = PublisherSubscriber()

    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
