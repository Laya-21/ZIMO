import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class SimplePublisher(Node):
    def __init__(self):
        super().__init__("simple_publisher")
        self.pub_ = self.create_publisher(String, "chatter", 10)
        self.counter_ = 0
        self.frequency_ = 1.0  # Frequency in Hz
        self.get_logger().info(f"Publishing at {self.frequency_} Hz")
        self.timer_ = self.create_timer(1.0 / self.frequency_, self.timerCallback)

    def timerCallback(self):
        msg = String()
        msg.data = "hello laya - counter %d" % self.counter_
        self.pub_.publish(msg)
        self.counter_ += 1


def main():
    rclpy.init()
    simple_publisher = SimplePublisher()
    rclpy.spin(simple_publisher)
    simple_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
