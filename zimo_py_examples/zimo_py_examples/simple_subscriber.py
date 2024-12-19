import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class SimpleSubscriber(Node):
    def __init__(self):
        super().__init__("simple_subscriber")  # Fixed typo in node name
        self.create_subscription(
            String,              # Message type
            "chatter",           # Topic name
            self.msg_callback,   # Callback function
            10                   # QoS profile (queue size)
        )
        self.get_logger().info("SimpleSubscriber node has started.")

    def msg_callback(self, msg):  # Added msg as an argument
        self.get_logger().info(f"I heard: {msg.data}")  # Corrected string formatting

def main():
    rclpy.init()  # Initialize the rclpy library
    simple_subscriber = SimpleSubscriber()  # Create an instance of the node
    rclpy.spin(simple_subscriber)  # Keep the node running, waiting for messages
    simple_subscriber.destroy_node()  # Clean up and destroy the node
    rclpy.shutdown()  # Shutdown the rclpy library

if __name__ == '__main__':  # Corrected the condition
    main()
