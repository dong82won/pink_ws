import rclpy
from rclpy.node import Node
from my_first_package_msgs.msg import BigData
import time

class BigDataListener(Node):
    def __init__(self):
        super().__init__('bigdata_listener')

        self.subscription = self.create_subscription(
            BigData,
            'big_data',
            self.listener_callback,
            10
        )
        self.subscription  # prevent unused variable warning

        self.last_time = time.time()

        self.get_logger().info("BigData Listener started")

    def listener_callback(self, msg):
        now = time.time()
        dt = now - self.last_time
        self.last_time = now

        self.get_logger().info(
            f"Received array size={len(msg.data)} | interval={dt:.3f}s"
        )

def main(args=None):
    rclpy.init(args=args)
    node = BigDataListener()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()
