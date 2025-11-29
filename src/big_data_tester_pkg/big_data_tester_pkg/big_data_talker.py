import rclpy
from rclpy.node import Node
from my_first_package_msgs.msg import BigData
import numpy as np

class BigDataTalker(Node):
    def __init__(self):
        super().__init__('bigdata_talker')

        self.publisher_ = self.create_publisher(BigData, 'big_data', 10)

        # 전송할 배열 크기 설정: float64 × 100,000 ≈ 800KB 메시지
        self.array_size = 200000

        # 10Hz 로 발행
        self.timer = self.create_timer(0.1, self.timer_callback)

        self.get_logger().info(f"BigData Talker started, publishing size={self.array_size}")

    def timer_callback(self):
        msg = BigData()
        msg.data = np.random.rand(self.array_size).tolist()  # float64 값 배열

        self.publisher_.publish(msg)
        self.get_logger().info(f"Published array size={self.array_size}")

def main(args=None):
    rclpy.init(args=args)
    node = BigDataTalker()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()
