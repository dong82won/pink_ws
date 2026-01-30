import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np

# 파라미터 관련 임포트
from rcl_interfaces.msg import ParameterDescriptor, IntegerRange
from rcl_interfaces.msg import SetParametersResult

# QoS 관련 임포트
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

# [핵심 1] 멀티쓰레딩을 위한 모듈 임포트
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor

class CartoonStyler(Node):
    def __init__(self):
        super().__init__('cartoon_styler')

        self.get_logger().info("Start Cartoon Styler (Multi-Threaded).")

        # [핵심 2] 재진입 가능한 콜백 그룹 생성
        # 이 그룹에 속한 콜백은 앞선 처리가 안 끝나도 병렬로 실행될 수 있습니다.
        self.callback_group = ReentrantCallbackGroup()

        # 1. QoS 설정 (Best Effort)
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        # 2. Subscriber 생성
        # [핵심 3] callback_group을 지정하여 병렬 실행 허용
        self.subscription = self.create_subscription(
            Image,
            '/image_raw',
            self.image_callback,
            qos_profile,
            callback_group=self.callback_group 
        )

        # 3. Publisher 생성
        self.publisher = self.create_publisher(
            Image,
            '/image_cartoon',
            qos_profile
        )
        
        # 4. 파라미터 설정 (sigma_color, sigma_space)
        param_desc_sigma_color = ParameterDescriptor(
            description='Bilateral filter sigma_color',
            integer_range=[IntegerRange(from_value=0, to_value=100, step=1)],
        )
        param_desc_sigma_space = ParameterDescriptor(
            description='Bilateral filter sigma_space',
            integer_range=[IntegerRange(from_value=0, to_value=25, step=1)],
        )

        self.declare_parameter('sigma_color', 20, param_desc_sigma_color)
        self.declare_parameter('sigma_space', 5, param_desc_sigma_space)

        # 파라미터 변경 콜백 등록
        self.add_on_set_parameters_callback(self.parameter_callback)

        # Initialize CV bridge
        self.cv_bridge = CvBridge()

    def parameter_callback(self, params):
        for param in params:
            self.get_logger().info(f"{param.name} is changed to {param.value}")
        return SetParametersResult(successful=True)

    def image_callback(self, msg):
        """
        이미지가 들어올 때마다 별도의 쓰레드에서 실행됩니다.
        """
        try:
            # 파라미터 읽기
            sigma_color = self.get_parameter('sigma_color').value
            sigma_space = self.get_parameter('sigma_space').value

            # ROS Image -> OpenCV
            cv_image = self.cv_bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

            # ---------------------------------------------------------
            # [성능 팁] bilateralFilter가 너무 느리다면 아래 Resize 주석을 해제하세요.
            # 속도가 3~4배 빨라집니다.
            # ---------------------------------------------------------
            # small_image = cv2.resize(cv_image, None, fx=0.5, fy=0.5)
            # blr = cv2.bilateralFilter(small_image, -1, sigma_color, sigma_space)
            # edge = 255 - cv2.Canny(small_image, 80, 120)
            # edge = cv2.cvtColor(edge, cv2.COLOR_GRAY2BGR)
            # cartoon_small = cv2.bitwise_and(blr, edge)
            # cartoon_image = cv2.resize(cartoon_small, (cv_image.shape[1], cv_image.shape[0]))
            
            # (현재 원본 크기 사용)
            blr = cv2.bilateralFilter(cv_image, -1, sigma_color, sigma_space)
            edge = 255 - cv2.Canny(cv_image, 80, 120)
            edge = cv2.cvtColor(edge, cv2.COLOR_GRAY2BGR)
            cartoon_image = cv2.bitwise_and(blr, edge)

            # OpenCV -> ROS Image
            cartoon_msg = self.cv_bridge.cv2_to_imgmsg(cartoon_image, encoding='bgr8')
            
            # Publish
            self.publisher.publish(cartoon_msg)

        except Exception as e:
            self.get_logger().error(f"Error in processing: {e}")


def main(args=None):
    rclpy.init(args=args)
    
    node = CartoonStyler()
    
    # [핵심 4] MultiThreadedExecutor 사용
    # num_threads 인자를 비워두면 CPU 코어 수만큼 자동으로 쓰레드를 생성합니다.
    executor = MultiThreadedExecutor() 
    executor.add_node(node)

    try:
        # node.spin() 대신 executor.spin()을 사용해야 합니다.
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()