import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import subprocess  # [추가] 터미널 명령어 실행을 위한 모듈
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

class ImgPublisher(Node):
  def __init__(self):
    super().__init__('img_pub_node')
    
    # QoS 설정: Reliable + Depth 1
    qos_profile = QoSProfile(
        reliability=ReliabilityPolicy.BEST_EFFORT,
        history=HistoryPolicy.KEEP_LAST,
        depth=1
    )
    self.publisher_ = self.create_publisher(Image, 'image_raw', qos_profile)

    self.declare_parameter('width', 640)
    self.declare_parameter('height', 480)
    self.declare_parameter('fps', 30)

    self.width = self.get_parameter('width').value
    self.height = self.get_parameter('height').value
    self.fps = self.get_parameter('fps').value

    time_period = 1.0 / self.fps
    self.timer = self.create_timer(time_period, self.timer_callback)

    self.video_device_id = 4
    
    # ------------------------------------------------------------------------
    # [핵심] 파이썬 코드가 아니라 '시스템 명령어'로 설정을 강제 주입합니다.
    # OpenCV 설정이 안 먹힐 때 사용하는 최후의 수단이자 가장 확실한 방법입니다.
    # ------------------------------------------------------------------------
    # try:
    #     self.get_logger().info("시스템 명령어로 카메라 설정을 강제합니다...")
    #     # 1. 자동 노출 우선 기능 끄기 (FPS 저하의 주범)
    #     subprocess.run(f"v4l2-ctl -d /dev/video{self.video_device_id} -c exposure_auto_priority=0", shell=True)
    #     # 2. 자동 노출 끄기 (수동 모드: 1)
    #     subprocess.run(f"v4l2-ctl -d /dev/video{self.video_device_id} -c exposure_auto=1", shell=True)
    #     # 3. 셔터 속도(노출 시간) 고정 (값: 100) -> 30fps 확보용
    #     # 화면이 너무 어둡다면 150, 200으로 늘리세요.
    #     subprocess.run(f"v4l2-ctl -d /dev/video{self.video_device_id} -c exposure_absolute=100", shell=True)
    #     self.get_logger().info(">>> 하드웨어 설정 강제 적용 완료 (v4l2-ctl)")

    # except Exception as e:
    #     self.get_logger().error(f"시스템 명령어 실행 실패: {e}")

    # ------------------------------------------------------------------------
    # V4L2 백엔드로 카메라 열기
    self.cap = cv2.VideoCapture(self.video_device_id, cv2.CAP_V4L2)

    # 포맷 MJPG 설정 (USB 대역폭 확보)
    self.cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc('M', 'J', 'P', 'G'))
    self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.width)
    self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.height)
    self.cap.set(cv2.CAP_PROP_FPS, self.fps)
    self.cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)

    if not self.cap.isOpened():
      self.get_logger().error(f"카메라 ({self.video_device_id}) 연결 실패")
    else:
      # 설정 결과 확인
      w = self.cap.get(cv2.CAP_PROP_FRAME_WIDTH)
      h = self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT)
      fps = self.cap.get(cv2.CAP_PROP_FPS)
      # 코덱 확인
      fourcc = int(self.cap.get(cv2.CAP_PROP_FOURCC))
      codec = "".join([chr((fourcc >> 8 * i) & 0xFF) for i in range(4)])
      self.get_logger().info(f"OpenCV 상태: {w}x{h}, FPS: {fps}, 코덱: {codec}")

    self.cv_bridge = CvBridge()

  def timer_callback(self):
    ret, frame = self.cap.read()
    if ret:
      try:
        # cv2.imshow('My Window', frame)
        img = self.cv_bridge.cv2_to_imgmsg(frame, encoding='bgr8')
        img.header.stamp = self.get_clock().now().to_msg()
        img.header.frame_id = "camera_frame"
        self.publisher_.publish(img)

      except Exception as e:
        self.get_logger().error(f"변환 오류: {e}")
    cv2.waitKey(1)

def main(args=None):
  rclpy.init(args=args)
  node = ImgPublisher()
  try:
    rclpy.spin(node)
  except KeyboardInterrupt:
    pass
  finally:
    if node.cap.isOpened():
        node.cap.release()
    node.destroy_node()
    rclpy.shutdown()

if __name__ =='__main__':
  main()