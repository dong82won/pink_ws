import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CompressedImage  # [수정] CompressedImage 메시지 타입 추가
from cv_bridge import CvBridge
import cv2
import numpy as np  # [수정] 압축 데이터 처리를 위해 numpy 추가
import subprocess
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

class ImgPublisher(Node):
  def __init__(self):
    super().__init__('img_pub_node')
    
    # ---------------------------------------------------------
    # 1. QoS 설정 (Best Effort 설정)
    # ---------------------------------------------------------
    qos_profile = QoSProfile(
        reliability=ReliabilityPolicy.BEST_EFFORT,
        history=HistoryPolicy.KEEP_LAST,
        depth=1
    )

    # 2. 퍼블리셔 생성
    # (A) 원본 이미지 퍼블리셔 (기존)
    self.publisher_ = self.create_publisher(Image, 'image_raw', qos_profile)
    
    # (B) [추가] 압축 이미지 퍼블리셔 (새로 추가됨)
    # 동일한 Best Effort QoS를 사용하여 통신 끊김 방지
    self.publisher_compressed = self.create_publisher(CompressedImage, 'image_raw/compressed', qos_profile)

    # 3. 파라미터 설정
    self.declare_parameter('width', 640)
    self.declare_parameter('height', 480)
    self.declare_parameter('fps', 30)

    self.width = self.get_parameter('width').value
    self.height = self.get_parameter('height').value
    self.fps = self.get_parameter('fps').value

    self.video_device_id = 4
    
    # ------------------------------------------------------------------------
    # [선택] v4l2-ctl 하드웨어 강제 설정 (필요시 주석 해제하여 사용)
    # ------------------------------------------------------------------------
    # try:
    #     self.get_logger().info("시스템 명령어로 카메라 설정을 강제합니다...")
    #     subprocess.run(f"v4l2-ctl -d /dev/video{self.video_device_id} -c exposure_auto_priority=0", shell=True)
    #     subprocess.run(f"v4l2-ctl -d /dev/video{self.video_device_id} -c exposure_auto=1", shell=True)
    #     subprocess.run(f"v4l2-ctl -d /dev/video{self.video_device_id} -c exposure_absolute=100", shell=True)
    #     self.get_logger().info(">>> 하드웨어 설정 강제 적용 완료 (v4l2-ctl)")
    # except Exception as e:
    #     self.get_logger().error(f"시스템 명령어 실행 실패: {e}")

    # 4. 카메라 오픈 (V4L2 + MJPG)
    self.cap = cv2.VideoCapture(self.video_device_id, cv2.CAP_V4L2)
    self.cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc('M', 'J', 'P', 'G'))
    self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.width)
    self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.height)
    self.cap.set(cv2.CAP_PROP_FPS, self.fps)
    self.cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)

    if not self.cap.isOpened():
      self.get_logger().error(f"카메라 ({self.video_device_id}) 연결 실패")
    else:
      w = self.cap.get(cv2.CAP_PROP_FRAME_WIDTH)
      h = self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT)
      fps = self.cap.get(cv2.CAP_PROP_FPS)
      self.get_logger().info(f"OpenCV 상태: {w}x{h}, FPS: {fps}")

    self.cv_bridge = CvBridge()
    
    # 타이머 시작
    time_period = 1.0 / self.fps
    self.timer = self.create_timer(time_period, self.timer_callback)

  def timer_callback(self):
    ret, frame = self.cap.read()
    if ret:
      try:        
        # (옵션) 로컬 화면 띄우기 (Headless 환경이면 주석 처리 권장)
        # cv2.imshow('My Window', frame)

        # ---------------------------------------------------------
        # (A) 원본 이미지 발행 (/image_raw)
        # ---------------------------------------------------------
        current_time = self.get_clock().now().to_msg()
        
        img_msg = self.cv_bridge.cv2_to_imgmsg(frame, encoding='bgr8')
        img_msg.header.stamp = current_time
        img_msg.header.frame_id = "camera_link" # frame_id 통일
        self.publisher_.publish(img_msg)

        # ---------------------------------------------------------
        # (B) [추가] 압축 이미지 직접 발행 (/image_raw/compressed)
        # ---------------------------------------------------------
        msg_comp = CompressedImage()
        msg_comp.header.stamp = current_time
        msg_comp.header.frame_id = "camera_link"
        msg_comp.format = "jpeg"

        # JPEG 압축 (품질 50% 설정 - 전송 속도 향상)
        encode_param = [int(cv2.IMWRITE_JPEG_QUALITY), 50]
        success, encoded_img = cv2.imencode('.jpg', frame, encode_param)

        if success:
            # numpy array를 bytes로 변환하여 메시지에 담기
            msg_comp.data = np.array(encoded_img).tobytes()
            self.publisher_compressed.publish(msg_comp)

      except Exception as e:
        self.get_logger().error(f"발행 오류: {e}")
    
    # cv2.waitKey(1)

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