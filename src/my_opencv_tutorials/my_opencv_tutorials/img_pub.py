import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

import cv2

class ImgPublisher(Node):
  def __init__(self):
    super().__init__('image_publisher')
    self.publisher_ = self.create_publisher(Image, 'image_raw', 10)
    
    time_period = 0.01
    self.timer = self.create_timer(time_period, self.timer_callback)

    self.video_device_id = 6
    self.cap = cv2.VideoCapture(self.video_device_id)

    if not self.cap.isOpened():
      self.get_logger().error(f"카메라 ({self.video_device_id})를 열 수 없습니다.")
    else:
      self.get_logger().info(f"카메라({self.video_device_id}) 연결 성공")
    self.cv_bridge = CvBridge()

  def timer_callback(self):
    ret, frame = self.cap.read()

    if ret:
      try:
        img = self.cv_bridge.cv2_to_imgmsg(frame, encoding='bgr8')
        self.publisher_.publish(img)
      except Exception as e:
        self.get_logger().error(f"변환 오류: {e}")
    else:
      self.get_logger().warning("프레임을 읽을 수 없습니다. (Camera disconnected?)")

    cv2.waitKey(1)

def main(args=None):
  rclpy.init(args=args)
  node = ImgPublisher()

  try:
    rclpy.spin(node)
  except KeyboardInterrupt:
    pass
  finally:
    node.cap.release()  
    node.destroy_node()
    rclpy.shutdown()

if __name__ =='__main__':
  main()


