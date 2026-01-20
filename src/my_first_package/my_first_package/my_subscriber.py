import rclpy as rp
from rclpy.node import Node
from turtlesim.msg import Pose

class TurtlesimSubscriber(Node):
  def __init__(self):
    super().__init__('turtlesim_subscriber') # 노드 이름
    self.subscription = self.create_subscription(
      Pose,                          #데이터 타입
      'turtle1/pose',                #토픽 이름
      self.listener_callback,        #콜백 실행
      10)
    #self.subscription  # prevent unused variable warning

  def listener_callback(self, msg):
    self.get_logger().info(f'Turtle Position->x:{msg.x}, y:{msg.y}, theta:{msg.theta}')


def main():
  rp.init()
  turtle_subscriber = TurtlesimSubscriber()
  rp.spin(turtle_subscriber)

  turtle_subscriber.destroy_node()
  rp.shutdown()


if __name__ == '__main__':
    main()