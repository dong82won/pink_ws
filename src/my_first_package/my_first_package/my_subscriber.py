import rclpy as rp
from rclpy.node import Node
from turtlesim.msg import Pose

class TurtlesimSubscriber(Node):
  def __init__(self):
    super().__init__('turtlesim_subscriber')
    self.subscription = self.create_subscription(
      Pose,
      'turtle1/pose',
      self.listener_callback,
      10)
    #self.subscription  # prevent unused variable warning

  def listener_callback(self, msg):
    self.get_logger().info(f'Turtle Position -> x: {msg.x}, y: {msg.y}, theta: {msg.theta}')

def main():
  rp.init()
  turtle_subscriber = TurtlesimSubscriber()
  rp.spin(turtle_subscriber)

  turtle_subscriber.destroy_node()
  rp.shutdown()


if __name__ == '__main__':
    main()