import rclpy as rp
from rclpy.node import Node
from geometry_msgs.msg import Twist

class TurtlesimPublisher(Node):
  def __init__(self):
    super().__init__('turtlesim_publisher')
    self.publisher = self.create_publisher(Twist, 'turtle1/cmd_vel', 10)

    timer_period = 0.5  # seconds
    self.timer = self.create_timer(timer_period, self.timer_callback)

  def timer_callback(self):
    msg = Twist()
    msg.linear.x = 2.0
    msg.angular.z = 2.0
    self.publisher.publish(msg)
    self.get_logger().info(f'Publishing: linear_x={msg.linear.x}, angular_z={msg.angular.z}')

def main():
  rp.init()
  turtle_publisher = TurtlesimPublisher()
  rp.spin(turtle_publisher)

  turtle_publisher.destroy_node()
  rp.shutdown()


if __name__ == '__main__':
    main()