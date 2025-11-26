import rclpy as rp
from rclpy.node import Node
from turtlesim.msg import Pose
from geometry_msgs.msg import Twist

from my_first_package_msgs.msg import CmdAndPoseVel

class CmdAndPose(Node):
  def __init__(self):
    super().__init__('turtlesim_cmd_pose')
    self.sub_pose = self.create_subscription(
      Pose, 'turtle1/pose', self.pose_callback, 10)
    self.sub_cmdvel = self.create_subscription(
      Twist, 'turtle1/cmd_vel', self.cmdvel_callback, 10)

    self.timer_period = 1.0  # seconds
    self.publisher = self.create_publisher(CmdAndPoseVel, 'cmd_and_pose', 10)
    self.timer = self.create_timer(self.timer_period, self.timer_callback)

    self.cmd_pose = CmdAndPoseVel()
    #self.subscription  # prevent unused variable warning

  def pose_callback(self, msg):
    self.cmd_pose.pose_x = msg.x
    self.cmd_pose.pose_y = msg.y
    self.cmd_pose.linear_vel = msg.linear_velocity
    self.cmd_pose.angular_vel = msg.angular_velocity
    #print(self.cmd_pose)
    #self.get_logger().info(f'Turtle Position -> x: {msg.x}, y: {msg.y}, theta: {msg.theta}')

  def cmdvel_callback(self, msg):
    self.cmd_pose.cmd_vel_linear = msg.linear.x
    self.cmd_pose.cmd_vel_angular = msg.angular.z
    print(self.cmd_pose)
    #self.get_logger().info(f'Cmd Vel -> linear_x: {msg.linear.x}, angular_z: {msg.angular.z}')

  def timer_callback(self):
    self.publisher.publish(self.cmd_pose)
    # self.get_logger().info('Published CmdAndPoseVel message')

def main():
  rp.init()
  turtle_cmd_pose_node = CmdAndPose()
  rp.spin(turtle_cmd_pose_node)

  turtle_cmd_pose_node.destroy_node()
  rp.shutdown()


if __name__ == '__main__':
    main()