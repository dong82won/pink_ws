import rclpy as rp
from rclpy.node import Node
from rclpy.action import ActionServer

from my_first_package_msgs.action import DistTurtle


class DistTurtleActionServer(Node):

  def __init__(self):
      super().__init__('dist_turtle_action_server')
      self._action_server = ActionServer(
          self,
          DistTurtle,
          'dist_turtle',
          self.execute_callback)

  def execute_callback(self, goal_handle):
      # self.get_logger().info('Executing goal...')

      # target_distance = goal_handle.request.target_distance
      # current_distance = 0.0
      # feedback_msg = DistTurtle.Feedback()

      # while current_distance < target_distance:
      #     # Simulate distance increment
      #     current_distance += 0.1
      #     feedback_msg.current_distance = current_distance
      #     goal_handle.publish_feedback(feedback_msg)
      #     self.get_logger().info(f'Current Distance: {current_distance:.2f}')

      goal_handle.succeed()
      result = DistTurtle.Result()
      return result

def main(args=None):
    rp.init(args=args)
    dist_turtle_action_server = DistTurtleActionServer()
    rp.spin(dist_turtle_action_server)
    rp.shutdown()

if __name__ == '__main__':
    main()
