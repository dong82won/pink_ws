import rclpy as rp
from rclpy.node import Node
from rclpy.action import ActionServer
import time
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
        feedback_msg=DistTurtle.Feedback()
        for n in range(0, 10):
            feedback_msg.remaining_dist = float(n)
            goal_handle.publish_feedback(feedback_msg)
            self.get_logger().info(f'Feedback: {feedback_msg.remaining_dist} units traveled')
            time.sleep(0.5)

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
