# 이해가 안된다!! 핑크랩 응용 3-10 일정한 거리를 이동시키는 액션 서버 구현하기

import rclpy as rp
from rclpy.node import Node
from rclpy.action import ActionServer
import time
from my_first_package_msgs.action import DistTurtle

from rclpy.executors import MultiThreadedExecutor
import math
from turtlesim.msg import Pose
from geometry_msgs.msg import Twist
from my_first_package.my_subscriber import TurtlesimSubscriber

from rcl_interfaces.msg import SetParametersResult


class TurtlesimSub_Action(TurtlesimSubscriber):
    def __init__(self, ac_server):
        super().__init__()
        self._ac_server = ac_server

    def listener_callback(self, msg):
        self._ac_server.current_pose = msg

class DistTurtleActionServer(Node):

    def __init__(self):
        super().__init__('dist_turtle_action_server')
        self.total_dist = 0.0
        self.is_first_time = True
        self.current_pose = Pose()
        self.previous_pose = Pose()
        self.publisher = self.create_publisher(Twist, 'turtle1/cmd_vel', 10)
        self._action_server = ActionServer(self, DistTurtle,'dist_turtle',self.execute_callback)

        self.declare_parameter('quantile_time', 0.75)
        self.declare_parameter('almost_goal_time', 0.95)

        (guatile_time, almost_goal_time) = self.get_parameters(['quantile_time', 'almost_goal_time'])
        self.get_logger().info(f'Quantile time: {guatile_time.value} and  Almost goal time: {almost_goal_time.value}')#, f'Almost goal time: {almost_goal_time.value}')

        self.guantile_time = guatile_time.value
        self.almost_goal_time = almost_goal_time.value

        self.add_on_set_parameters_callback(self.parameter_callback)

    def parameter_callback(self, params):
        for param in params:
            if param.name == 'quantile_time':
                self.guantile_time = param.value
                self.get_logger().info(f'Quantile time changed to: {param.value}')
            if param.name == 'almost_goal_time':
                self.almost_goal_time = param.value
                self.get_logger().info(f'Almost goal time changed to: {param.value}')

        self.get_logger().info(f'Quantile time: {self.guantile_time} and Almost goal time: {self.almost_goal_time}')
        return SetParametersResult(successful=True)


    def calc_diff_pose(self):
        if self.is_first_time:
            self.previous_pose = self.current_pose
            self.is_first_time = False
            return 0.0

        diff = math.sqrt((self.current_pose.x - self.previous_pose.x) ** 2 +
                         (self.current_pose.y - self.previous_pose.y) ** 2)
        self.previous_pose = self.current_pose
        return diff

    def execute_callback(self, goal_handle):
        feedback_msg=DistTurtle.Feedback()

        msg = Twist()
        msg.linear.x = goal_handle.request.linear_x
        msg.angular.z = goal_handle.request.angular_z

        while True:
            self.total_dist += self.calc_diff_pose()
            feedback_msg.remaining_dist = goal_handle.request.dist - self.total_dist
            goal_handle.publish_feedback(feedback_msg)
            self.get_logger().info(f'Feedback: {feedback_msg.remaining_dist} units remaining')
            self.publisher.publish(msg)
            time.sleep(0.1)

            if feedback_msg.remaining_dist < 0.0:
                break

        goal_handle.succeed()
        result = DistTurtle.Result()

        result.pos_x = self.current_pose.x
        result.pos_y = self.current_pose.y
        result.pos_theta = self.current_pose.theta
        result.result_dist = self.total_dist

        self.total_dist = 0.0
        self.is_first_time = True
        return result

def main(args=None):
    rp.init(args=args)
    # dist_turtle_action_server = DistTurtleActionServer()
    # rp.spin(dist_turtle_action_server)
    # rp.shutdown()

    executor = MultiThreadedExecutor()
    ac = DistTurtleActionServer()
    sub = TurtlesimSub_Action(ac_server = ac)

    executor.add_node(ac)
    executor.add_node(sub)

    try:
        executor.spin()
    finally:
        ac.destroy_node()
        sub.destroy_node()
        rp.shutdown()

if __name__ == '__main__':
    main()
