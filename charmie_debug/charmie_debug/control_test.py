import sys
import rclpy
import time
from rclpy.node import Node
from rclpy.duration import Duration
from rclpy.action import ActionClient

from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint

class MinimalPublisher(Node):

    def __init__(self):
        super().__init__('minimal_publisher')
        self._action_client = ActionClient(self, FollowJointTrajectory, '/joint_trajectory_controller/follow_joint_trajectory')
        
    def send_goal(self, a1, a2, a3, a4, a5, a6, a7, a8):
        goal_msg = FollowJointTrajectory.Goal()

        joint_names = ['camera_joint_z', 'camera_joint_y', 'base_joint_z',
                       'mn_joint2', 'mn_joint3', 'mn_joint4',
                       'left_gripper_joint', 'right_gripper_joint']

        points = []
        point1 = JointTrajectoryPoint()
        point1.positions = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0] #start point

        point2 = JointTrajectoryPoint()
        point2.time_from_start = Duration(seconds=1, nanoseconds=0).to_msg()
        point2.positions = [a1, a2, a3, a4, a5, a6, a7, a8] #finish point

        points.append(point1)
        points.append(point2)

        goal_msg.goal_time_tolerance = Duration(seconds=1, nanoseconds=0).to_msg()
        goal_msg.trajectory.joint_names = joint_names
        goal_msg.trajectory.points = points

        self._action_client.wait_for_server()
        self._send_goal_future = self._action_client.send_goal_async(goal_msg, feedback_callback=self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal Rejected :(')
            return

        self.get_logger().info('Goal Accepted :)')

        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self._get_result_callback)

    def get_result_callback(self, future):
        result = future.result().result
        self.get_logger().info('Result: ' + str(result))
        rclpy.shutdown()

    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback

def main(args=None):
    rclpy.init(args=args)

    minimal_publisher = MinimalPublisher()
    
    #STATE 0 - INIT
    #angle = float(sys.argv[1])
    a1 = 0.55
    a2 = 0.0
    a3 = 0.0
    a4 = 0.0
    a5 = 0.0
    a6 = 1.57
    a7 = 0.0
    a8 = 0.0
    future = minimal_publisher.send_goal(a1, a2, a3, a4, a5, a6, a7, a8)
    time.sleep(2)

    #STATE 1 - GRASP
    #angle = float(sys.argv[1])
    a1 = 0.55
    a2 = 0.0
    a3 = 0.0
    a4 = 0.0
    a5 = 0.0
    a6 = 1.57
    a7 = 0.005
    a8 = 0.005
    future = minimal_publisher.send_goal(a1, a2, a3, a4, a5, a6, a7, a8)
    time.sleep(2)

    #STATE 2 - RAISE
    #angle = float(sys.argv[1])
    a1 = 0.55
    a2 = 0.0
    a3 = 0.0
    a4 = 0.0
    a5 = 0.0
    a6 = 0.0
    a7 = 0.005
    a8 = 0.005
    future = minimal_publisher.send_goal(a1, a2, a3, a4, a5, a6, a7, a8)
    time.sleep(2)
    
    rclpy.spin(minimal_publisher)

    
    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
