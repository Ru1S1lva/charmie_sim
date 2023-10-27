import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from rclpy.action import ActionClient
from std_msgs.msg import String
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
import readchar

class JointController(Node):
    def __init__(self):
        super().__init__('joint_controller')
        self._action_client = ActionClient(self, FollowJointTrajectory, '/joint_trajectory_controller/follow_joint_trajectory')
        self.last_joint_positions = [0.0] * 9 # nr juntas
        self.active_joint = 0  # Inicialmente controlando a junta 0
        self.joint_names = self.joint_names = ['camera_joint_z', 'camera_joint_y', 'base_joint_z',
                            'mn_joint2', 'mn_joint3', 'mn_joint4', 'mn_joint5',
                            'left_gripper_joint', 'right_gripper_joint']
        
        self.left_gripper_joint = self.joint_names.index('left_gripper_joint')
        self.right_gripper_joint = self.joint_names.index('right_gripper_joint')

    def send_goal(self):
        goal_msg = FollowJointTrajectory.Goal()

        points = []
        point1 = JointTrajectoryPoint()
        point1.positions = self.last_joint_positions

        point2 = JointTrajectoryPoint()
        point2.time_from_start = Duration(seconds=1, nanoseconds=0).to_msg()
        point2.positions = self.last_joint_positions  # Não altera as posições

        points.append(point1)
        points.append(point2)

        goal_msg.goal_time_tolerance = Duration(seconds=1, nanoseconds=0).to_msg()
        goal_msg.trajectory.joint_names = self.joint_names
        goal_msg.trajectory.points = points

        self._action_client.wait_for_server()
        self._send_goal_future = self._action_client.send_goal_async(goal_msg)

    def update_joint(self, delta):
        if self.active_joint in [self.left_gripper_joint, self.right_gripper_joint]:
            # Se a junta ativa for um dos grippers, atualize ambos os grippers
            self.last_joint_positions[self.left_gripper_joint] += delta/4
            self.last_joint_positions[self.right_gripper_joint] += delta/4
        else:
            # Se não, atualize apenas a junta ativa
            self.last_joint_positions[self.active_joint] += delta
        self.send_goal()
        print(f"Joint Value: {[round(value, 3) for value in self.last_joint_positions]}")

    def change_active_joint(self, delta):
        self.active_joint += delta
        self.active_joint = max(0, min(len(self.last_joint_positions) - 1, self.active_joint))
        print(f"Active Joint: {self.joint_names[self.active_joint]}")

    def reset_all_joints(self):
        self.last_joint_positions = [0.0] * len(self.last_joint_positions)
        self.send_goal()
        print(f"Joints Reset")

def main(args=None):
    rclpy.init(args=args)
    node = JointController()

    try:
        while rclpy.ok():
            key = readchar.readchar()
            if key == 'w':
                node.update_joint(0.02)
            elif key == 's':
                node.update_joint(-0.02)
            elif key == 'a':
                node.change_active_joint(-1)
            elif key == 'd':
                node.change_active_joint(1)
            elif key == 'q':
                node.reset_all_joints()
            elif key == '\x1b':  # Tecla 'esc' para sair
                break  # Saia do loop

    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()