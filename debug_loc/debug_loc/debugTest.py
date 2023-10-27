#!/usr/bin/env python3
import rclpy
from std_msgs.msg import Int16, Bool
from geometry_msgs.msg import Pose2D, PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
import math
import numpy as np

class TestNode(Node):

    def __init__(self):
        super().__init__("LOCAL_DEBUG")
        self.get_logger().info("Initiliased Local Debug Node")
        self.counter = 0

        #self.local_pos = self.create_publisher(PoseWithCovarianceStamped, "flag_start_button", 10)
        self.amcl_pos = self.create_subscription(PoseWithCovarianceStamped, "amcl_pose", self.start_amcl_callback, 10)
        self.local_pos_sub = self.create_subscription(Odometry, "odom", self.odom_callback, 10)
        
        self.init_pose_sub = self.create_subscription(PoseWithCovarianceStamped,"initialpose", self.get_init_pose_callback, 10)
        self.init_pose = PoseWithCovarianceStamped()
        self.is_true = Bool()

    def get_init_pose_callback(self, init_pose: PoseWithCovarianceStamped):
        print('Initial pose :', init_pose)
        self.is_true.data = True
        self.init_pose = init_pose              

    def start_amcl_callback(self, amcl: PoseWithCovarianceStamped):
        x = amcl.pose.pose.position.x
        y = amcl.pose.pose.position.y

        qx = amcl.pose.pose.orientation.x
        qy = amcl.pose.pose.orientation.y
        qz = amcl.pose.pose.orientation.z
        qw = amcl.pose.pose.orientation.w

        #YAW
        roll, pitch, yaw = euler_from_quaternion(qx, qy, qz, qw)
        covariance = amcl.pose.covariance
        std_deviation = np.sqrt(np.diag(covariance))
        rmse = np.sqrt(np.sum(np.diag(covariance)))

        print("AMCL X: ", x)
        print("AMCL Y: ", y)
        print("AMCL YAW: ", yaw)
        print("-------------------")
        print("COVARIANCE: \n", covariance)
        print("-------------------")
        print("STD: \n", std_deviation)
        print("-------------------")
        print("RMSE: \n", rmse)
        print("-------------------")

    def get_quaternion_from_euler(self, roll, pitch, yaw):
        """
        Convert an Euler angle to a quaternion.
        
        Input
            :param roll: The roll (rotation around x-axis) angle in radians.
            :param pitch: The pitch (rotation around y-axis) angle in radians.
            :param yaw: The yaw (rotation around z-axis) angle in radians.
        
        Output
            :return qx, qy, qz, qw: The orientation in quaternion [x,y,z,w] format
        """
        qx = np.sin(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) - np.cos(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
        qy = np.cos(roll/2) * np.sin(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.cos(pitch/2) * np.sin(yaw/2)
        qz = np.cos(roll/2) * np.cos(pitch/2) * np.sin(yaw/2) - np.sin(roll/2) * np.sin(pitch/2) * np.cos(yaw/2)
        qw = np.cos(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
        
        return [qx, qy, qz, qw]

    def odom_callback(self, odom: Odometry):
        if self.is_true:
            coord_x = self.init_pose.pose.pose.position.x
            coord_y = self.init_pose.pose.pose.position.y
            coord_theta = self.init_pose.pose.pose.orientation.z
            print(coord_x, coord_y, coord_theta)

            quaternion = self.get_quaternion_from_euler(0,0,coord_theta)

            odom = Odometry()
            odom.pose.pose.position.x = coord_x # x corrdinates
            odom.pose.pose.position.y = coord_y # y corrdinates 
            
            odom.pose.pose.orientation.x = quaternion[0]
            odom.pose.pose.orientation.y = quaternion[1]
            odom.pose.pose.orientation.z = quaternion[2]
            odom.pose.pose.orientation.w = quaternion[3] 

            self.odometry_publisher.publish(odom)



 
def euler_from_quaternion(x, y, z, w):
    """
    Convert a quaternion into euler angles (roll, pitch, yaw)
    roll is rotation around x in radians (counterclockwise)
    pitch is rotation around y in radians (counterclockwise)
    yaw is rotation around z in radians (counterclockwise)
    """
    t0 = +2.0 * (w * x + y * z)
    t1 = +1.0 - 2.0 * (x * x + y * y)
    roll_x = math.atan2(t0, t1)
     
    t2 = +2.0 * (w * y - z * x)
    t2 = +1.0 if t2 > +1.0 else t2
    t2 = -1.0 if t2 < -1.0 else t2
    pitch_y = math.asin(t2)
     
    t3 = +2.0 * (w * z + x * y)
    t4 = +1.0 - 2.0 * (y * y + z * z)
    yaw_z = math.atan2(t3, t4)
     
    return roll_x, pitch_y, yaw_z # in radians
        

def main(args=None):
    rclpy.init(args=args)
    node = TestNode()
    rclpy.spin(node)
    rclpy.shutdown()