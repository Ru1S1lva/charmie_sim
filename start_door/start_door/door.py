#!/usr/bin/env python3
import rclpy
import numpy as np
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool
from rclpy.node import Node

class DoorNode(Node):

    def __init__(self):
        super().__init__("DOOR_NODE")
        self.get_logger().info("Initiliased Door Node Node")

        #LIDAR SUB
        self.lidar_sub = self.create_subscription(LaserScan, 'scan', self.lidar_callback, 10)
        
        #DOOR PUB
        self.door_pub = self.create_publisher(Bool, 'door_state', 10) #True-Close  False-Open
        #self.door_open = Bool()

    def lidar_callback(self, scan: LaserScan):

        door_state = Bool()
        middle_index = len(scan.ranges) // 2  # Índice do sensor do meio
        middle_values = scan.ranges[middle_index-10:middle_index+10]  # Leitura do sensor do meio
        middle_range = np.mean(middle_values)
        """ print("v1: ", scan.ranges[middle_index-1])
        print("v2: ", scan.ranges[middle_index])
        print("v3: ", scan.ranges[middle_index+1])"""
        print("media: ", middle_range)

        if 0.3 <= middle_range <= 1:
            door_state.data = True
            self.get_logger().info('Door state: Closed')
            #print('Door_state: ', door_state)
        else:
            door_state.data = False
            self.get_logger().info('Door state: Open')
            #print('Door_state: ', door_state)
        
        self.door_pub.publish(door_state)


def main(args=None):
    rclpy.init(args=args)
    node = DoorNode()
    rclpy.spin(node)
    rclpy.shutdown()