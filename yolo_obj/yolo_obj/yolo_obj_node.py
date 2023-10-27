#!/usr/bin/env python3
from ultralytics import YOLO
from ultralytics.yolo.engine.results import Results
from ultralytics.yolo.utils import DEFAULT_CFG, ROOT, ops
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, String
from sensor_msgs.msg import Image
from charmie_interfaces.msg import Object, ObjectArray
from cv_bridge import CvBridge
import cv2 
import cvzone

import numpy as np

import math

bridge = CvBridge()

""" classNames = ["Apple", "Bag", "Banana", "Bottle", "Bowl", "Chair",
              "Cup", "Fork", "Knife", "Manga", "Mug", "Pear", "Person",
              "Plastic-bag", "Plate", "Pringles", "Shelf", "Spoon",
              "Table", "Tin-can", "Trash-can"] """

cylindrical = ['Bottle', 'Cup', 'Mug', 'Pringles', 'Tin-can']

rectangle = ['Fork', 'Knife', 'Spoon']

oval = ['Bowl', 'Plate']

random = ['Apple', 'Banana', 'Manga', 'Pear']

class Yolo_obj(Node):

    def __init__(self):
        super().__init__('Yolo_Obj')

        self.model = YOLO('/home/ruisilva/charmie_sim/src/yolo_obj/yolo_obj/best.pt')

        self.yolov8_inference = ObjectArray()

        self.subscription = self.create_subscription(Image, '/camera/image_raw', self.camera_callback, 10)
        #self.subscription 

        self.yolov8_pub = self.create_publisher(ObjectArray, "/Yolov8_Inference", 1)
        self.img_pub = self.create_publisher(Image, "/inference_result", 1)

        self.obj_format_pub = self.create_publisher(String, "/obj_format", 1)

    def camera_callback(self, data):

        img = bridge.imgmsg_to_cv2(data, "bgr8")
        results = self.model(img)
        msg = String()

        self.yolov8_inference.header.frame_id = "inference"
        #self.yolov8_inference.header.stamp = Yolo_Obj.get_clock().now().to_msg()

        format_detected = []

        for r in results:
            boxes = r.boxes
            for box in boxes:
                self.inference_result = Object()
                b = box.xyxy[0].to('cpu').detach().numpy().copy()  # get box coordinates in (top, left, bottom, right) format
                c = box.cls
                self.inference_result.class_name = self.model.names[int(c)]
                self.inference_result.top = int(b[0])
                self.inference_result.left = int(b[1])
                self.inference_result.bottom = int(b[2])
                self.inference_result.right = int(b[3])
                self.yolov8_inference.objects.append(self.inference_result)

                if self.inference_result.class_name in cylindrical:
                    format_detected.append('Cylindrical')
                    #msg.data = 'Cylindrical'
                    #self.obj_format_pub.publish(msg)
                elif self.inference_result.class_name in rectangle:
                    format_detected.append('Rectangle')
                    #msg.data = 'Rectangle'
                    #self.obj_format_pub.publish(msg)
                elif self.inference_result.class_name in oval:
                    format_detected.append('Oval')
                    #msg.data = 'Oval'
                    #self.obj_format_pub.publish(msg)
                elif self.inference_result.class_name in random:
                    format_detected.append('Random')
                    #msg.data = 'Random'
                    #self.obj_format_pub.publish(msg)
                else:
                    format_detected.append('None')

            #camera_subscriber.get_logger().info(f"{self.yolov8_inference}")
        if format_detected:
            msg.data = " ".join(format_detected)
            self.obj_format_pub.publish(msg)

        annotated_frame = results[0].plot()
        img_msg = bridge.cv2_to_imgmsg(annotated_frame)  

        self.img_pub.publish(img_msg)
        self.yolov8_pub.publish(self.yolov8_inference)
        self.yolov8_inference.objects.clear()
        
def main(args=None):
    rclpy.init(args=args)
    node = Yolo_obj()
    rclpy.spin(node)
    rclpy.shutdown()
