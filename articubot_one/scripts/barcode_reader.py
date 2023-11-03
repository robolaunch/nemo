#!/usr/bin/env python3

import rclpy 
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped
import tf2_ros
import json
import time
from tf2_msgs.msg import TFMessage
from geometry_msgs.msg import Vector3
import numpy as np
import math

class BarcodeReader(Node):
    def __init__(self):
        
        super().__init__('barcode_reader')

        self.tf_pose = TFMessage()
        self.robot_pose = PoseStamped()

        self.pose = PoseStamped()

        self.barcode_subscription = self.create_subscription(String, 'barcode', self.barcode_callback, 10)
        self.tf_subscription      = self.create_subscription(TFMessage, 'tf', self.tf_callback, 10)
        self.pose_subscription    = self.create_subscription(PoseStamped, 'robot_position', self.robot_pose_callback, 10)

        self.barcode_publisher0 = self.create_publisher(String, 'barcode_pose0', 10)
        self.barcode_publisher1 = self.create_publisher(String, 'barcode_pose1', 10)
        self.barcode_publisher2 = self.create_publisher(String, 'barcode_pose2', 10)
        self.barcode_publisher3 = self.create_publisher(String, 'barcode_pose3', 10)

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.barcode_list = []

        time.sleep(3)

    def robot_pose_callback(self, data):
        self.pose = data
        # print(self.pose.pose.position.x)

    def barcode_callback(self, msg):

        id, code = msg.data.split("-")


        if not code in self.barcode_list:
            self.barcode_list.append(code)

            print("barcode: " + msg.data)
            print("barcode: " + code + " barcode list : " + str( self.barcode_list))
            print()

            _, _, yaw = self.quaternion_to_euler([self.pose.pose.orientation.w,
                                                  self.pose.pose.orientation.x,
                                                  self.pose.pose.orientation.y,
                                                  self.pose.pose.orientation.z,])
            
            magnitude = 0.5
            direction_vec = [math.cos(yaw+math.pi*0.5)*magnitude, math.sin(yaw+math.pi*0.5)*magnitude]


            json_obj = {
                "barcode": code,
                "coordinates":
                {
                    "x": self.pose.pose.position.x + direction_vec[0],
                    "y": self.pose.pose.position.y + direction_vec[1],
                    "yaw" : yaw* 180/math.pi
                }
            }

            msg_send = String()
            msg_send.data = json.dumps(json_obj, ensure_ascii=False )  # str(json_obj) # msg.data + " " + str(self.pose.pose.position.x) + " " + str(self.pose.pose.position.y)

            if   id == "0":
                self.barcode_publisher0.publish(msg_send)
            elif id == "1":
                self.barcode_publisher1.publish(msg_send)
            elif id == "2":
                self.barcode_publisher2.publish(msg_send)
            elif id == "3":
                self.barcode_publisher3.publish(msg_send)



    def tf_callback(self, data):
        self.tf_pose = data
        map2odom_pose = data.transforms[0].transform.translation
        odom2map_pose = data.transforms[0].transform.translation

        # print("x pose: ",odom2map.x + map2odom.x)
        # print("y pose: ",odom2map.y + map2odom.y)
        # print()

        self.robot_pose.pose.position.x = odom2map_pose.x + map2odom_pose.x
        self.robot_pose.pose.position.y = odom2map_pose.y + map2odom_pose.y
        self.robot_pose.header.stamp = self.get_clock().now().to_msg()

    def quaternion_to_euler(self, quaternion):
        w, x, y, z = quaternion

        # Calculate Euler angles (roll, pitch, yaw)
        roll  = math.atan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y))
        pitch = math.asin(2 * (w * y - z * x))
        yaw   = math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))

        return roll, pitch, yaw


        

if __name__ == '__main__':
    rclpy.init()
    node = BarcodeReader()
    rclpy.spin(node)
    rclpy.shutdown()