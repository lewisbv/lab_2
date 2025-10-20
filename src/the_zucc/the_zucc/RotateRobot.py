# Lewis Busch-Vogel and Akash Iyer
# Lab 2 - Detect object and rotate to follow

import rclpy
from rclpy.node import Node
import cv2
import numpy as np
from std_msgs.msg import Int32
from geometry_msgs.msg import Twist

class RotateRobot(Node):
    def __init__(self):	
        super().__init__('rotate_robot')
        self.subscription = self.create_subscription(Int32, "DetectObject", self.rotate_callback,10)
        self.publisher_ = self.create_publisher(Twist, "/cmd_vel", 5)

    def rotate_callback(self, object_center: Int32):
        image_width = 320
        image_center = image_width / 2
        diff = object_center.data - image_center
        scale = 100
        w = 0

        if diff > 10:
            w = -0.5
        elif diff < 10:
            w = 0.5
        else:
            w = 0

        twist = Twist()
        twist.angular.z = float(w)
        self.publisher_.publish(twist)

def main():
    rclpy.init()
    node = RotateRobot()
    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
