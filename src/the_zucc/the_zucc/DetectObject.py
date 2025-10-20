# Lewis Busch-Vogel and Akash Iyer
# Lab 2 - Detect object and rotate to follow

import rclpy
from rclpy.node import Node
import cv2
import numpy as np
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import Int32

class DetectObject(Node):
    def __init__(self):	
        super().__init__('detect_object')
        self.subscription = self.create_subscription(CompressedImage, '/image_raw/compressed', self.detect_callback, 10)
        self.publisher_ = self.create_publisher(Int32, "DetectObject", 10)
    
    def process(self, frame):
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        yellow_lower = np.array([21, 100, 150])
        yellow_upper = np.array([40, 255, 255])

        yellow_mask = cv2.inRange(hsv, yellow_lower, yellow_upper)
        ret1, thresh_frame = cv2.threshold(yellow_mask, 0, 255, cv2.THRESH_BINARY)
        blurred_frame =  cv2.GaussianBlur(thresh_frame, (21, 21), 0)

        contours, frame = cv2.findContours(blurred_frame, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        return contours

    def detect_callback(self, msg: CompressedImage):
        np_arr = np.frombuffer(msg.data, np.uint8)
        frame = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)

        if frame is None:
            return
        
        contours = self.process(frame)
        msg = Int32()
        msg.data = 160

        if contours:
            contour = max(contours, key=cv2.contourArea)
            x, y, w, h = cv2.boundingRect(contour)
            center = (int(x + w / 2), int(y + h / 2))
            
            msg.data = center[0]

        self.publisher_.publish(msg)

def main():
    rclpy.init()
    node = DetectObject()
    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
