import rclpy
import cv2 as cv
from rclpy.node import Node 
from sensor_msgs.msg import Image
from std_msgs.msg import Float32
from cv_bridge import CvBridge
import sys
import time
import csv
import numpy as np

class Depth_calculator(Node):

    def __init__(self):
        super().__init__("depth_node")
        self.subscriber = self.create_subscription(
            Image,
            "/camera/aligned_depth_to_color/image_raw",
            self.callback,
            10
        )
        self.sub_for_coords = self.create_subscription(
            Image, 
            "/object_detection/coords", 
            self.get_coords,
            10
        )
        self.pub_canvas_average_depth = self.create_publisher(
            Float32, 
            "/depth_node/canvas_average_depth", 
            1
        )

        self.cv_bridge = CvBridge()
        self.bridge_coords = CvBridge()
        self.coords = np.array([])


    def callback(self, msg):
        depths = self.cv_bridge.imgmsg_to_cv2(msg, desired_encoding = "16UC1")
        if self.coords.size:
            self.average = calc_average(depths, self.coords)
            msg2 = Float32()
            msg2.data = self.average
            self.pub_canvas_average_depth.publish(msg2)


    def get_coords(self, msg):
        #не тру, переписать
        self.coords = self.bridge_coords.imgmsg_to_cv2(msg, desired_encoding = "16UC1")
        #print(self.coords)


def calc_average(data, coords):
    #заглушка, нужно считать
    average = np.round(860.0)
    return average


def write_to_csv(name, source, mode=1):
    with open(name, mode='w') as output:
            output_writer = csv.writer(output, delimiter=',', quoting=csv.QUOTE_MINIMAL)
            if mode:
                for each in source:
                    output_writer.writerow(each)
            else:
                output_writer.writerow(source)
    
def main(args = None):
    rclpy.init(args = args)
    calculator = Depth_calculator()
    rclpy.spin(calculator)
    calculator.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
