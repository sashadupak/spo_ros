import rclpy
import cv2 as cv
from rclpy.node import Node 
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from sys import argv
import time

class Detector(Node):
    def __init__(self):
        super().__init__("detector_node")
        self.pub = self.create_publisher(Image, "/object_detection/output", 1)
        self.subscriber = self.create_subscription(Image, "/camera/color/image_raw", self.callback, 10)
        self.cv_bridge = CvBridge()
        self.declare_parameter('first',150)
        self.declare_parameter('second',150)
        self.i = 0

    def callback(self, msg):
        self.i = self.i+1
        first = self.get_parameter('first').get_parameter_value().integer_value
        second = self.get_parameter('second').get_parameter_value().integer_value
        time_now = time.time()
        img_opencv = self.cv_bridge.imgmsg_to_cv2(msg, desired_encoding = "bgr8") 
        canny = cv.Canny(img_opencv, first, second)
        ret, thresh = cv.threshold(canny, 127, 255, 0)
        contours, hierarchy = cv.findContours(thresh, cv.RETR_TREE, cv.CHAIN_APPROX_SIMPLE)
        cv.drawContours(img_opencv, contours, -1, (0,255,0), 3)
        img_msg = self.cv_bridge.cv2_to_imgmsg(img_opencv, encoding = "bgr8")
        img_msg.header =  msg.header
        self.pub.publish(img_msg)
        self.get_logger().info("detection took {}s".format(time.time()-time_now))
    
def main(args = None):
    rclpy.init(args = args)
    detector = Detector()
    rclpy.spin(detector)
    detector.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
