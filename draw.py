import rclpy
import cv2 as cv
import numpy as np
from rclpy.node import Node 
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import time

MAXINT = 999999999

class Detector(Node):
    def __init__(self):
        super().__init__("detector_node")
        self.subscriber = self.create_subscription(
            Image, 
            "/camera/color/image_raw", 
            self.callback, 
            10
        )
        self.pub = self.create_publisher(
            Image, 
            "/object_detection/output", 
            1
        )
        self.pub2 = self.create_publisher(
            Image, 
            "/object_detection/coords",
            1
        )
        self.cv_bridge = CvBridge()
        self.cv_bridge2 = CvBridge()
        
        self.declare_parameter('first',375)
        self.declare_parameter('second',185)
        self.declare_parameter('thrsh_low', 100000)
        self.declare_parameter('thrsh_high', 110000)
        
        self.min_box, self.cnt = [],[]

    def callback(self, msg):
        first = self.get_parameter('first').get_parameter_value().integer_value
        second = self.get_parameter('second').get_parameter_value().integer_value
        thrsh_low = self.get_parameter('thrsh_low').get_parameter_value().integer_value
        thrsh_high = self.get_parameter('thrsh_high').get_parameter_value().integer_value

        _range = set(range(thrsh_low,thrsh_high))

        time_now = time.time()
        
        img_opencv = self.cv_bridge.imgmsg_to_cv2(msg, desired_encoding = "bgr8") 
        canny = cv.Canny(img_opencv, first, second)
        ret, thresh = cv.threshold(canny, 127, 255, 0)
        contours, hierarchy = cv.findContours(thresh, cv.RETR_TREE, cv.CHAIN_APPROX_SIMPLE)
        
        #not a python way but
        min_area = MAXINT
        # перебираем все найденные контуры в цикле
        for cnt in contours:
            area, box = contouring(cnt)
            if (area in _range) and (area <= min_area):
                min_area = area
                self.min_box = box
                self.cnt = cnt
        if self.min_box.size:
            cv.drawContours(img_opencv,[self.min_box],0,(255,0,0),2)
            #не тру, переписать
            #передавать контур а не координаты?
            msg2 = self.cv_bridge2.cv2_to_imgmsg(self.min_box.astype(np.uint16))
            msg2.header =  msg.header
            self.pub2.publish(msg2)



        img_msg = self.cv_bridge.cv2_to_imgmsg(img_opencv, encoding = "bgr8")
        img_msg.header =  msg.header
        self.pub.publish(img_msg)
        self.get_logger().info("detection took {}s".format(time.time()-time_now))
 

def contouring(cnt):
    rect = cv.minAreaRect(cnt) # пытаемся вписать прямоугольник
    box = cv.boxPoints(rect) # поиск четырех вершин прямоугольника
    box = np.int0(box) # округление координат      
    area = int(rect[1][0]*rect[1][1]) # вычисление площади
    return area, box

    
def main(args = None):
    rclpy.init(args = args)
    detector = Detector()
    rclpy.spin(detector)
    detector.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

        
