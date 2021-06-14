'''!
Декларация класса Depth_calculator и запуск ноды данного класса в main()
'''
import rclpy
import cv2 as cv
from rclpy.node import Node 
from sensor_msgs.msg import Image
from std_msgs.msg import Float32
from cv_bridge import CvBridge
import sys
import time
import numpy as np

class Depth_calculator(Node):
    '''!
    @brief Класс ноды Depth_calculator

    @details Читает -/aligned_depth_to_color/ - depth image натянутый на обычную картинку \n
    Читает -/object_detection/coords - здесь считываются координаты, по которым находится среднее значение высоты холста\n
    Публикует -/depth_node/canvas_average_depth - среднее значение высоты холста
    
    '''

    ## Конструктор класса
    def __init__(self):
        super().__init__("depth_node")
        ## Subscriber читает sensor_msgs.msg.Image из топика камеры /camera/aligned_depth_to_color/image_raw
        self.subscriber = self.create_subscription(
            Image,
            "/camera/aligned_depth_to_color/image_raw",
            self.callback,
            10
        )
        ## Subscriber читает координаты из топика камеры /object_detection/coords
        self.sub_for_coords = self.create_subscription(
            Image, 
            "/object_detection/coords", 
            self.get_coords,
            10
        )
        ## Publisher публикует std_msgs.msg.Float32 значение средней высоты холста в топик /object_detection/coords
        self.pub_canvas_average_depth = self.create_publisher(
            Float32, 
            "/depth_node/canvas_average_depth", 
            1
        )
        ## cvbridge для передачи изображения в формате ros msg 
        self.cv_bridge = CvBridge()
        ## cvbridge для передачи координат в формате ros msg 
        self.bridge_coords = CvBridge()
        ## Координаты пикселей вершин
        self.coords = np.array([])

    def callback(self, msg):
        '''!
        Вычисление и публикация средней высоты холста
        @return: null
        '''
        depths = self.cv_bridge.imgmsg_to_cv2(msg, desired_encoding = "16UC1")
        if self.coords.size:
            self.average = calc_average(depths, self.coords)
            msg2 = Float32()
            msg2.data = self.average
            self.pub_canvas_average_depth.publish(msg2)

    ##  Получение координат-пикселей вершин
    def get_coords(self, msg):
        self.coords = self.bridge_coords.imgmsg_to_cv2(msg, desired_encoding = "16UC1")

## Вычисление среднего
def calc_average(data, coords):
    #заглушка, нужно считать
    average = np.round(860.0)
    return average


def main(args = None):
    '''!
    Запуск ноды
    @param args: аргументы
    @return: null
    '''
    rclpy.init(args = args)
    calculator = Depth_calculator()
    rclpy.spin(calculator)
    calculator.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
