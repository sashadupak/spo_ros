'''!
Декларация класса Detector и запуск ноды данного класса в main()
'''
import sys
import rclpy
import cv2 as cv
import numpy as np
from rclpy.node import Node 
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import time

class Detector(Node):
    '''!
    @brief Класс ноды Detector
    @details Читает - /camera/color/image_raw - обычное изображение с камеры \n
    Публикует в /object_detection/output - исходное изображение, с холстом обведенным рамкой \n
    Публикует в /object_detection/coords - координаты пикселей-вершин \n
    Параметры first, second - параметры фильтра Кэнни \n
    Параметры thrsh_low, thrsh_high - параметры площади холста
    '''

    ## Конструктор класса
    def __init__(self):
        super().__init__("detector_node")
        ## Subscriber читает sensor_msgs.msg.Image из топика камеры /camera/color/image_raw
        self.subscriber = self.create_subscription(
            Image, 
            "/camera/color/image_raw", 
            self.callback, 
            10
        )
        ## Publisher публикует sensor_msgs.msg.Image c обведенным холстом в топик /object_detection/output
        self.pub = self.create_publisher(
            Image, 
            "/object_detection/output", 
            1
        )
        ## Publisher публикует координаты пикселей вершин холста в топик /object_detection/coords
        self.pub2 = self.create_publisher(
            Image, 
            "/object_detection/coords",
            1
        )
        ## cvbridge для передачи изображения в формате ros msg 
        self.cv_bridge = CvBridge()
        ## cvbridge для передачи изображения в формате ros msg 
        self.cv_bridge2 = CvBridge()

        self.declare_parameter('first',375)
        self.declare_parameter('second',185)
        self.declare_parameter('thrsh_low', 100000)
        self.declare_parameter('thrsh_high', 110000)
        ## координаты вершин контура
        self.min_box = []
        ## координаты контура        
        self.cnt = []
    def callback(self, msg):
        '''!
        функция поиска контура и публикации координат пикселей вершин
        @return: null
        '''
        first = self.get_parameter('first').get_parameter_value().integer_value
        second = self.get_parameter('second').get_parameter_value().integer_value
        thrsh_low = self.get_parameter('thrsh_low').get_parameter_value().integer_value
        thrsh_high = self.get_parameter('thrsh_high').get_parameter_value().integer_value
        # Множество с границами площадей
        _range = set(range(thrsh_low,thrsh_high))
        # Перевод из sensor_msgs.msg.Image в cv2-like формат (numpy array) 
        img_opencv = self.cv_bridge.imgmsg_to_cv2(msg, desired_encoding = "bgr8") 
        # Фильтр кэннни
        canny = cv.Canny(img_opencv, first, second)
        # Поиск контуров
        ret, thresh = cv.threshold(canny, 127, 255, 0)
        contours, hierarchy = cv.findContours(thresh, cv.RETR_TREE, cv.CHAIN_APPROX_SIMPLE)
                
        min_area = sys.maxsize
        # перебираем все найденные контуры в цикле и отфильтровываем лишние по площади
        for cnt in contours:
            area, box = contouring(cnt)
            if (area in _range) and (area <= min_area):
                min_area = area
                self.min_box = box
                self.cnt = cnt
        if len(self.min_box):
            cv.drawContours(img_opencv,[self.min_box],0,(255,0,0),2)
            #не тру, переписать
            #передавать контур а не координаты?
            ## формиурем сообщение
            msg2 = self.cv_bridge2.cv2_to_imgmsg(self.min_box.astype(np.uint16))
            msg2.header =  msg.header
            ## передаем сообщение
            self.pub2.publish(msg2)
        ## формируем сообщение и публикуем изображение
        img_msg = self.cv_bridge.cv2_to_imgmsg(img_opencv, encoding = "bgr8")
        img_msg.header =  msg.header
        self.pub.publish(img_msg)
 

def contouring(cnt):
    '''!
    Выделение прямоугольного контура
    @param cnt: контур
    @return: area, box - площадь и массив вершин
    '''
    rect = cv.minAreaRect(cnt) # пытаемся вписать прямоугольник
    box = cv.boxPoints(rect) # поиск четырех вершин прямоугольника
    box = np.int0(box) # округление координат      
    area = int(rect[1][0]*rect[1][1]) # вычисление площади
    return area, box

## Запуск ноды
def main(args = None):
    '''!
    Запуск ноды
    @param args: аргументы
    @return: null
    '''
    rclpy.init(args = args)
    detector = Detector()
    rclpy.spin(detector)
    detector.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

        
