'''!
Декларация класса PCDListenerFinalPublisher и запуск ноды данного класса в main()
'''

from mypack.pointcloud2_to_numpy import *
import rclpy 
from rclpy.node import Node
from std_msgs.msg import Float32
from std_msgs.msg import Int64MultiArray
import cv2 as cv
import time

class PCDListenerFinalPublisher(Node):
    '''!
    @brief Класс ноды PCDListenerFinalPublisher
    @details читает - /camera/depth/color/points - поинтклауд \n
    читает - /depth_node/canvas_average_depth - среднее значение высоты уровня холста \n
    публикует в /result - список из 8 значений х,у 4 вершин холста в локальной системе координат
    '''

    ## Конструктор класса
    def __init__(self):
        super().__init__('pcd_subscriber_node')
        ## Subscriber читает sensor_msgs.msg.PointCloud2 массив PointCloud из топика /camera/depth/color/points
        self.pcd_subscriber = self.create_subscription(
            PointCloud2,    # Msg type
            '/camera/depth/color/points',# topic
            self.listener_callback,      # Function to call
            10                          # QoS
        )
        ## Subscriber читает std_msgs.msg.Float32 значение средней высоты из топика /depth_node/canvas_average_depth
        self.sub_canvas_average_depth = self.create_subscription(
            Float32,
            "/depth_node/canvas_average_depth",
            self.callback_get_depth, 
            10
        )
        ## Publisher публикует массив std_msgs.msg.Int64MultiArray координат вершин в топик /result
        self.publisher = self.create_publisher(
            Int64MultiArray,
            '/result',
            10
        )
        ## Координаты вершин
        self.coordinates = []  
        ## Среднее значение высоты холста
        self.depth = int()

    def pub_values(self):
        '''!
        Функция публикации вершин
        @return: null
        '''
        msg = Int64MultiArray()
        msg.data = self.coordinates
        self.publisher.publish(msg)

    def listener_callback(self, msg):
        '''!
        функция чтения Pointcloud топика и вычисление координат вершин
        @return: null
        '''
        array = pointcloud2_to_array(
            msg,
            split_rgb = True,
            remove_padding = True
        )
        pool = []
        points = pointcloud2_to_xyz_array(msg)
        """
        надо тестить 
        N = tbd
        до этого передавать разницу между холстом и столом
        eps = difference/N
        """
        eps=15
        for each in points:
            if abs(np.round(each[2]*1000) - self.depth) <= eps:
                pool.append([each[0]*1000,each[1]*1000])
        #find(min\max) поадекватнее, учесть что холст под углом
        #на этапе распознавания добавить определение угла наклона 
        pool = np.array(pool)
        minx = min(pool[:,0])
        maxx = max(pool[:,0])
        miny = min(pool[:,1])
        maxy = max(pool[:,1])

        minx_y = np.mean(pool[pool[:,0] == minx][:,1])
        maxx_y = np.mean(pool[pool[:,0] == maxx][:,1])
        miny_x = np.mean(pool[pool[:,1] == miny][:,0])
        maxy_x = np.mean(pool[pool[:,1] == maxy][:,0])


        to_pub = [minx,minx_y,miny_x,miny,maxx,maxx_y,maxy_x,maxy]
        self.coordinates = [int(each) for each in to_pub]
        self.pub_values()

    def callback_get_depth(self, msg):
        self.depth = msg.data
## Запуск ноды
def main(args = None):
    '''!
    Запуск ноды
    @param args: аргументы
    @return: null
    '''
    rclpy.init(args=args)
    pcd = PCDListenerFinalPublisher()
    rclpy.spin(pcd_listener)
    pcd.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
