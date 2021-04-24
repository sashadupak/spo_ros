from pointcloud2_to_numpy import *
import rclpy 
from rclpy.node import Node
from std_msgs.msg import Float32
import cv2 as cv
import time

class PCDListener(Node):

    def __init__(self):
        super().__init__('pcd_subscriber_node')  
        self.pcd_subscriber = self.create_subscription(
            PointCloud2,    # Msg type
            '/camera/depth/color/points',# topic
            self.listener_callback,      # Function to call
            10                          # QoS
        )
        self.sub_canvas_average_depth = self.create_subscription(
            Float32,
            "/depth_node/canvas_average_depth",
            self.callback_get_depth, 
            10
        )
        self.depth = int()

                
    def listener_callback(self, msg):
        array = pointcloud2_to_array(
            msg,
            split_rgb = True,
            remove_padding = True
        )
        pool = []
        points = pointcloud2_to_xyz_array(msg)
        ###надо тестить 
        #N = tbd
        #до этого передавать разницу между холстом и столом
        ###eps = difference/N
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

        print("left 1{} {}".format(minx,minx_y))
        print("left 2 {} {}".format(miny_x,miny))
        print("right 1 {} {}".format(maxx,maxx_y))
        print("right 2 {} {}".format(maxy_x,maxy))
       


    def callback_get_depth(self, msg):
        self.depth = msg.data

def main(args = None):
    # Boilerplate code.
    rclpy.init(args=args)
    pcd_listener = PCDListener()
    rclpy.spin(pcd_listener)
    pcd_listener.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
