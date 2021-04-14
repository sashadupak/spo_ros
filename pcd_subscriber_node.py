from pointcloud2_to_numpy import *
import rclpy 
from rclpy.node import Node

class PCDListener(Node):

    def __init__(self):
        super().__init__('pcd_subscriber_node')  
        self.pcd_subscriber = self.create_subscription(
            PointCloud2,    # Msg type
            '/camera/depth/color/points',# topic
            self.listener_callback,      # Function to call
            10                          # QoS
        )

                
    def listener_callback(self, msg):
        points = pointcloud2_to_xyz_array(msg)
        #pointcloud array [x,y,z] with respect to camera_link frame 
        
        
def main(args=None):
    # Boilerplate code.
    rclpy.init(args=args)
    pcd_listener = PCDListener()
    rclpy.spin(pcd_listener)
    pcd_listener.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
