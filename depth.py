import rclpy
from rclpy.node import Node 
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

class Detector(Node):
    def __init__(self):
        super().__init__("detector_node")
        self.subscriber = self.create_subscription(Image, "/camera/depth/image_rect_raw", self.callback, 1)
        self.cv_bridge = CvBridge()

    def callback(self, msg):
        depths = self.cv_bridge.imgmsg_to_cv2(msg, desired_encoding = "16UC1") 
        #array w*d(480*640 default) depths in mm for each pixel


def main(args = None):
    rclpy.init(args = args)
    detector = Detector()
    rclpy.spin(detector)
    detector.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
