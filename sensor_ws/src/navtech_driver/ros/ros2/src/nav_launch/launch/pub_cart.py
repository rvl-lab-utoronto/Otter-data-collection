import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from util import radar_polar_to_cartesian
import cv2
from cv_bridge import CvBridge
import numpy as np

class ImageTransposeNode(Node):

    def __init__(self):
        super().__init__('image_transpose_node')
        self.bridge = CvBridge()

        # Create a subscriber
        self.subscription = self.create_subscription(
            Image,
            '/radar_data/b_scan_image',
            self.listener_callback,
            10)

        # Create a publisher
        self.publisher = self.create_publisher(
            Image,
            '/radar_image/cart',
            10)

    def listener_callback(self, msg):
        # Convert ROS Image message to OpenCV image
        cv_image = self.bridge.imgmsg_to_cv2(msg, 'passthrough')

        # Transpose the image
        azimuth = np.linspace(0, 2*np.pi, cv_image.shape[0], endpoint=False)
        transposed_image = cv2.transpose(cv_image)
        cart_image = radar_polar_to_cartesian(fft_data=cv_image, azimuths=azimuth, radar_resolution=0.29203,
                cart_resolution=0.5, cart_pixel_width=3000)

        # Convert OpenCV image back to ROS Image message
        transposed_msg = self.bridge.cv2_to_imgmsg(cart_image, 'mono8')

        # Publish the transposed image
        self.publisher.publish(transposed_msg)

def main(args=None):
    rclpy.init(args=args)

    image_transpose_node = ImageTransposeNode()

    rclpy.spin(image_transpose_node)

    # Destroy the node explicitly
    image_transpose_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

