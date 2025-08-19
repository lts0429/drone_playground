import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import numpy as np
import cv2

class DepthToRGBNode(Node):
    def __init__(self):
        super().__init__('depth_to_rgb_node')
        self.bridge = CvBridge()
        self.rgb_image = None
        self.depth_image = None
        self.subscription_depth = self.create_subscription(
            Image,
            'rgbd/depth',  # Depth image topic
            self.depth_callback,
            10
        )
        self.subscription_rgb = self.create_subscription(
            Image,
            '/rgbd/image',  # RGB image topic
            self.rgb_callback,
            10
        )
        self.publisher = self.create_publisher(
            Image,
            'rgbd/depth_overlay',  # Output topic for overlay
            10
        )
        self.get_logger().info('DepthToRGBNode started.')

    def rgb_callback(self, msg):
        try:
            self.rgb_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f'Error converting RGB image: {e}')

    def depth_callback(self, msg):
        try:
            depth_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
            min_depth = 0.01
            max_depth = 100.0
            depth_clipped = np.clip(depth_image, min_depth, max_depth)
            depth_normalized = ((depth_clipped - min_depth) / (max_depth - min_depth) * 255.0).astype(np.uint8)
            depth_eq = cv2.equalizeHist(depth_normalized)
            depth_rgb = cv2.applyColorMap(depth_eq, cv2.COLORMAP_JET)
            self.depth_image = depth_rgb
            # Overlay if RGB image is available
            if self.rgb_image is not None and self.rgb_image.shape[:2] == depth_rgb.shape[:2]:
                overlay = cv2.addWeighted(self.rgb_image, 0.9, depth_rgb, 0.1, 0)
                rgb_msg = self.bridge.cv2_to_imgmsg(overlay, encoding='bgr8')
                rgb_msg.header = msg.header
                self.publisher.publish(rgb_msg)
        except Exception as e:
            self.get_logger().error(f'Error converting depth to RGB or overlaying: {e}')

def main(args=None):
    rclpy.init(args=args)
    node = DepthToRGBNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
