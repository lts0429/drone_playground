import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge


class StereoBGR2RGB(Node):
    def __init__(self):
        super().__init__('stereo_bgr2rgb')
        self.bridge = CvBridge()

        # Subscribers
        self.sub_left = self.create_subscription(
            Image, 'oak/left/image_raw', self.callback_left, 10)
        self.sub_right = self.create_subscription(
            Image, 'oak/right/image_raw', self.callback_right, 10)

        # Publishers
        self.pub_left = self.create_publisher(Image, 'oak/left/image_rgb', 10)
        self.pub_right = self.create_publisher(Image, 'oak/right/image_rgb', 10)

    def callback_left(self, msg):
        cv_img = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        msg_rgb = self.bridge.cv2_to_imgmsg(cv_img, encoding="rgb8")
        msg_rgb.header = msg.header
        self.pub_left.publish(msg_rgb)

    def callback_right(self, msg):
        cv_img = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        msg_rgb = self.bridge.cv2_to_imgmsg(cv_img, encoding="rgb8")
        msg_rgb.header = msg.header
        self.pub_right.publish(msg_rgb)


def main():
    rclpy.init()
    node = StereoBGR2RGB()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
