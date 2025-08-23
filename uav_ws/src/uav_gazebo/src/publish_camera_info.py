#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo

class FixRightCameraInfo(Node):
    def __init__(self):
        super().__init__('fix_right_camera_info')
        self.sub = self.create_subscription(CameraInfo, 'camera_left/info_in',
                                            self.callback, 10)
        self.pub = self.create_publisher(CameraInfo, 'camera_left/info', 10)

        self.baseline = 0.05

    def callback(self, msg: CameraInfo):
        # Compute Tx
        Tx = -msg.p[0] * self.baseline

        # Adjust projection matrix
        msg.p[3] = Tx

        self.pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = FixRightCameraInfo()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
