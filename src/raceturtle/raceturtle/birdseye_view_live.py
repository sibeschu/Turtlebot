import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np

class PerspectiveNode(Node):
    def __init__(self):
        super().__init__('perspective_node')
        self.bridge = CvBridge()
        self.sub = self.create_subscription(
            Image,
            '/camera/image_rect_color',
            self.image_callback,
            10
        )
        self.pub = self.create_publisher(
            Image,
            '/birdseye_view',
            10
        )

    def image_callback(self, msg):
        #define source and destination vertices for transform (to be adjusted based on camera view)
        src = np.float32([
            [493, 443],  # top-left
            [696, 440],  # top-right
            [865, 646],  # bottom-right
            [310, 646],  # bottom-left
        ])

        dst = np.float32([
            # [310, 443],  # move top-left x to bottom-left x
            # [865, 440],  # move top-right x to bottom-right x
            # [865, 648],  # keep bottom-right fixed
            # [310, 648 ],  # keep bottom-left fixed
            [510, 400],
            [642, 400],
            [642, 648],
            [510, 648]
        ])

        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

        M = cv2.getPerspectiveTransform(src, dst)

        warped = cv2.warpPerspective(frame, M, (1152, 648), cv2.INTER_LINEAR, cv2.BORDER_CONSTANT)

        cv2.imshow("Warped View", warped)
        cv2.waitKey(1)

def main():
    rclpy.init()
    node = PerspectiveNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
