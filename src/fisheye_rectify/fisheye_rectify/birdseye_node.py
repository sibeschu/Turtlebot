#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np

class BirdseyeNode(Node):
    def __init__(self):
        super().__init__('birdseye_node')
        self.bridge = CvBridge()

        self.sub = self.create_subscription(
            Image, '/camera/rectified_image', self.cb, 10)
        self.pub = self.create_publisher(
            Image, '/camera/birdseye_image', 10)

        # >>>>> HIER DEINE 4 PUNKTE EINTRAGEN (aus click_points Terminal) <<<<<
        self.src = np.float32([
            [36, 34],   # Punkt 1 (oben-links) ersetzen
            [976, 32],   # Punkt 2 (oben-rechts) ersetzen
            [985, 555],   # Punkt 3 (unten-rechts) ersetzen
            [29, 558],   # Punkt 4 (unten-links) ersetzen
        ])

        self.out_w = 600
        self.out_h = 800

        self.dst = np.float32([
            [0, 0],
            [self.out_w - 1, 0],
            [self.out_w - 1, self.out_h - 1],
            [0, self.out_h - 1],
        ])

        self.H = cv2.getPerspectiveTransform(self.src, self.dst)
        self.get_logger().info("Birdseye node started, publishing /camera/birdseye_image")

    def cb(self, msg: Image):
        img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

        bird = cv2.warpPerspective(img, self.H, (self.out_w, self.out_h))

        # Ungültige Bereiche grau machen
        mask_src = np.ones((img.shape[0], img.shape[1]), dtype=np.uint8) * 255
        mask_warp = cv2.warpPerspective(mask_src, self.H, (self.out_w, self.out_h))
        bird[mask_warp == 0] = (128, 128, 128)

        out = self.bridge.cv2_to_imgmsg(bird, encoding='bgr8')
        out.header = msg.header
        self.pub.publish(out)

def main():
    rclpy.init()
    node = BirdseyeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
