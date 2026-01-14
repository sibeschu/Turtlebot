#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

import numpy as np
import cv2

from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge

from message_filters import Subscriber, ApproximateTimeSynchronizer


class FisheyeRectifyNode(Node):
    def __init__(self):
        super().__init__('fisheye_rectify_node')

        self.declare_parameter('in_image', '/camera/image_raw')
        self.declare_parameter('in_info', '/camera/camera_info')
        self.declare_parameter('out_image', '/camera/rectified_image')
        self.declare_parameter('output_width', 0)
        self.declare_parameter('output_height', 0)
        self.declare_parameter('balance', 0.0)
        self.declare_parameter('fov_scale', 1.0)
        self.declare_parameter('interpolation', 'linear')  # linear|nearest

        in_image = self.get_parameter('in_image').value
        in_info = self.get_parameter('in_info').value
        out_image = self.get_parameter('out_image').value

        self.output_width = int(self.get_parameter('output_width').value)
        self.output_height = int(self.get_parameter('output_height').value)
        self.balance = float(self.get_parameter('balance').value)
        self.fov_scale = float(self.get_parameter('fov_scale').value)
        interp = self.get_parameter('interpolation').value
        self._interp_flag = cv2.INTER_LINEAR if interp == 'linear' else cv2.INTER_NEAREST

        self.bridge = CvBridge()
        self.pub = self.create_publisher(Image, out_image, 10)

        self.sub_img = Subscriber(self, Image, in_image)
        self.sub_info = Subscriber(self, CameraInfo, in_info)
        self.sync = ApproximateTimeSynchronizer([self.sub_img, self.sub_info], queue_size=10, slop=0.1)
        self.sync.registerCallback(self.cb)

        self._map1 = None
        self._map2 = None
        self._last_key = None

        self.get_logger().info("Fisheye rectify node running.")
        self.get_logger().info(f" Subscribing: {in_image} + {in_info}")
        self.get_logger().info(f" Publishing:  {out_image}")

    def _make_maps(self, cam_info: CameraInfo, img_w: int, img_h: int):
        K = np.array(cam_info.k, dtype=np.float64).reshape(3, 3)

        d_list = list(cam_info.d)
        if len(d_list) < 4:
            raise RuntimeError(f"CameraInfo.d has only {len(d_list)} elements, need >=4 for fisheye.")
        D = np.array(d_list[:4], dtype=np.float64).reshape(4, 1)

        out_w = self.output_width if self.output_width > 0 else img_w
        out_h = self.output_height if self.output_height > 0 else img_h
        DIM = (img_w, img_h)
        NEW_DIM = (out_w, out_h)

        R = np.eye(3, dtype=np.float64)

        newK = cv2.fisheye.estimateNewCameraMatrixForUndistortRectify(
            K, D, DIM, R, balance=self.balance, new_size=NEW_DIM, fov_scale=self.fov_scale
        )

        map1, map2 = cv2.fisheye.initUndistortRectifyMap(
            K, D, R, newK, NEW_DIM, m1type=cv2.CV_16SC2
        )

        return map1, map2, NEW_DIM

    def cb(self, img_msg: Image, info_msg: CameraInfo):
        try:
            cv_img = self.bridge.imgmsg_to_cv2(img_msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f"cv_bridge conversion failed: {e}")
            return

        h, w = cv_img.shape[:2]

        K_key = tuple(np.array(info_msg.k, dtype=np.float64).round(10))
        D_key = tuple(np.array(list(info_msg.d)[:4], dtype=np.float64).round(10))
        out_w = self.output_width if self.output_width > 0 else w
        out_h = self.output_height if self.output_height > 0 else h
        key = (w, h, out_w, out_h, self.balance, self.fov_scale, K_key, D_key)

        try:
            if self._last_key != key or self._map1 is None or self._map2 is None:
                self._map1, self._map2, new_dim = self._make_maps(info_msg, w, h)
                self._last_key = key
                self.get_logger().info(
                    f"Built maps: in={w}x{h} out={new_dim[0]}x{new_dim[1]} balance={self.balance} fov_scale={self.fov_scale}"
                )

            rect = cv2.remap(cv_img, self._map1, self._map2,
                             interpolation=self._interp_flag, borderMode=cv2.BORDER_CONSTANT)

            out = self.bridge.cv2_to_imgmsg(rect, encoding='bgr8')
            out.header = img_msg.header
            self.pub.publish(out)

        except Exception as e:
            self.get_logger().error(f"Rectification failed: {e}")


def main():
    rclpy.init()
    node = FisheyeRectifyNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
