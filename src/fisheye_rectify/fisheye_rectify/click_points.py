#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

class ClickPoints(Node):
    def __init__(self):
        super().__init__('click_points')
        self.bridge = CvBridge()
        self.sub = self.create_subscription(Image, '/camera/rectified_image', self.cb, 10)
        self.last = None
        self.points = []
        cv2.namedWindow("rectified (click 4 points)", cv2.WINDOW_NORMAL)
        cv2.setMouseCallback("rectified (click 4 points)", self.on_mouse)
        self.get_logger().info("Click 4 points in the OpenCV window. Press 'r' to reset, 'q' to quit.")

    def cb(self, msg: Image):
        self.last = msg

    def on_mouse(self, event, x, y, flags, param=None):
        if event == cv2.EVENT_LBUTTONDOWN:
            self.points.append((x, y))
            print(f"POINT {len(self.points)}: [{x}, {y}]")
            if len(self.points) == 4:
                print("\nSRC = np.float32([")
                for (px, py) in self.points:
                    print(f"  [{px}, {py}],")
                print("])\n")

    def spin_once(self):
        if self.last is None:
            cv2.waitKey(1)
            return True

        img = self.bridge.imgmsg_to_cv2(self.last, desired_encoding='bgr8').copy()

        # draw points
        for i, (x, y) in enumerate(self.points):
            cv2.circle(img, (x, y), 6, (0, 255, 0), -1)
            cv2.putText(img, str(i+1), (x+8, y-8), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)

        cv2.imshow("rectified (click 4 points)", img)
        key = cv2.waitKey(1) & 0xFF
        if key == ord('r'):
            self.points = []
            print("RESET points.")
        if key == ord('q'):
            return False
        return True

def main():
    rclpy.init()
    node = ClickPoints()
    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.05)
            if not node.spin_once():
                break
    finally:
        cv2.destroyAllWindows()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
