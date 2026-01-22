import rclpy
from rclpy.node import Node
import math
from sensor_msgs.msg import JointState
from nav_msgs.msg import Odometry

class JointStateListener(Node):
    def __init__(self):
        super().__init__('odometrie_node')
        delta_t = 0

        self.subscription = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            10
        )

        self.publisher_ = self.create_publisher(
            Odometry,
            '/fake_odom',
            10
        )

        self.get_logger().info('Odometrie node started')

        self.get_logger().info('Initializing ...')

        self.wheel_radius = 0.0175
        self.baseline = 0.16

        self.current_time = 0
        self.prev_time = 0

        self.prev_l_pos = 0
        self.prev_r_pos = 0

        self.prev_x = 0
        self.prev_y = 0

        self.theta = 0

    def joint_state_callback(self, msg):
        #self.get_logger().info(f'Received JointState message: {msg}')  # Log the received message

        """
        delta_pos = (delta_pos1 - delta_pos2)

        phi = w * delta_t

        x1 = x0 + v_center * delta_t
        y1 = y0 + v_center * delta_t

        v_center 
        """

        self.current_time = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        delta_t = self.current_time - self.prev_time

        current_l_pos = msg.position[0]
        current_r_pos = msg.position[1]

        # Radwinkel
        delta_pos_l = current_l_pos - self.prev_l_pos 
        delta_pos_r = current_r_pos - self.prev_r_pos

        # Radgeschwindigkeit
        omeg_l = delta_pos_l / delta_t
        omeg_r = delta_pos_r / delta_t 

        # Geschwindigkeit
        v_l = omeg_l * self.wheel_radius * 2
        v_r = omeg_r * self.wheel_radius * 2

        # Strecke
        ds_l = v_l * delta_t 
        ds_r = v_r * delta_t         

        ds_center = (ds_l + ds_r) / 2 

        phi = (ds_r - ds_l) / self.baseline

        self.prev_l_pos = current_l_pos
        self.prev_r_pos = current_r_pos

        self.theta = (self.theta + phi ) % (2 * math.pi)

        x = self.prev_x + ds_center * math.cos(self.theta)
        y = self.prev_y + ds_center * math.sin(self.theta)

        self.prev_x = x 
        self.prev_y = y 

        self.prev_time = self.current_time

        # self.get_logger().info(f'delta_t: {self.delta_t}')
        # self.get_logger().info(f'delta_pos_l: {delta_pos_l}')
        # self.get_logger().info(f'v_r: {v_r}')
        # self.get_logger().info(f'phi: {phi}')
        # self.get_logger().info(f'ds_center: {ds_center}')
        self.get_logger().info(f'theta: {self.theta}, x: {x}, y: {y} ')

        odom_out = Odometry()
        odom_out.header.stamp.sec = msg.header.stamp.sec
        odom_out.header.stamp.nanosec = msg.header.stamp.nanosec

        odom_out.pose.pose.position.x = x
        odom_out.pose.pose.position.y = y 
        self.publisher_.publish(odom_out)
        # Kreisbahn


        

def main(args=None):
    rclpy.init(args=args)
    node = JointStateListener()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()