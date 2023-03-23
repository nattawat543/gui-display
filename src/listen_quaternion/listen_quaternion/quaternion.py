import rclpy
from rclpy.node import Node
from std_msgs.msg import *
from sensor_msgs.msg import Imu
from tf_transformations import *
import math





class IMUDRIVER_Subscriber(Node):

    def __init__(self):
        super().__init__('minimal_subscriber')
        self.subscription = self.create_subscription(
            Imu,
            "/imu/data",
            self.listener_callback,
            10)

    def listener_callback(self, msg):
        orientation_q = msg.orientation
        orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
        (r, p, y) = euler_from_quaternion(orientation_list)
        (self.roll, self.pitch, self.yaw) =(r*(180/math.pi), p*(180/math.pi), y*(180/math.pi))
        self.get_logger().info(f'I heard: {self.roll:.2f}, {self.pitch:.2f}, {self.yaw:.2f}')



def main(args=None):
    rclpy.init(args=args)
    subscriber = IMUDRIVER_Subscriber()
    rclpy.spin(subscriber)
    IMUDRIVER_Subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
