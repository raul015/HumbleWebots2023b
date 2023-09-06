import rclpy
from rclpy.node import Node
from fp_core_msgs.msg import RotationRobot
import time

class Subscriber_RobotRotation(Node):

    def __init__(self):
        super().__init__('subscriber_RotationRobot')
        self.subscription = self.create_subscription(
            RotationRobot,
            'tiago_orientazione_assoluta',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):

        self.get_logger().info(str(msg.x[0]))
        self.get_logger().info(str(msg.x[1]))
        self.get_logger().info(str(msg.x[2]))


        self.get_logger().info(str(msg.y[0]))
        self.get_logger().info(str(msg.y[1]))
        self.get_logger().info(str(msg.y[2]))


        self.get_logger().info(str(msg.z[0]))
        self.get_logger().info(str(msg.z[1]))
        self.get_logger().info(str(msg.z[2]))


        self.get_logger().info("")

        time.sleep(0.5) # Posso anche toglierlo 


def main(args=None):
    rclpy.init(args=args)

    subscriber = Subscriber_RobotRotation()
    rclpy.spin(subscriber)

    subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()