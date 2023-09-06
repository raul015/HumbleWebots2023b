import rclpy
from rclpy.node import Node
from fp_core_msgs.msg import JointPosition
from fp_core_msgs.msg import PositionRobot
import time

class Subscriber_RobotPosition(Node):

    def __init__(self):
        super().__init__('subscriber_PositionRobot')
        self.subscription = self.create_subscription(
            PositionRobot,
            'tiago_posizione_assoluta',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):

        self.get_logger().info(str(msg.x))
        self.get_logger().info(str(msg.y))
        self.get_logger().info(str(msg.z))
        self.get_logger().info("")

        time.sleep(0.5) # Posso anche toglierlo 


def main(args=None):
    rclpy.init(args=args)

    subscriber = Subscriber_RobotPosition()
    rclpy.spin(subscriber)

    subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()