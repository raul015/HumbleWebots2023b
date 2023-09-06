import rclpy
from rclpy.node import Node
from fp_core_msgs.msg import JointPosition
import time

class Subscriber_JointPosition(Node):

    def __init__(self):
        super().__init__('subscriber_sensors_positions')
        self.subscription = self.create_subscription(
            JointPosition,
            'Sensors_values',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
        for i in range(8):
            self.get_logger().info(str(msg.name[i] +": " + str(msg.position[i])))
            
        self.get_logger().info("")

        time.sleep(0.5)


def main(args=None):
    rclpy.init(args=args)

    minimal_subscriber = Subscriber_JointPosition()

    rclpy.spin(minimal_subscriber)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()