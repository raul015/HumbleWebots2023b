import rclpy
from rclpy.node import Node
from fp_core_msgs.msg import JointPosition

class publisher_JointPos(Node):
    
    def __init__(self):
        super().__init__('cmd_joint_position')
        self.publisher = self.create_publisher(JointPosition, 'Joints_position',10)
        timer_period = 0.5
        self.timer = self.create_timer(timer_period,self.timer_callback)
        
    
    def timer_callback(self):
        
        msg = JointPosition()

        msg.name.append('motor 1')
        msg.name.append('motor 2')
        msg.name.append('motor 3')
        msg.name.append('motor 4')
        msg.name.append('motor 5')
        msg.name.append('motor 6')
        msg.name.append('motor 7')
        msg.name.append('motor 7 left ')

        msg.position.append(1.0)
        msg.position.append(1.0)
        msg.position.append(1.0)
        msg.position.append(1.0)
        msg.position.append(1.0)
        msg.position.append(1.0)
        msg.position.append(1.0)
        msg.position.append(1.0)
        

        self.publisher.publish(msg)

def main(args=None):
    
    rclpy.init(args=args)
    publisher_obj = publisher_JointPos()
    rclpy.spin(publisher_obj)
    
    publisher_obj.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()