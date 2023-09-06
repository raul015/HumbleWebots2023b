import rclpy
from rclpy.node import Node
from fp_core_msgs.msg import JointVelocity

class publisher_JointVel(Node):
    
    def __init__(self):
        super().__init__('cmd_joint_velocity')
        self.publisher = self.create_publisher(JointVelocity, 'Joints_velocity',10)
        timer_period = 0.5
        self.timer = self.create_timer(timer_period,self.timer_callback)
        
    
    def timer_callback(self):
        
        msg = JointVelocity()
        # campi del messaggio
        msg.name.append('motor 1')
        msg.name.append('motor 2')
        msg.name.append('motor 3')
        msg.name.append('motor 4')
        msg.name.append('motor 5')
        msg.name.append('motor 6')
        msg.name.append('motor 7')
        msg.name.append('motor 7 left ')
        
        msg.velocity.append(0.1)
        msg.velocity.append(0.1)
        msg.velocity.append(0.1)
        msg.velocity.append(0.1)
        msg.velocity.append(0.1)
        msg.velocity.append(0.1)
        msg.velocity.append(0.1)
        msg.velocity.append(0.1)
        
        
        self.publisher.publish(msg)

def main(args=None):
    
    rclpy.init(args=args)
    publisher_obj = publisher_JointVel()
    
    rclpy.spin(publisher_obj)
    publisher_obj.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()