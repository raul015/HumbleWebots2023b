import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Range
from geometry_msgs.msg import Twist

MAX_RANGE = 0.5

class ObstacleAvoider(Node):
    
    
    # Mezzo e sinistra 
    
    def __init__(self):
        
        super().__init__('tiago_obstacle_avoidance')

        self.__publisher = self.create_publisher(Twist, 'tiago_cmd_vel', 1)

        self.create_subscription(Range, 'left_sensor', self.__left_sensor_callback, 1)
        
        self.create_subscription(Range, 'right_sensor', self.__right_sensor_callback, 1)
        
        self.create_subscription(Range, 'middle_sensor', self.__middle_sensor_callback, 1)
        
        self.left_libero = True
        self.right_libero = True
        self.middle_libero = True
        
        
    def __left_sensor_callback(self, message):
        
        self.__left_sensor_value = message.range
        self.get_logger().info('sensore di sinistra : ' + str(self.__left_sensor_value))
        
        if(self.__left_sensor_value <  MAX_RANGE):
            
            # command_message = Twist()
            
            self.get_logger().info('Il sensore di sinistra ha rilevato qualcosa')
            
            # command_message.linear.x = 0.0
            # command_message.angular.z = 1.0
            
            # self.__publisher.publish(command_message)

            self.left_libero = False
            
        else:
            self.left_libero = True
        
        
    def __right_sensor_callback(self, message):
        
        
        self.__right_sensor_value = message.range
        self.get_logger().info('sensore di destra : ' + str(self.__right_sensor_value))
        
        if(self.__right_sensor_value < MAX_RANGE):
            
            # command_message = Twist()
            self.get_logger().info('Il sensore di destra ha rilevato qualcosa')
            # command_message.linear.x = 0.0
            # command_message.angular.z = -1.0
            
            # self.__publisher.publish(command_message)

            self.right_libero = False
            
        else:
            self.right_libero = True
        
        
    def __middle_sensor_callback(self, message):
        
        command_message = Twist()
        
        self.__middle_sensor_value = message.range
        self.get_logger().info('sensore di mezzo : ' + str(self.__middle_sensor_value))
        
        # Fino a qua stampa i valori del messaggi rilevati dai sensori
        
        if( (self.__middle_sensor_value < 0.6*MAX_RANGE)):
            
            command_message.linear.x = 0.0
            
            if(self.left_libero == False and self.right_libero == True):
                
                command_message.angular.z = 1.0
                self.__publisher.publish(command_message)
            
            elif(self.left_libero == True and self.right_libero == False):
                
                command_message.angular.z = -1.0
                self.__publisher.publish(command_message)
                
            elif(self.left_libero == False and self.right_libero == False):
                
                command_message.angular.z = -1.0
                self.__publisher.publish(command_message)
                
            
            elif(self.left_libero == True and self.right_libero == True):
                
                command_message.angular.z = -1.0
                self.__publisher.publish(command_message)
                
                
            self.middle_libero = False
                        
            self.get_logger().info('Il sensore centrale ha rilevato qualcosa')

        else:
                        
            if(self.left_libero == True and self.right_libero == True):
            
                command_message.linear.x = -1.0
                command_message.linear.z = 0.0
                self.__publisher.publish(command_message)


                
            elif(self.left_libero == False and self.right_libero == True):
                
                command_message.linear.x = 0.0
                command_message.angular.z = 1.0
                self.__publisher.publish(command_message)

                
            elif(self.left_libero == True and self.right_libero == False):
                
                command_message.linear.x = 0.0
                command_message.angular.z = -1.0
                self.__publisher.publish(command_message)

            elif(self.left_libero == False and self.right_libero == False):
            
                command_message.linear.x = 1.0
                command_message.linear.z = -1.0
                self.__publisher.publish(command_message)

            self.middle_libero = True


        

def main(args=None):
    
    rclpy.init(args=args)
    avoider = ObstacleAvoider()
        
    rclpy.spin(avoider)
    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    

    avoider.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()