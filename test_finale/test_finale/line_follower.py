import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Range
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64
# Range è un messaggio di tipo float

# Sensore sul nero valore 0.026 --- 0.046 (?)

MAX_RANGE = 0.5
MAX_SPEED = 10.2


cmd = Twist()


class LineFollower(Node):
    
    # Mezzo e sinistra 
    
    def __init__(self):
        
        super().__init__('tiago_line_follower_avoidance')
        
        self.inizio = False
        self.fine = False

        # self.create_subscription(Float64, 'tiago_ir_left', self.__left_sensor_callback, 1) # Sensore a IR
        
        # self.create_subscription(Float64, 'tiago_ir_right', self.__right_sensor_callback, 1)    # Sensore a IR
        
        # self.create_subscription(Range, 'middle_sensor', self.__middle_sensor_callback, 1) #   Sensore per ostacoli davanti
        
        
        self.create_subscription(Float64, 'tiago_ir_left', self.leftIR_cb, 1) # Sensore a IR
        
        self.create_subscription(Float64, 'tiago_ir_right', self.rightIR_cb, 1)    # Sensore a IR
        
        self.create_subscription(Range, 'tiago_middle', self.middleSensor_cb, 1) #   Sensore per ostacoli davanti
        
        self.create_subscription(Float64, 'tiago_light_sensor', self.lightSensor_cb, 1) #   Sensore per ostacoli davanti

        self.__publisher = self.create_publisher(Twist, 'tiago_cmd_vel', 1)

        self.left_libero = False
        self.right_libero = False
        self.middle_libero = False
        
        # inizializzazione
        self.GS_LEFT = 0.0 
        self.GS_RIGHT = 0.0
        self.GS_middle = 100
        self.GS_LIGHT = 256
        
        
        # self.velocita = -1.0# Parametro di velocita del robot 
        # self.velocita_angolare = 0.0
        
    def LineFollowModule(self):
        
        self.get_logger().info('chiama delle funzione LineFollowModule')  
        
        if(self.GS_middle > 0.4 or self.GS_LIGHT> 10):
            
            print('Non è stato rilevato alcuno ostacolo')

            if(self.GS_LEFT >= 500 and self.GS_RIGHT >= 500):
                print('una delle due ruote è su olio....')
                
                
                if(self.GS_LEFT < 500 and self.GS_RIGHT >=500):
                    cmd.linear.x = -0.1
                    cmd.angular.x = -0.2
                    self.__publisher.publish(cmd)

                
                elif(self.GS_LEFT >= 500 and self.GS_RIGHT < 500):
                
                    cmd.linear.x = -0.1
                    cmd.angular.x = 0.2
                    self.__publisher.publish(cmd)

                
                elif(self.GS_LEFT >= 500 and self.GS_RIGHT >= 500):
                    
                    cmd.linear.x = -0.12
                    cmd.angular.x = -0.1
                    self.__publisher.publish(cmd)

    
            else:
            
                print('fase di non slittamento')
            
            
                if(self.GS_LEFT < 100 and self.GS_RIGHT >= 100):
                    
                    cmd.linear.x = -0.1
                    cmd.angular.x = -0.2
                    self.__publisher.publish(cmd)

                    
                elif(self.GS_LEFT >= 100 and self.GS_RIGHT < 100):
                    
                    cmd.linear.x = -0.1
                    cmd.angular.x = 0.2
                    self.__publisher.publish(cmd)

                    
                elif(self.GS_LEFT >= 100 and self.GS_RIGHT >= 100):
                        
                    cmd.linear.x = -0.12
                    cmd.angular.x = -0.1
                    self.__publisher.publish(cmd)
        
                elif(self.GS_LEFT < 100 and self.GS_RIGHT < 100):
                        
                    cmd.linear.x = 0.0
                    cmd.angular.x = 0.0
                    self.__publisher.publish(cmd)
        else:
            
            if(self.GS_LIGHT <= 10):
                print('illuminazione non va bene per segui linea ')
                
            if(self.GS_middle <= 0.4):
                print('trovato un ostacolo')
                
            cmd.linear.x = 0.0
            cmd.angular.x = 0.0
            self.__publisher.publish(cmd)

    # Sono 3 che leggono i valori dei sensori, non serve richiarmarli esplicitamente
           
    def rightIR_cb(self,msg):
        
        print('destro: ' + str(msg.data))
        self.GS_RIGHT = msg.data # Non prendo oggetto Float ma oggetto.data
        
    def leftIR_cb(self,msg):
        
        print('sinistro: ' + str(msg.data))
        
        self.GS_LEFT = msg.data # Non prendo oggetto Float ma oggetto.data
        
    def lightSensor_cb(self,msg):
        
        print('illuminazione: ' + str(msg.data))
        self.GS_LIGHT = msg.data
                
    def middleSensor_cb(self,msg):
        print('centrale: ' + str(msg.range))

        self.GS_middle = msg.range # Non prendo oggetto Range ma oggetto.range
        print('range: ' + str(self.GS_middle))
        self.LineFollowModule()

        self.fine = True
 
        
    #     if(self.GS_middle>0.2):
            
    #         print('Non è stato rilevato alcuno ostacolo')

    #         if(self.GS_LEFT < 100 and self.GS_RIGHT >= 100):
                
    #             cmd.linear.x = -0.1
    #             cmd.angular.x = -0.2
    #             self.__publisher.publish(cmd)

                
    #         elif(self.GS_LEFT >= 100 and self.GS_RIGHT < 100):
                
    #             cmd.linear.x = -0.1
    #             cmd.angular.x = 0.2
    #             self.__publisher.publish(cmd)

                
    #         elif(self.GS_LEFT >= 100 and self.GS_RIGHT >= 100):
                
    #             cmd.linear.x = -0.12
    #             cmd.angular.x = -0.1
    #             self.__publisher.publish(cmd)

                
    #         elif(self.GS_LEFT < 100 and self.GS_RIGHT < 100): 
                
    #             cmd.linear.x = 0.0
    #             cmd.angular.x = 0.0
    #             self.__publisher.publish(cmd)
  
    #     else:
            
            
    #         if(self.GS_middle <= 0.2):
                
    #             self.get_logger().info('chiama delle funzione per apertura e chiusura della porta')   

                
    #         cmd.linear.x = -0.0
    #         cmd.angular.x = 0.0
    #         self.__publisher.publish(cmd)
    
    # # Sono 3 che leggono i valori dei sensori, non serve richiarmarli esplicitamente
           
    def rightIR_cb(self,msg):
        
        print('aggiornamento destro: ' + str(msg.data))
        self.GS_RIGHT = msg.data # Non prendo oggetto Float ma oggetto.data
        
    def leftIR_cb(self,msg):
        
        print('aggiornamento sinistro: ' + str(msg.data))
        
        self.GS_LEFT = msg.data # Non prendo oggetto Float ma oggetto.data
        
    def middleSensor_cb(self,msg):
        print('aggiornamento centrale: ' + str(msg.range))

        self.GS_middle = msg.range # Non prendo oggetto Range ma oggetto.range
        print('range: ' + str(self.GS_middle))
        self.LineFollowModule()

        self.fine = True
        

def main(args=None):
    
    rclpy.init(args=args)
    follower = LineFollower()
        
    rclpy.spin(follower)
    


    follower.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()