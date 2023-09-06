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
        
        self.create_subscription(Range, 'middle_sensor', self.middleSensor_cb, 1) #   Sensore per ostacoli davanti
        
        
        self.__publisher = self.create_publisher(Twist, 'tiago_cmd_vel', 1)

        self.__right_sensor_value = 8.0 # inizializzazione 
        self.__left_sensor_value  = 8.0 # inizializzazion
        
        self.left_libero = False
        self.right_libero = False
        self.middle_libero = False
        
        # inizializzazione
        self.GS_LEFT = 0.0 
        self.GS_RIGHT = 0.0
        self.GS_middle = 100
        
        
        # self.velocita = -1.0# Parametro di velocita del robot 
        # self.velocita_angolare = 0.0
        
    def LineFollowModule(self):
        
        #Da inserire per far muovere il robot se le condizioni sono rispettate
        
        # self.cmd.linear.x = self.velocita
        # self.cmd.angular.x = self.velocita_angolare
        
        self.get_logger().info('chiama delle funzione LineFollowModule')   
        
        print('valore sensore SINISTRO: ' + str(self.__left_sensor_value))
        print('valore sensore DESTRO: ' + str(self.__right_sensor_value))
        
        if(self.GS_middle>0.1):
            
            print('Non è stato rilevato alcuno ostacolo')

            if(self.GS_LEFT < 5 and self.GS_RIGHT >5):
                
                print('sensore sinistro su NERO ')
                print('sensore destro su BIANCO')
                
                # cmd.linear.x = -0.5
                # cmd.angular.x = -1.0
                cmd.linear.x = -0.1
                cmd.angular.x = -0.25
                self.__publisher.publish(cmd)

                
            elif(self.GS_LEFT > 5 and self.GS_RIGHT <5):
                
                print('sensore sinistro su BIANCO')
                print('sensore destro su NERO')

                # cmd.linear.x = -0.5
                # cmd.angular.x = 1.0
                cmd.linear.x = -0.1
                cmd.angular.x = 0.25
                self.__publisher.publish(cmd)

                
            elif(self.GS_LEFT > 5 and self.GS_RIGHT >5):
                
                
                print('sensore sinistro su BIANCO')
                print('sensore destro su BIANCO')
                
                # Se sono sul bianco vado sempre dritto
                # cmd.linear.x = -0.5
                # cmd.angular.x = 0.0
                cmd.linear.x = -0.1
                cmd.angular.x = 0.0
                self.__publisher.publish(cmd)

                
            elif(self.GS_LEFT< 5 and self.GS_RIGHT <5): 
                
                # Condizione per fermarsi, sono arrivato alla fine del percorso
                print('sensore sinistro su NERO')
                print('sensore destro su NERO ')
                
                # Se sono sul nero mi fermo
                cmd.linear.x = 0.0
                cmd.angular.x = 0.0
                self.__publisher.publish(cmd)
  
        else:
            
            print('è stato rilevato un ostacolo')
            
            #FERMO I MOTORI PER SICUREZZA COME PRIMO APPROCCIO
            
            cmd.linear.x = -0.0
            cmd.angular.x = 0.0
            self.__publisher.publish(cmd)
    
    # Sono 3 che leggono i valori dei sensori, non serve richiarmarli esplicitamente
           
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

    
    # def __left_sensor_callback(self, message):
        
    #     self.__left_sensor_value = message.data    
    #     self.get_logger().info('inizio sensore sinistro : ' + str(message.data))
            
    #     if(self.__left_sensor_value < 5 ):

    #         self.left_libero = False   #  Sensore sinistro sulla linea nera 
            
    #     else:
    #         self.left_libero = True  # Sensore sinistro sul pavimento bianco        
        
    # def __right_sensor_callback(self, message):
        
    #     self.__right_sensor_value = message.data
    #     self.get_logger().info('inizio sensore destro : ' + str(message.data))

    #     if(self.__right_sensor_value < 5):
            
    #         self.right_libero  = False # Sensore destro sula linea nera 
            
    #     else:
    #         self.right_libero  = True   # Sensore destro sul pavimento bianco
             
    # def __middle_sensor_callback(self, message):
        
    #     #   range soltanto per il valore centrale
        
    #     command_message = Twist()
        
    #     self.__middle_sensor_value = message.range # Da utilizzare poi per individuare ostacoli durante il percorso...
        
    #     if(self.inizio == True):
            
    #         self.get_logger().info('Inizio = True')
            
    #         if(self.left_libero == False and self.right_libero == False): # Inizialemnte sta sul nero quindi vado dritto
                
    #             self.get_logger().info('inizio entrambi i sensori sulla riga nera')

    #             command_message.linear.x = -2.0
    #             command_message.angular.z = -2.0
    #             self.__publisher.publish(command_message)

    #         else:
    #            self.inizio = False   
               
    #     else:
            
    #         self.get_logger().info('Inizio = False')

    #         if( self.left_libero == True and self.right_libero == True and self.fine == False):
                
    #             # Se sono sul bianco vado sempre dritto
                
    #             command_message.linear.x = -0.5
    #             command_message.angular.z = 0.0
    #             self.__publisher.publish(command_message)

            
    #         elif(self.left_libero == True and self.right_libero == False and self.fine == False):
                
    #             command_message.linear.x = -0.5
    #             command_message.angular.z = 1.0
    #             self.__publisher.publish(command_message)

            
    #         elif(self.left_libero == False and self.right_libero == True and self.fine == False):
                
    #             command_message.linear.x = -0.5
    #             command_message.angular.z = -1.0
    #             self.__publisher.publish(command_message)

                
    #         elif(self.left_libero == False and self.right_libero == False and self.fine == False):
    #             command_message.linear.x = 0.0
    #             command_message.angular.z = 0.0
    #             self.__publisher.publish(command_message)
    #             self.fine = True

def main(args=None):
    
    rclpy.init(args=args)
    follower = LineFollower()
        
    rclpy.spin(follower)
    
    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    #bianco left 632
    #bianco right 632

    follower.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()