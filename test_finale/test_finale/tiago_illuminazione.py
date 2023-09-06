import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
import time

illuminazione = Float64()


class TiagoIlluminazione(Node):
        
    def __init__(self):
        
        super().__init__('tiago_line_illuminazione')
        
        self.__publisher=  self.create_publisher(Float64, 'tiago_illuminazione',10) #   Sensore per ostacoli davanti
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

        
    def timer_callback(self):

        self.Illuminazione()
        
    def Illuminazione(self):
        
        # self.get_logger().info('chiama delle funzione Illuminazione')   
        # Implementazione logica per far variare l'illumiazione 
        
        time.sleep(2.0) # Illuminazione ferma per 5 secondi
        
        # self.get_logger().info('Illuminazione: 0.0')   

        illuminazione.data = 0.0
        self.__publisher.publish(illuminazione)

        time.sleep(2.0) # Illuminazione ferma per 5 secondi
        
        # self.get_logger().info('Illuminazione: 4.0')   

        illuminazione.data = 4.0
        self.__publisher.publish(illuminazione)
        
        time.sleep(2.0) # Illuminazione ferma per 5 secondi 
        
def main(args=None):
    
    rclpy.init(args=args)
    
    nodo_illumina = TiagoIlluminazione()
        
    rclpy.spin(nodo_illumina)
    nodo_illumina.destroy_node()
    
    rclpy.shutdown()

if __name__ == '__main__':
    main()