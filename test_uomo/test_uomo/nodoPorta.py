import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
import time

posizione = Float64()

class PortaSimulazione(Node):
        
    def __init__(self):
        
        super().__init__('nodo_porta')
        
        self.__publisher=  self.create_publisher(Float64, 'posizione_porta',1) #   Sensore per ostacoli davanti
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

        
    def timer_callback(self):

        self.Apertura()
        
    def Apertura(self):
        
        self.get_logger().info('chiama delle funzione per apertura e chiusura della porta')   
        # Implementazione logica per far variare l'illumiazione 
        
        time.sleep(2.0)
        self.get_logger().info('Porta: 0.0')   

        posizione.data = 0.0
        self.__publisher.publish(posizione)

        time.sleep(2.0) 
        self.get_logger().info('porta: 0.5')   

        posizione.data = 0.5
        self.__publisher.publish(posizione)
        
        time.sleep(2.0) 
        self.get_logger().info('porta: -0.5')   

        posizione.data = -0.5
        self.__publisher.publish(posizione)
                
def main(args=None):
    
    rclpy.init(args=args)
    
    nodo_porta = PortaSimulazione()
        
    rclpy.spin(nodo_porta)
    nodo_porta.destroy_node()
    
    rclpy.shutdown()

if __name__ == '__main__':
    main()