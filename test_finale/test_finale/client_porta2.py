import rclpy
from rclpy.node import Node
from fp_core_msgs.srv import SetFloat
from fp_core_msgs.srv import RequestData

class NodoClientPortaSecondo(Node):

    def __init__(self):
        super().__init__('client_porta_secondo')
        self.cli = self.create_client(SetFloat, 'PortaPercorso2')

        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Servizio porta non disponibile, aspetta')
        self.req = SetFloat.Request()
        
        
        self.cli2 = self.create_client(RequestData, 'abilitazioneTelecamera')
        
        while not self.cli2.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Servizio camera non disponibile, aspetta')
            
        self.req2 = RequestData.Request()
        
        
    def send_request(self,richiesta):
        
        self.req.data = richiesta
        self.future = self.cli.call_async(self.req)
        rclpy.spin_until_future_complete(self, self.future)
        return self.future.result()
    
    def send_request_camera(self):
        
        self.future = self.cli2.call_async(self.req2)
        rclpy.spin_until_future_complete(self, self.future)
        return self.future.result()
    
    
def main(args=None):
    rclpy.init(args=args)
    
    client = NodoClientPortaSecondo()
    #Mando la richiesta di 1.5 per l'apertura della seconda porta
    client.send_request(1.5)
    client.send_request_camera()
    client.destroy_node()

    rclpy.shutdown()


if __name__ == '__main__':
    main()
    