import rclpy
from rclpy.node import Node
from fp_core_msgs.srv import SetFloat

class NodoClientPorta(Node):

    def __init__(self):
        super().__init__('client_porta')
        self.cli = self.create_client(SetFloat, 'PortaPercorso1')

        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Servizio non disponibile, aspetta')
        self.req = SetFloat.Request()
        
    def send_request(self,richiesta):
        
        self.req.data = richiesta
        self.future = self.cli.call_async(self.req)
        rclpy.spin_until_future_complete(self, self.future)
        return self.future.result()
    
    
def main(args=None):
    rclpy.init(args=args)
    
    client = NodoClientPorta()
    client.send_request(-1.5)
    
    client.destroy_node()

    rclpy.shutdown()


if __name__ == '__main__':
    main()
    