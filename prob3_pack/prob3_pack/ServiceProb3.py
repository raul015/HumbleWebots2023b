import sys # utilizziamo gli argomenti del terminale tramite questa libreria 
import rclpy
from rclpy.node import Node
from fp_core_msgs.srv import MoveJoint

class ServiceProb3(Node):
    def __init__(self):
        super().__init__('ServiceProb3')
        self.prob3client = self.create_client(MoveJoint, 
                                              'Movimento')
        
        while not self.prob3client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('servizio non disponibile, aspetta ancora ....')
        self.value = MoveJoint.Request()
        
    def client_request(self):
        
        #Valori in cui verranno salvati i valori inseriti da terminale...
        
        self.value.actuator_ids = eval((sys.argv[1]))
        self.value.position = eval((sys.argv[2]))
        self.value.velocity = eval((sys.argv[3]))
        self.value.acceleration = eval((sys.argv[4]))
        self.value.block = eval((sys.argv[5]))
        self.value.relative= eval((sys.argv[6]))

        self.response = self.prob3client.call_async(self.value)
        
def main(args=None):
    
    rclpy.init(args=args)
    client_prob3 = ServiceProb3()
    client_prob3.client_request()
    while rclpy.ok():
        
        rclpy.spin_once(client_prob3)
        if(client_prob3.response.done()):
            try:
                response = client_prob3.response.result()
            except Exception as e:
                client_prob3.get_logger().info('Chiamata servizio fallita %r' %(e,))
                
            else:
                client_prob3._logger().info('Chiamata servizio riuscita %s %s %s %s %s %s' %
                (client_prob3.value.actuator_ids, client_prob3.value.position ,
                 client_prob3.value.velocity, client_prob3.value.acceleration,
                 client_prob3.value.block, client_prob3.value.relative))
            break
        
    client_prob3.destroy_node()
    rclpy.shutdown()
        
    if __name__ == '__main__':
        main()
        
        