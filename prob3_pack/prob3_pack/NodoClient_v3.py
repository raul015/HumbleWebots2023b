import rclpy
from rclpy.node import Node
from fp_core_msgs.srv import MoveJoint
import time

class NodoClient(Node):

    def __init__(self):
        super().__init__('nodo_cliente')
        self.cli = self.create_client(MoveJoint, 'MoveJoint')

        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Servizio non disponibile, aspetta')
        self.req = MoveJoint.Request()
        
    def send_request(self,Oggetto_movejoint):
        
        self.req.actuator_ids = Oggetto_movejoint.actuator_ids
        self.req.position = Oggetto_movejoint.position
        self.req.velocity = Oggetto_movejoint.velocity
        
        if(Oggetto_movejoint.acceleration is None):
            self.req.acceleration = 2.0
        else:
            self.acceleration = Oggetto_movejoint.acceleration
            
        if(Oggetto_movejoint.block is None):
            self.req.block = Oggetto_movejoint.block
        else:
            self.req.block = Oggetto_movejoint.block
            
        if(Oggetto_movejoint.relative is None):
            self.req.relative = Oggetto_movejoint.relative
        else:
            self.req.relative = Oggetto_movejoint.relative
        self.future = self.cli.call_async(self.req)
        rclpy.spin_until_future_complete(self, self.future)
        return self.future.result()
    
    def build_request(self,actuatore_ids,position,velocity,acceleration = None,block = False,relative = False):
        
        Object_Movejoint = MoveJoint_class(actuatore_ids,position,velocity,acceleration,block,relative)
        
        return  Object_Movejoint
    
    def send_request_MoveJoint(self,Class_MoveJoint): # IL self è necessario per chiamare la funzione dentro la classe NodoClient
                    
        ritorno = self.send_request(Class_MoveJoint)
        
        return ritorno


class MoveJoint_class():
    
    def __init__(self,actuator_ids,position,velocity,acceleration = None,block = False,relative= False):
        
        self.actuator_ids = actuator_ids
        self.position = position
        self.velocity = velocity
        if(acceleration is None):
            self.acceleration = 2.0
        else:
            self.acceleration = acceleration
            
        if(block is False):
            self.block = block
        else:
            self.block = block
            
        if(relative is False):
            self.relative = relative
        else:
            self.relative = relative
    
def main(args=None):
    rclpy.init(args=args)
    vettore_richieste = []
    
    client = NodoClient()
    
    
    #   Richiesta minima 
    
    vettore_richieste.append(client.build_request([1,2,3,4,5,6,7,8] , [1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0], 0.2))
    vettore_richieste.append(client.build_request([1,2,3,4,5,6,7,8] , [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0], 0.2))
    vettore_richieste.append(client.build_request([1,2,3,4,5,6,7,8] , [0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5], 0.2))
    
    vettore_richieste.append(client.build_request([1,2,3,4,5,6,7,8] , [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0], 0.1))
    
    vettore_richieste.append(client.build_request([1,2,3,4,5,6,7,8] , [-1.0, -1.0, -1.0, -1.0, -1.0, -1.0, 1.0, 1.0], 0.2))
    vettore_richieste.append(client.build_request([1,2,3,4,5,6,7,8] , [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0], 0.2))
    vettore_richieste.append(client.build_request([1,2,3,4,5,6,7,8] , [-0.5, -0.5, -0.5, -0.5, -0.5, -0.5, 0.5, 0.5], 0.3))
    
    
    # Posizioni date in coordinate relative 
    
    vettore_richieste.append(client.build_request([1,2,3,4,5,6,7,8] , [0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1], 0.2, acceleration = 2.0, block=False, relative= True))
    vettore_richieste.append(client.build_request([1,2,3,4,5,6,7,8] , [0.2, 0.2, 0.2, 0.2, 0.2, 0.2, 0.2, 0.2], 0.2, acceleration = 2.0, block=False, relative= True))
    vettore_richieste.append(client.build_request([1,2,3,4,5,6,7,8] , [-0.1, -0.1, -0.1, -0.1, -0.1, -0.1, 0.1, 0.1], 0.2, acceleration = 2.0, block=False, relative= True))
    vettore_richieste.append(client.build_request([1,2,3,4,5,6,7,8] , [-0.2, -0.2, -0.2, -0.2, -0.2, -0.2, -0.1, -0.1], 0.2, acceleration = 2.0, block=False, relative= True))
    
    
    # Di nuovo coordinate assolute
    
    vettore_richieste.append(client.build_request([1,2,3,4,5,6,7,8] , [1.0, 0.1, 1.2, 1.5, 2.0, 0.5, 0.2, 0.2], 0.2))
    vettore_richieste.append(client.build_request([1,2,3,4,5,6,7,8] , [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0], 0.2))
    vettore_richieste.append(client.build_request([1,2,3,4,5,6,7,8] , [0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5], 0.3))
    
    vettore_richieste.append(client.build_request([1,2,3,4,5,6,7,8] , [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0], 0.3))

    # Vettore richieste contiene oggetti MoveJoint

    for i in range(len(vettore_richieste)):
        
        ritorno = client.send_request_MoveJoint(vettore_richieste[i])  
        client.get_logger().info('messaggio : ' + str(ritorno.message))
        client.get_logger().info('success :'+str(ritorno.success))
        if(ritorno.message == "error"):
            client.get_logger().info('errore in input')
            break
        time.sleep(3.0)


    client.destroy_node()

    rclpy.shutdown()


if __name__ == '__main__':
    main()
    
    
       # Build delle richieste da mandare al robot oggetti MoveJoint , POSIZIONE ASSOLUTE 
    
    # vettore_richieste.append(MoveJoint_class([1,2,3,4,5,6,7,8] , [1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0], 0.2, acceleration = 2.0, block=False, relative= False))
    # vettore_richieste.append(MoveJoint_class([1,2,3,4,5,6,7,8] , [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0], 0.2, acceleration = 2.0, block=False, relative= False))
    # vettore_richieste.append(MoveJoint_class([1,2,3,4,5,6,7,8] , [0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5], 0.3, acceleration = 2.0, block=False, relative= False))
    
    # vettore_richieste.append(MoveJoint_class([1,2,3,4,5,6,7,8] , [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0], 0.3, acceleration = 2.0, block=False, relative= False))
    
    # vettore_richieste.append(MoveJoint_class([1,2,3,4,5,6,7,8] , [-1.0, -1.0, -1.0, -1.0, -1.0, -1.0, 0.5, 0.5], 0.2, acceleration = 2.0, block=False, relative= False))
    # vettore_richieste.append(MoveJoint_class([1,2,3,4,5,6,7,8] , [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0], 0.2, acceleration = 2.0, block=False, relative= False))
    # vettore_richieste.append(MoveJoint_class([1,2,3,4,5,6,7,8] , [-0.5, -0.5, -0.5, -0.5, -0.5, 0.5, 0.5, 0.5], 0.3, acceleration = 2.0, block=False, relative= False))

    # vettore_richieste.append(MoveJoint_class([1,2,3,4,5,6,7,8] , [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0], 0.3, acceleration = 2.0, block=False, relative= False))




    # Build delle richieste da mandare al robot oggetti MoveJoint , POSIZIONE RELATIVE
    
    
    # vettore_richieste.append(MoveJoint_class([1,2,3,4,5,6,7,8] , [0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1], 0.2, acceleration = 2.0, block=False, relative= True))
    # vettore_richieste.append(MoveJoint_class([1,2,3,4,5,6,7,8] , [0.2, 0.2, 0.2, 0.2, 0.2, 0.2, 0.2, 0.2], 0.2, acceleration = 2.0, block=False, relative= True))
    # vettore_richieste.append(MoveJoint_class([1,2,3,4,5,6,7,8] , [-0.1, -0.1, -0.1, -0.1, -0.1, -0.1, 0.1, 0.1], 0.2, acceleration = 2.0, block=False, relative= True))
    # vettore_richieste.append(MoveJoint_class([1,2,3,4,5,6,7,8] , [-0.2, -0.2, -0.2, -0.2, -0.2, -0.2, -0.1, -0.1], 0.2, acceleration = 2.0, block=False, relative= True))
    
    
    
        # Build delle richieste da mandare al robot oggetti MoveJoint , POSIZIONE RELATIVE + ASSOLUTE

    # vettore_richieste.append(MoveJoint_class([1,2,3,4,5,6,7,8] , [0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1], 0.2, acceleration = 2.0, block=False, relative= True))
    # vettore_richieste.append(MoveJoint_class([1,2,3,4,5,6,7,8] , [0.2, 0.2, 0.2, 0.2, 0.2, 0.2, 0.2, 0.2], 0.2, acceleration = 2.0, block=False, relative= True))
    
    # vettore_richieste.append(MoveJoint_class([1,2,3,4,5,6,7,8] , [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0], 0.3, acceleration = 2.0, block=False, relative= False))

    # vettore_richieste.append(MoveJoint_class([1,2,3,4,5,6,7,8] , [-0.1, -0.1, -0.1, -0.1, -0.1, -0.1, 0.1, 0.1], 0.2, acceleration = 2.0, block=False, relative= True))
    # vettore_richieste.append(MoveJoint_class([1,2,3,4,5,6,7,8] , [-0.2, -0.2, -0.2, -0.2, -0.2, -0.2, -0.1, -0.1], 0.2, acceleration = 2.0, block=False, relative= True))
    
    
    # vettore_richieste.append(MoveJoint_class([1,2,3,4,5,6,7,8] , [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0], 0.3, acceleration = 2.0, block=False, relative= False))
