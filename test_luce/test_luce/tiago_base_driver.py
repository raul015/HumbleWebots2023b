import rclpy
from fp_core_msgs.msg import WheelPosition
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64
from sensor_msgs.msg import Range
from scipy.spatial.transform import Rotation
from fp_core_msgs.msg import PositionRobot
from fp_core_msgs.msg import RotationRobot

import numpy as np
import matplotlib.pyplot as plt

import csv
    
# Inizio dichiarazioni variabili per Odometria 
plot_x = []
plot_y = []

    # Posizioni robot
x_robot = 2.47169
x_robot_old =  2.47169

y_robot = -2.93575
y_robot_old = -2.93575
phi_robot = 0.00959403
phi_robot_old = 0.00959403
# phi_robot = np.pi
# phi_robot_old= np.pi



    # Inizializzazione velocità e accelerazione in x e y del robot
    
dx = 0.0 #[m/s]
dy = 0.0 #[m/s]   
ddx = 0.0 #[m/s^2]
ddy = 0.0 #[m/s^2]

    # Inizializzazione velocità ruote
    
wl_robot = 0.0
wr_robot = 0.0

    #
u_robot = 0.0 #[m/s]
w_robot = 0.0 #[rad/s]

    # Inizializzo i due valori di encoder vecchi
old_encoder_val = []
old_encoder_val.append(0.0)
old_encoder_val.append(0.0)


encoder_val = []
encoder_val.append(-2.51831)
encoder_val.append(-0.505754)

#
dRk = 0.0
dLk = 0.0
rk = 0.0
    # Tempo di simulazione, 32 millisecondi espresso in secondi
    
delta_t = 32/1000.0

# Fine dichiarazioni variabili per Odometria 


#NB! MODIFIHE SUL MONDO DI WEBOTS MEGLIO ESEGUIRE SUL FILE .WBT

HALF_DISTANCE_BETWEEN_WHEELS = 0.202 #L/2
D = 0.404  # Distanza ruote [m]
R = 0.0985 # Raggio ruota   [m]

WHEEL_RADIUS = 0.0985 #R

# I motori vanno da 10.0 a 0.0
max_speed_motor = 10.0
min_speed_motor = 0.0

motor_left_name = 'wheel_left_joint'
motor_right_name = 'wheel_right_joint'

sensor_left_name = 'wheel_left_joint_sensor'
sensor_right_name = 'wheel_right_joint_sensor'

gps_name = 'gps'
camera_name = 'TiagoCamera'

TIME_STEP = 32 # milliseconds

msg = WheelPosition() # Messaggio per le posizioni....              

msg.name.append(sensor_left_name)
msg.data.append(0.0)

msg.name.append(sensor_right_name)
msg.data.append(0.0)

messaggio_ir_right = Float64()
messaggio_ir_left = Float64()
messaggio_distanza_middle = Range()
messaggio_light = Float64()


messaggio_posizione = PositionRobot()
messaggio_orientamento = RotationRobot()

forward_speed = 0.0
angular_speed = 0.0

command_motor_left = 0.0
command_motor_right = 0.0

illuminazione = 4.0 # Valore di defaul della illuminazione.

class TIAGODriver:
        
    def init(self, webots_node, properties): 
        
        rclpy.init(args=None)

        #WbNodeRef bb8_node = wb_supervisor_node_get_from_def("BB-8")
        #WbFieldRef translation_field = wb_supervisor_node_get_field(bb8_node, "translation")
        #wb_supervisor_field_set_sf_vec3f(translation_field, new_value)

        self.__robot = webots_node.robot
        
        self.__timestep = TIME_STEP
            
        self.__node = rclpy.create_node('tiago_base_driver')
    
        self.__node.get_logger().info('Inizio di tiago_base_driver')
        
        self.target_twist = Twist() # Creo un messaggio anche per il Twist...
            
        
        #  Salvo in delle variabili device Motor e Sensor   
        
        self.__motor_left = self.__robot.getDevice(motor_left_name)
        self.__sensor_left = self.__robot.getDevice(sensor_left_name)
        
        self.__sensor_left.enable(self.__timestep)
        self.__motor_left.setPosition(float('inf'))
        self.__motor_left.setVelocity(0.0)
        #self.__motor_left.setControlPID(10, 3, 3)

        
        self.__motor_right = self.__robot.getDevice(motor_right_name)
        self.__sensor_right = self.__robot.getDevice(sensor_right_name)
        
        self.__sensor_right.enable(self.__timestep)
        self.__motor_right.setPosition(float('inf'))
        self.__motor_right.setVelocity(0.0)
        
        
        self.gps= self.__robot.getDevice(gps_name)
        self.gps.enable(self.__timestep)
        
        
        self.__sensor_LEFTIR = self.__robot.getDevice('base_sonar_03_link')
        self.__sensor_LEFTIR.enable(self.__timestep)
               
        self.__sensor_RIGHTIR = self.__robot.getDevice('base_sonar_01_link')
        self.__sensor_RIGHTIR.enable(self.__timestep)
        
        # Nuova modifica *******+++++ 
        self.__sensor_MIDDLE = self.__robot.getDevice('base_sonar_02_link')
        self.__sensor_MIDDLE.enable(self.__timestep)
        # Fine modifica *******+++++
                
       # Inserimento di una telecamera
        
        self.__camera = self.__robot.getDevice(camera_name)
        # self.__camera.enable(self.__timestep)
        # self.__camera.recognitionEnable(self.__timestep)
        
        # Inserimento di un Lidar
        
        self.__lidar = self.__robot.getDevice('lidar')
        self.__lidar.enable(self.__timestep)
        self.__lidar.enablePointCloud()
        
        # Inserimento sensore di illuminazione
        
        self.__light_sensor = self.__robot.getDevice('light sensor')
        self.__light_sensor.enable(self.__timestep)

        #  I publisher non hanno callbackublisher(Float64,'tiago_ir_left',1)     
        self.right_ir_publisher = self.__node.create_publisher(Float64,'tiago_ir_right',1)
        
        self.left_ir_publisher = self.__node.create_publisher(Float64,'tiago_ir_left',1)
        #Nuova aggiunta  *******+++++
        self.middle_distance_publisher = self.__node.create_publisher(Range,'middle_sensor',1)
        #Fine nuova aggiunta *******+++++
        self.light_publisher = self.__node.create_publisher(Float64,'tiago_light_sensor',1)
        
        # CREO UN SUBSCRIBER PER FAR VARIARE I VALORI RELATIVI ALLA ILLUMINAZIONE
        
        self.__node.create_subscription(Float64,'tiago_illuminazione',self.__illuminazione_callback,1)

        # self.publisher_sensor_values = self.__node.create_publisher(WheelPosition,'Tiago_base_sensors',10)
        
        self.__node.create_subscription(Twist,'tiago_cmd_vel',self.__cmd_vel_callback,1)
        
        self.__node.get_logger().info('  TIAGo Base is supervisor? ' + str(self.__robot.getSupervisor()))
        
        self.posizione_robot__publisher = self.__node.create_publisher(PositionRobot,'tiago_posizione_assoluta',1)
        self.orientazione_robot__publisher = self.__node.create_publisher(RotationRobot,'tiago_orientazione_assoluta',1)

        self.posizione_robot = self.__robot.getSelf().getPosition() # Vettore 1x3        
        self.orientazione_robot = self.__robot.getSelf().getOrientation() # Vettore 3x3    
    
        # Da verificare la formula se devo inserire anche il raggio, passaggio da twist a velocità delle due ruote (IMPORTANTE)
        
        command_motor_left = 0.0
        command_motor_right = 0.0 
        
        self.__motor_right.setVelocity(command_motor_right)
        self.__motor_left.setVelocity(command_motor_left)


        # self.prova = webots_node.node.getSelf()
        # self.__node.get_logger().info('prova: ' + str(self.prova))

        # self.luce = self.__robot.getFromDef('PointLight')
        # self.luce = self.__robot.getSelf().getFromProtoDef('PointLight')
        
        # self.prova = webots_node.robot.getFromDef('TIAGo Base.luce')
        # self.__node.get_logger().info('stampo self.prova ' + str(self.prova))
        
        
        # Print informazioni sul RObot
        
        self.direzione_1 = dir(webots_node.robot)
        self.__node.get_logger().info('Struttura Robot \n ' + str(self.direzione_1))

        self.direzione_2 = dir(webots_node.robot.getSelf().getField('children'))
        self.__node.get_logger().info('Struttura Robot children \n ' + str(self.direzione_2))
        
        self.root = dir(webots_node.robot.getRoot())
        self.__node.get_logger().info('Root \n ' + str(self.root))
        
        self.root_children_field = webots_node.robot.getRoot().getField('children')
        
        self.root_children = dir(self.root_children_field )
        self.__node.get_logger().info('Children:   \n ' + str(self.root_children ))
        
        self.n =  self.root_children_field.getCount()
        
        self.__node.get_logger().info('Il world contiene i seguenti nodi:   \n ' + str(self.n))

        for k in range(self.n):
            self.nodeee = self.root_children_field.getMFNode(k)
            self.__node.get_logger().info(str(self.nodeee.getTypeName())+ '  ' + str(k))
            
            # contenitore = self.nodeee.getTypeName().split(" ")
            # if(contenitore == 'Door'):
            #     self.__node.get_logger().info('trovato '+ str(contenitore))
            #     self.__node.get_logger().info('alla posizione'+ str(k))


        # I PointLight si trovano dal nodo 3 al nodo 12, 3 e 12 sono compresi nel conteggio
        
        # Prova set
        
        # for k in range(5,14):

        #     prova_luce = self.root_children_field.getMFNode(k)
        #     campo_luce = prova_luce.getField('intensity')
        #     campo_luce.setSFFloat(illuminazione) 
        #     # Setto l'illuminazione iniziale a 4
            
        for k in range(self.n):
            
            nodo_k = self.root_children_field.getMFNode(k)
                    
            if(nodo_k.getTypeName() == 'PointLight'):  # Cerco il nodo PointLihgt in base al nome
                
                self.__node.get_logger().info('trovato PointLight alla posizione  '+ str(k))
                light_position = nodo_k.getField('intensity')
                light_position.setSFFloat(illuminazione)
                self.__node.get_logger().info('settato la posizione della luce i a 4.0')

            
            
        # self.direzione_3 = dir(webots_node.robot.getSelf().POINT_LIGHT)   TIAGo%20Base
        self.__node.get_logger().info('Prova print POINT_LIGHT \n ' + str(webots_node.robot.getSelf().getField('luce')))

        

    def __cmd_vel_callback(self,twist): # Idea cambiare i valori ad ogni chiamata di questa funzione
        
        global forward_speed,angular_speed
        forward_speed = twist.linear.x
        angular_speed = twist.angular.x
        
    def __illuminazione_callback(self, oggetto): 
                
        global illuminazione
        illuminazione = oggetto.data
        self.__node.get_logger().info('Illuminazione ricevuta '+ str(illuminazione) + ' !!!' )

        
    
    def step(self): 
        
        global command_motor_left, command_motor_right, forward_speed,angular_speed
        rclpy.spin_once(self.__node, timeout_sec=0)

        #----------------------------------- Pubblicazione valori dei sensori ----------------------------

        messaggio_ir_left.data = self.__sensor_LEFTIR.getValue()
        self.left_ir_publisher.publish(messaggio_ir_left) # Pubblicazione del valore IR SINISTRO

        messaggio_ir_right.data = self.__sensor_RIGHTIR.getValue()
        self.right_ir_publisher.publish(messaggio_ir_right) # Pubblicazione del valore IR DESTRO

        messaggio_distanza_middle.range  = self.__sensor_MIDDLE.getValue()
        self.middle_distance_publisher.publish(messaggio_distanza_middle) # Pubblicazione del valore IR CENTRALE
        
        messaggio_light.data = self.__light_sensor.getValue()
        self.light_publisher.publish(messaggio_light)
        self.__node.get_logger().info('Sensore di illuminazione: ' + str(self.__light_sensor.getValue()))

        
        # Modifica equazione calcolo velocità ruote
        
        command_motor_left = (forward_speed - angular_speed*HALF_DISTANCE_BETWEEN_WHEELS)/WHEEL_RADIUS
        command_motor_right = (forward_speed + angular_speed*HALF_DISTANCE_BETWEEN_WHEELS)/WHEEL_RADIUS
                        
        # Creo il messaggio da mandare al SUBSCRIBER che leggera le posizione assolute
        
        messaggio_posizione.x = self.__robot.getSelf().getPosition()[0] 
        messaggio_posizione.y = self.__robot.getSelf().getPosition()[1] 
        messaggio_posizione.z = self.__robot.getSelf().getPosition()[2] 
        
        # Pubblicazione delle posizioni e orientazioni assolute del robot rispetto ad un sistema di riferimento FISSO
        
        self.posizione_robot__publisher.publish(messaggio_posizione)
        self.orientazione_robot__publisher.publish(messaggio_orientamento)
        
        # Inizio Operazione per odometria 
        
        # Seconda versione.............
        
        global phi_robot_old, phi_robot, wl_robot,wr_robot,dLk,dRk,R,delta_t,D,rk,x_robot,y_robot
        
        wl_robot = self.__motor_left.getVelocity()
        wr_robot = self.__motor_right.getVelocity()
        
        # wl_robot = command_motor_left
        # wr_robot = command_motor_right
        
        [dLk, dRk] = calcolo_distanze(wl_robot,wr_robot,R,delta_t)
        rk = calcolo_curvatura(dLk,dRk,D)
        
        phi_robot =  calcolo_angolo(dLk,dRk,D,phi_robot_old)
        
        [x_robot,y_robot] = calcolo_posizioni(x_robot,y_robot,rk,phi_robot_old,phi_robot)
        
        self.__node.get_logger().info('Pos x = ' + str(x_robot) + '| Pos x real = '+str(self.__robot.getSelf().getPosition()[0]))
        self.__node.get_logger().info('Pos y = ' + str(y_robot) + '| Pos y real = '+str(self.__robot.getSelf().getPosition()[1]))
        self.__node.get_logger().info('phi = ' + str(phi_robot) + ' rad')
        self.__node.get_logger().info('forward_speed' + str(forward_speed))
        self.__node.get_logger().info('raggio di curvatura: ' + str(rk))
        self.__node.get_logger().info('getleft: ' + str(wl_robot))
        self.__node.get_logger().info('getright: ' + str(wr_robot))
        self.__node.get_logger().info('cmdleft: ' + str(command_motor_left))
        self.__node.get_logger().info('cmdright: ' + str(command_motor_right))

        self.__motor_left.setVelocity(command_motor_left)
        self.__motor_right.setVelocity(command_motor_right)
        
        phi_robot_old = phi_robot
        
        # Illuminazione in quanto il mondo è cambiato...
        
        # for k in range(5,14):
        #     prova_luce = self.root_children_field.getMFNode(k)
        #     campo_luce = prova_luce.getField('intensity')
        #     campo_luce.setSFFloat(illuminazione) # Setto l'illuminazione iniziale a 4
            
            
        
        if(self.__light_sensor.getValue()<8.00):

            
            self.__node.get_logger().info('disabilitazione camera ')

            self.__camera.disable()
            self.__camera.recognitionDisable()
        else:
            
            self.__node.get_logger().info('riabilitazione camera ')
            self.__camera.enable(self.__timestep)
            self.__camera.recognitionEnable(self.__timestep)
            
            numero_oggetti = self.__camera.getRecognitionNumberOfObjects()
            oggetti_trovati = self.__camera.getRecognitionObjects()
            self.__node.get_logger().info('numero oggetti riconosciuti: ' + str(numero_oggetti))
            
            for we in range(numero_oggetti):
                
                self.__node.get_logger().info('id oggetto riconosciuto: ' + str(oggetti_trovati[we].getId()))
                self.__node.get_logger().info('posizione oggetto riconosciuto: ' + str(oggetti_trovati[we].getPosition()))
                self.__node.get_logger().info('orientamento oggetto riconosciuto: ' + str(oggetti_trovati[we].getOrientation()))
                self.__node.get_logger().info('size oggetto riconosciuto: ' + str(oggetti_trovati[we].getSize()))
                self.__node.get_logger().info('modello oggetto riconosciuto: ' + str(oggetti_trovati[we].getModel()))


        for k in range(self.n):
            
            nodo_k = self.root_children_field.getMFNode(k)
                    
            if(nodo_k.getTypeName() == 'PointLight'):  # Cerco il nodo PointLihgt in base al nome
                
                self.__node.get_logger().info('trovato PointLight alla posizione  '+ str(k))
                light_position = nodo_k.getField('intensity')
                light_position.setSFFloat(illuminazione)
                self.__node.get_logger().info('settato la posizione della luce i a 3.0')

        self.__node.get_logger().info('valore di illuminazione ' + str(illuminazione))

            
        # # if(self.__light_sensor.getValue()<260.00):
        
        # if(self.__light_sensor.getValue()<70.00):

            
        #     self.__node.get_logger().info('disabilitazione camera ')

        #     self.__camera.disable()
        #     self.__camera.recognitionDisable()
        # else:
            
        #     self.__node.get_logger().info('riabilitazione camera ')
        #     self.__camera.enable(self.__timestep)
        #     self.__camera.recognitionEnable(self.__timestep)
            
        self.__node.get_logger().info('valore sensore illuminazione: ' +  str( self.__light_sensor.getValue()))

            
            
# Funzioni seguendo le slide di Robotica, velocità non costanti

def calcolo_distanze(velocita_sx,velocita_dx,raggio,delta_t):
            
    dl_k = velocita_sx*raggio*delta_t
    dr_k = velocita_dx*raggio*delta_t
        
    return dl_k,dr_k

def calcolo_curvatura(dl_k,dr_k,distanza):
        
    if(dl_k - dr_k == 0.0):
        
        raggio_curvatura = 0.0
    else:
        
        raggio_curvatura = (distanza/2.0)*((dr_k+dl_k)/(dr_k - dl_k))
        
    return raggio_curvatura

def calcolo_angolo(dl_k,dr_k,distanza,old_angolo):
        
    angolo = old_angolo + ((dr_k - dl_k)/distanza)
    
        
    return angolo
    
    
def calcolo_posizioni(pos_x_old,pos_y_old,curvatura,angolo_old,angolo):
    
    x_k = pos_x_old - curvatura*(np.sin(angolo_old) - np.sin(angolo))
    y_k = pos_y_old + curvatura*(np.cos(angolo_old) - np.cos(angolo))
    
    
    return x_k,y_k