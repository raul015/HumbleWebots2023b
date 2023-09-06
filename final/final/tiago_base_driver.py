import rclpy
from fp_core_msgs.msg import WheelPosition
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64
from sensor_msgs.msg import Range
from webots_ros2_driver_webots import controller

from fp_core_msgs.msg import PositionRobot
from fp_core_msgs.msg import RotationRobot

import numpy as np
from math import (asin, pi, atan2, cos)
# Prova calcolo roll pithc yaw

#NB! MODIFIHE SUL MONDO DI WEBOTS MEGLIO ESEGUIRE SUL FILE .WBT

HALF_DISTANCE_BETWEEN_WHEELS = 0.202
WHEEL_RADIUS = 0.0985

# I motori vanno da 10.0 a 0.0
max_speed_motor = 10.0
min_speed_motor = 0.0

motor_left_name = 'wheel_left_joint'
motor_right_name = 'wheel_right_joint'

sensor_left_name = 'wheel_left_joint_sensor'
sensor_right_name = 'wheel_right_joint_sensor'

gps_name = 'gps'


TIME_STEP = 32 # milliseconds

msg = WheelPosition() # Messaggio per le posizioni....              

msg.name.append(sensor_left_name)
msg.data.append(0.0)

msg.name.append(sensor_right_name)
msg.data.append(0.0)

messaggio_ir_right = Float64()
messaggio_ir_left = Float64()
messaggio_distanza_middle = Range()

messaggio_posizione = PositionRobot()
messaggio_orientamento = RotationRobot()

forward_speed = 0.0
angular_speed = 0.0

command_motor_left = 0.0
command_motor_right = 0.0

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
        
        # self.posizioni= self.prova.getPosition(self)
        # print(str(self.posizioni))
    

        #self.__motor_right.setControlPID(10, 3, 3)
        
        # Voglio avere controllo anche sui sensori di distanza per il riconoscimento dei colori
        
        
        # Nuova aggiunta con i sensori *******+++++
        
        #self.sensorTImer = self.create_timer(self.__timestep,self.sensor_callback)
        
        # Fine nuova aggiunta *******+++++
        
        
        # Robot.getDevice   ->getDistanceSensor è deprecato
        self.__sensor_LEFTIR = self.__robot.getDevice('base_sonar_03_link')
        self.__sensor_LEFTIR.enable(self.__timestep)
               
        self.__sensor_RIGHTIR = self.__robot.getDevice('base_sonar_01_link')
        self.__sensor_RIGHTIR.enable(self.__timestep)
        
        # Nuova modifica *******+++++ 
        self.__sensor_MIDDLE = self.__robot.getDevice('base_sonar_02_link')
        self.__sensor_MIDDLE.enable(self.__timestep)
        # Fine modifica *******+++++
        
        # Inserimento di un Lidar
        
        self.__lidar = self.__robot.getDevice('lidar')
        self.__lidar.enable(self.__timestep)
        self.__lidar.enablePointCloud()

        #  I publisher non hanno callback
             
        self.left_ir_publisher = self.__node.create_publisher(Float64,'tiago_ir_left',1)
        self.right_ir_publisher = self.__node.create_publisher(Float64,'tiago_ir_right',1)
        #Nuova aggiunta  *******+++++
        self.middle_distance_publisher = self.__node.create_publisher(Range,'middle_sensor',1)
        #Fine nuova aggiunta *******+++++
        
        # self.publisher_sensor_values = self.__node.create_publisher(WheelPosition,'Tiago_base_sensors',10)
        
        self.__node.create_subscription(Twist,'tiago_cmd_vel',self.__cmd_vel_callback,1)
        
        self.__node.get_logger().info('  TIAGo Base is supervisor? ' + str(self.__robot.getSupervisor()))
        
        
        # Prendo direttamente i di posizione dai valori del simulatore 
        
        # print(str(self.__robot.getSelf().getPosition()))
        # self.__node.get_logger().info(str(self.__robot.getSelf().getPosition()))
        
        
        # self.__node.create_subscription(PositionRobot,'tiago_posizione_assoluta',self.__posizione_assoluta_callback,1)
        # self.__node.create_subscription(PositionRobot,'tiago_orientazione_assoluta',self.__orientazione_assoluta_callback,1)
        
        self.posizione_robot__publisher = self.__node.create_publisher(PositionRobot,'tiago_posizione_assoluta',1)
        self.orientazione_robot__publisher = self.__node.create_publisher(RotationRobot,'tiago_orientazione_assoluta',1)



        self.posizione_robot = self.__robot.getSelf().getPosition() # Vettore 1x3        
        self.orientazione_robot = self.__robot.getSelf().getOrientation() # Vettore 3x3
        
        
        # self.posizione_robot --> Posizione del nodo robot espresso in coordinate globali del sistema  x,y,z
        
        # self.orientazione_robot ---> Matrice che rappresenta la rotazione del nodo in coordinate globali del sistema
        # Ritorna una matrice conenente 9 valori che sono inerpretati come matrice ortogonale di torazione 3x3
        # Ogni colonna della matrice rappresenta x,y,z del sistema di coordinate del nodo
        # Le colonne (e le righe) della matrice sono vettori unitari ortogonali 
        # a coppie (cioè formano una base ortonormale). 
        # Poiché la matrice è ortogonale, la sua trasposta è anche la sua inversa. 
        # Quindi trasponendo la matrice puoi ottenere la rotazione inversa.
        
        # Moltiplicando la matrice di rotazione a destra per un vettore e
        # quindi aggiungendo il vettore di posizione è possibile esprimere
        # le coordinate di un punto nel sistema di coordinate globale (mondo)
        # conoscendo le sue coordinate in un sistema di coordinate locale (nodo).
        
        
        # Prendo direttamente i di posizione dai valori del simulatore 
    
        #print(str(self.__robot.getSelf().getOrientation()))
        #self.__node.get_logger().info(str(self.__robot.getSelf().getOrientation()))
        
                    
    # def sensor_callback(self): 
        # Questo verrà fatto ogni 32 millisecondi, volendo 
        #Posso modificarlo a 16 millisecondi
        
        # print('prova sensore callback')
        
        # # Pubblicazione del valore IR DESTRO
        # messaggio_ir_right.data = self.__sensor_RIGHTIR.getValue()
        # self.right_ir_publisher.publish(messaggio_ir_right)

        # # Pubblicazione del valore IR SINISTRO
        # messaggio_ir_left.data = self.__sensor_LEFTIR.getValue()
        # self.left_ir_publisher.publish(messaggio_ir_left) 
        
        # # Pubblicazione del  valore di distanza del robot Centrale
        # messaggio_distanza_middle.range  = self.__sensor_MIDDLE.getValue()
        # self.middle_distance_publisher.publish(messaggio_distanza_middle) 
        # print('dentro sensor callback')
        
        # self.get_logger().info('STAMPA DENTRO sensor_callback')

        # self.get_logger().info('sensore ir L[' + str(messaggio_ir_left)+ ']' )
        # self.get_logger().info('sensore ir R[' + str(messaggio_ir_right)+ ']' )
        
        # self.get_logger().info('sensore distanza centrale[' + str(messaggio_ir_right)+ ']' )
        
        
        # Da verificare la formula se devo inserire anche il raggio, passaggio da twist a velocità delle due ruote (IMPORTANTE)
        
        command_motor_left = 0.0
        command_motor_right = 0.0 
        
        self.__motor_right.setVelocity(command_motor_right)
        self.__motor_left.setVelocity(command_motor_left)

    def __cmd_vel_callback(self,twist): # Idea cambiare i valori ad ogni chiamata di questa funzione
        
        global forward_speed,angular_speed
        forward_speed = twist.linear.x
        angular_speed = twist.angular.x

                
        #WHEEL_RADIUS = 0.0985
        
    def step(self): 
        
        global command_motor_left, command_motor_right, forward_speed,angular_speed
        
        rclpy.spin_once(self.__node, timeout_sec=0)

        # msg.data[0] = self.__sensor_left.getValue()
        # msg.data[1] = self.__sensor_right.getValue()
        
        #----------------------------------- Pubblicazione valori dei sensori ----------------------------

        messaggio_ir_left.data = self.__sensor_LEFTIR.getValue()
        self.left_ir_publisher.publish(messaggio_ir_left) # Pubblicazione del valore IR SINISTRO

        messaggio_ir_right.data = self.__sensor_RIGHTIR.getValue()
        self.right_ir_publisher.publish(messaggio_ir_right) # Pubblicazione del valore IR DESTRO

        messaggio_distanza_middle.range  = self.__sensor_MIDDLE.getValue()
        self.middle_distance_publisher.publish(messaggio_distanza_middle) # Pubblicazione del valore IR CENTRALE
        
        #----------------------------------- Pubblicazione valori dei sensori ----------------------------

        # self.__node.get_logger().info('STAMPA DENTRO step')
        
        # self.__node.get_logger().info('sensore ir L[' + str(messaggio_ir_left.data)+ ']' )
        # self.__node.get_logger().info('sensore ir R[' + str(messaggio_ir_right.data)+ ']' )
        
        # self.__node.get_logger().info('sensore distanza centrale[' + str(messaggio_ir_right.data)+ ']' )
        
        

        # self.__node.get_logger().info('valore gps [x]: ' + str(round(self.gps.getValues()[0],4)) )
        # self.__node.get_logger().info('valore gps [y]: ' + str(round(self.gps.getValues()[1],4)) )
        # self.__node.get_logger().info('valore gps [z]: ' + str(round(self.gps.getValues()[2],4)) )


        # Modifica equazione calcolo velocità ruote
        command_motor_left = (forward_speed - angular_speed*HALF_DISTANCE_BETWEEN_WHEELS)/WHEEL_RADIUS
        command_motor_right = (forward_speed + angular_speed*HALF_DISTANCE_BETWEEN_WHEELS)/WHEEL_RADIUS
                

        self.__motor_left.setVelocity(command_motor_left)
        self.__motor_right.setVelocity(command_motor_right)
        
        
        # Creo il messaggio da mandare al SUBSCRIBER che leggera le posizione assolute
        
        messaggio_posizione.x = self.__robot.getSelf().getPosition()[0] 
        messaggio_posizione.y = self.__robot.getSelf().getPosition()[1] 
        messaggio_posizione.z = self.__robot.getSelf().getPosition()[2] 
        
        
        # float64[3] x
        # float64[3] y
        # float64[3] z
        
        
        # Creo il messaggio da mandare al SUBSCRIBER che leggera le orientazioni assolute 
        
                
        messaggio_orientamento.x[0] = self.__robot.getSelf().getOrientation()[0]
        messaggio_orientamento.x[1] = self.__robot.getSelf().getOrientation()[1]
        messaggio_orientamento.x[2] = self.__robot.getSelf().getOrientation()[2]
        
        messaggio_orientamento.y[0] = self.__robot.getSelf().getOrientation()[3]
        messaggio_orientamento.y[1] = self.__robot.getSelf().getOrientation()[4]
        messaggio_orientamento.y[2] = self.__robot.getSelf().getOrientation()[5]
        
        messaggio_orientamento.z[0] = self.__robot.getSelf().getOrientation()[6]
        messaggio_orientamento.z[1] = self.__robot.getSelf().getOrientation()[7]
        messaggio_orientamento.z[2] = self.__robot.getSelf().getOrientation()[8]
        
        # Provo a creare una matrice di rotazione
        
        R = np.array([
                        [messaggio_orientamento.x[0],messaggio_orientamento.y[0],messaggio_orientamento.z[0]],
                        [messaggio_orientamento.x[1],messaggio_orientamento.y[1],messaggio_orientamento.z[1]],
                        [messaggio_orientamento.x[2],messaggio_orientamento.y[2],messaggio_orientamento.z[2]]
                        ])
                
        self.__node.get_logger().info('\n ' + str(R))
        self.__node.get_logger().info('\n ' + str(R[2,0]))

        
        # #Applicazione delle formule di Eulero, da rivedere bene, qualcosa non torna con queste formule...

        # if( R[2,0] != 1 and R[2,0] != -1 ):
            
        #     pitch_1 = -1*asin(R[2,0])
        #     pitch_2 = pi - pitch_1 
        #     roll_1 = atan2( R[2,1] / cos(pitch_1) , R[2,2] /cos(pitch_1) ) 
        #     roll_2 = atan2( R[2,1] / cos(pitch_2) , R[2,2] /cos(pitch_2) ) 
        #     yaw_1 = atan2( R[0,1] / cos(pitch_1) , R[0,0] / cos(pitch_1) )
        #     yaw_2 = atan2( R[0,1] / cos(pitch_2) , R[0,0] / cos(pitch_2) ) 
            
        # else:
            
        #     yaw = 0 # anything (we default this to zero)
            
        #     if R[2,0] == -1: 
        #         pitch = pi/2 
        #         roll = yaw + atan2(R[0,1],R[0,2]) 
        #     else: 
        #         pitch = -pi/2 
        #         roll = -1*yaw + atan2(-1*R[0,1],-1*R[0,2]) 

        # # convert from radians to degrees
        # roll = roll*180/pi 
        # pitch = pitch*180/pi
        # yaw = yaw*180/pi 
        
        # rxyz_deg = [roll , pitch , yaw] 

        # self.__node.get_logger().info('orientamento in gradi: ' + str(rxyz_deg))
        
        # self.__node.get_logger().info(str(self.__robot.getSelf().getOrientation()[0]))
        # self.__node.get_logger().info(str(self.__robot.getSelf().getOrientation()[1]))
        # self.__node.get_logger().info(str(self.__robot.getSelf().getOrientation()[2]))

        # Pubblicazione delle posizioni e orientazioni assolute del robot rispetto ad un sistema di riferimento FISSO
        
        self.posizione_robot__publisher.publish(messaggio_posizione)
        self.orientazione_robot__publisher.publish(messaggio_orientamento)
        
        
        range_image = self.__lidar.getRangeImage()
        
        # self.__node.get_logger().info('[' + str(range_image[0]) + ']' +
        #                               '[' + str(range_image[1]) + ']' +
        #                               '[' + str(range_image[2]) + ']' +
        #                               '[' + str(range_image[3]) + ']' +
        #                               '[' + str(range_image[4]) + ']' )

        
        
        # self.__node.get_logger().info('Posizione del robot:')
        # self.__node.get_logger().info(str(self.posizione_robot))
        
        # self.__node.get_logger().info('Orientazione del robot:')
        # self.__node.get_logger().info(str(self.orientazione_robot))



        
        # self.left_ir_publisher.publish(messaggio_ir_left) # Pubblicazione del valore IR SINISTRO
        # self.right_ir_publisher.publish(messaggio_ir_right) # Pubblicazione del valore IR DESTRO

        
        #Nuova aggiunta 
        
        # self.middle_distance_publisher.publish(messaggio_distanza_middle) # Pubblicazione del valore IR CENTRALE

        
        #ORDINE DI PUBBLICAZIONE PER OGNI STEP: PRIMA SENSORE SINISTRO POI SENSORE DESTRO....
        
        
        # self.publisher_sensor_values.publish(msg) # Pubblico i messaggi, invece di stamparli qui  
              
        # self.__node.get_logger().info('sensore pioneer L[' + str(0)+ '] = ' + str(msg.data[0]))
        # self.__node.get_logger().info('sensore pioneer R[' + str(1)+ '] = ' + str(msg.data[1]))
        
        
        #Test twist , questi saranno poi aggiornati dal file tiago_line_follower
        
        # Valori iniziale in ingresso al robot
        

        # self.__node.get_logger().info('get vel sx: ' + str(self.__motor_left.getVelocity()))
        # self.__node.get_logger().info('get vel dx: ' + str(self.__motor_right.getVelocity()))
       
       
       
        #self.__motor_right.setVelocity(command_motor_right)
        
        # pubblicazione misura laser  +  odometria  (spostamento robot + orientamento) + posizione assoluta
                            
        
                    