import rclpy
from fp_core_msgs.msg import WheelPosition
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64
from sensor_msgs.msg import Range

from fp_core_msgs.msg import PositionRobot
from fp_core_msgs.msg import RotationRobot

import numpy as np
import matplotlib.pyplot as plt

import csv

with open('src/odometria_test/odometria_test/prova.csv', 'a', newline='') as file:
    writer = csv.writer(file)
    field = ["pos_x", "pos_y"]
    

with open('src/odometria_test/odometria_test/prova1.csv', 'a', newline='') as file:
    writer = csv.writer(file)
    field = ["pos_x", "pos_y"]
    

# Distanze percorse dalle ruote ida t_k a t_k-1

# DL_k = velocita_sx * raggio_ruota*(tempo_k - tempo_k-1)
# DR_k = velocita_dx * raggio_ruota*(tempo_k - tempo_k-1)

# Raggio di curvatura del robot nell'intervallo che va da t_k a t_k-1

# r_k = (L/2)*(DR_k + DL_k)/(DR_K - DL_k)


# Calcolo posizione del robot all'istante k 

# theta_k = theta_k-1 + ((DR_k - DL_k)/L)
# X_k = x_k-1 + r_k( sin(theta_k-1) - sin(theta_k))
# Y_k = y_k-1 + r_k( cos(theta_k-1) - cos(theta_k))



# Inizio dichiarazioni variabili per Odometria 
plot_x = []
plot_y = []

    # Posizioni robot
x_robot = -2.51925
x_robot_old =  -2.51925

y_robot = -0.505749
y_robot_old = -0.505749
phi_robot = 0.00982444
phi_robot_old = 0.00982444
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
        
        self.middle_distance_publisher = self.__node.create_publisher(Range,'middle_sensor',1)
        
        
        
        #Fine nuova aggiunta *******+++++
        
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

    def __cmd_vel_callback(self,twist): # Idea cambiare i valori ad ogni chiamata di questa funzione
        
        global forward_speed,angular_speed
        forward_speed = twist.linear.x
        angular_speed = twist.angular.x

                        
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
        
        #############################################################################
        # Robot localization
        # Utilizzo equazioni per la cinematica del robot basato su velocita 
        # Dopo aver effettuato un primo set di funzioni
        
        
        
        # global wl_robot,wr_robot,delta_t,u_robot,w_robot,x_robot,y_robot,phi_robot,old_encoder_val,encoder_val
        
        # encoder_val[0] = self.__sensor_left.getValue()
        # encoder_val[1] = self.__sensor_right.getValue()
        
        # # Compute speed of the wheels
        # #[wl_robot,wr_robot] = get_wheels_speed(encoder_val, old_encoder_val, delta_t)
        # wl_robot = self.__motor_left.getVelocity()
        # wr_robot = self.__motor_right.getVelocity()
        # # Compute robot linear and angular speeds
        # [u_robot,w_robot] = get_robot_speeds(wl_robot, wr_robot, R, D)
        # #Compute new robot pose 
        # [x_robot, y_robot, phi_robot] = get_robot_pose(u_robot, w_robot, x_robot, y_robot, phi_robot, delta_t)
        
        # # old_encoder_val_dx =  self.__sensor_left.getValue()
        # # old_encoder_val_sx = self.__sensor_right.getValue()
        
        # old_encoder_val[0] = encoder_val[0]
        # old_encoder_val[1] = encoder_val[1]
        
        # self.__node.get_logger().info('getVelocity motor left : ' + str(self.__motor_left.getVelocity())) 
        # self.__node.get_logger().info('getVelocity motor right : ' + str(self.__motor_right.getVelocity())) 

        # messaggio_posizione.y = self.__robot.getSelf().getPosition()[1] 
        # messaggio_posizione.z = self.__robot.getSelf().getPosition()[2] 
        
        # self.__node.get_logger().info('Sim time: ' + str(self.__robot.getTime()))
         
        # self.__node.get_logger().info('Pos x = ' + str(x_robot) + '| Pos x real = '+str(self.__robot.getSelf().getPosition()[0]))
        # self.__node.get_logger().info('Pos y = ' + str(y_robot) + '| Pos y real = '+str(self.__robot.getSelf().getPosition()[1]))
        # self.__node.get_logger().info('phi = ' + str(phi_robot) + ' rad')

        # self.__motor_left.setVelocity(command_motor_left)
        # self.__motor_right.setVelocity(command_motor_right)

        # self.__node.get_logger().info('Basi timeStep: ' + str(self.__robot.getBasicTimeStep())) 
        
        
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
        x_robot_old = x_robot
        y_robot_old = y_robot
        
        
        with open('src/odometria_test//odometria_test//prova.csv', 'a', newline='') as file:
            writer = csv.writer(file)
            writer.writerow([str(x_robot), str(y_robot)])
            
        with open('src/odometria_test/odometria_test/prova1.csv', 'a', newline='') as file:
            writer = csv.writer(file)
            writer.writerow([str(self.__robot.getSelf().getPosition()[0]), str(self.__robot.getSelf().getPosition()[1])])
        
        # plot_x.append(x_robot)
        # plot_y.append(y_robot)

        # fig = plt.figure(figsize=(6, 3))
        # ln, = plt.plot(plot_x, plot_y, '-')
        # plt.axis([-5, 5, -5, 5])
        
        # def update(frame):
        #     x_robot.append()
        # plt.scatter(plot_x,plot_y)
        # plt.show(block = False)        

def get_wheels_speed(encoderValues, oldEncoderValues, delta_t): # Posso farlo con getVelociy() per ogni ruota del motore 
    wl = (encoderValues[0] - oldEncoderValues[0])/delta_t
    wr = (encoderValues[1] - oldEncoderValues[1])/delta_t

    return wl, wr

def get_robot_speeds(wl, wr, r, d):
    """Computes robot linear and angular speeds"""
    u = (r/2.0) * (wr + wl)
    w = (r/d) * (wr - wl)

    return u, w

def get_robot_pose(u, w, x_old, y_old, phi_old, delta_t):
    """Updates robot pose based on heading and linear and angular speeds"""
    delta_phi = w * delta_t
    phi = phi_old + delta_phi
    phi_avg = (phi_old + phi)/2   
    if (phi >= np.pi) :
        phi = phi - 2*np.pi
    elif (phi < -np.pi):
        phi = phi + 2*np.pi
    
    delta_x = u * np.cos(phi_avg) * delta_t
    delta_y = u * np.sin(phi_avg) * delta_t
    x = x_old + delta_x
    y = y_old + delta_y

    return x, y, phi

# Funzioni seguendo le slide di Robotica, velocità non costanti

def calcolo_distanze(velocita_sx,velocita_dx,raggio,delta_t):
    
    print('calcolo distanze')
    # if(velocita_sx == velocita_dx): # MOto rettilineo
        
    #     dl_k = forward_speed*delta_t
    #     dr_k = forward_speed*delta_t
        
    # else:
        
    dl_k = velocita_sx*raggio*delta_t
    dr_k = velocita_dx*raggio*delta_t
        
        # dr_k = velocita_sx*raggio*delta_t
        # dl_k = velocita_dx*raggio*delta_t
    return dl_k,dr_k

def calcolo_curvatura(dl_k,dr_k,distanza):
        
    if(dl_k - dr_k == 0.0):
        
        raggio_curvatura = 0.0
    else:
        
        raggio_curvatura = (distanza/2.0)*((dr_k+dl_k)/(dr_k - dl_k))
        
    return raggio_curvatura

def calcolo_angolo(dl_k,dr_k,distanza,old_angolo):
        
    angolo = old_angolo + ((dr_k - dl_k)/distanza)
    
    
    
    # return angolo
    # Da cambiare in base a come è posizionato il sistema di riferimento de robot 
    
    # if (angolo >= np.pi) :
    #     angolo = angolo - 2*np.pi
    # elif (angolo< -np.pi):
    #     angolo = angolo + 2*np.pi
        
    # if (angolo >= np.pi) :
    #     angolo = angolo + 2*np.pi
    # elif (angolo< -np.pi):
    #     angolo = angolo - 2*np.pi
    
    # if (angolo > 2*np.pi) :
    #     angolo = angolo - 2*np.pi
    # elif (angolo< -2*np.pi):
    #     angolo = angolo + 2*np.pi
        
    return angolo
    
    
def calcolo_posizioni(pos_x_old,pos_y_old,curvatura,angolo_old,angolo):
    
    # Questo è quello giusto, in base al sistema del riferimento del robot
    
    # x_k = pos_x_old + curvatura*(   np.sin(angolo_old) - np.sin(angolo))
    # y_k = pos_y_old - curvatura*(   np.cos(angolo_old) - np.cos(angolo))
    
    # x_k = pos_x_old - curvatura*(   np.sin(angolo_old) - np.sin(angolo))
    # y_k = pos_y_old + curvatura*(   np.cos(angolo_old) - np.cos(angolo))
    
    x_k = pos_x_old - curvatura*(np.sin(angolo_old) - np.sin(angolo))
    y_k = pos_y_old + curvatura*(np.cos(angolo_old) - np.cos(angolo))
    
    
    return x_k,y_k

# Funzioni seguendo le slide di Robotica, velocità costanti

def calcolo_distanzaD(velocita_sx,velocita_dx,raggio,delta_t):

    distanza = ((0.5)*raggio)*(velocita_sx-velocita_dx)*delta_t
    return distanza

def calcolo_angolo2(raggio,distanza,velocita_sx,velocita_dx,delta_t):
    
    angolo = (raggio/distanza)*(velocita_dx - velocita_sx)*delta_t
    return angolo

def calcolo_pos_x(raggio,distanza,velocita_sx,velocita_dx,delta_t):
    
    posizione_x = (distanza/2.0)* ((velocita_dx + velocita_sx)/(velocita_dx - velocita_sx))* np.sin((velocita_dx-velocita_sx)*(raggio/distanza)*delta_t)
    return posizione_x

def calcolo_pos_y(raggio,distanza,velocita_sx,velocita_dx,delta_t):
    
    posizione_y = -(distanza/2.0)* ((velocita_dx + velocita_sx)/(velocita_dx - velocita_sx))* np.cos((velocita_dx-velocita_sx)*(raggio/distanza)*delta_t)
    return posizione_y