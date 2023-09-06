import rclpy
from fp_core_msgs.msg import WheelPosition
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64
from sensor_msgs.msg import Range
from scipy.spatial.transform import Rotation
from fp_core_msgs.msg import PositionRobot
from fp_core_msgs.msg import RotationRobot
from fp_core_msgs.srv import RequestData

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

attivazione_telecamera = False

class TIAGODriver:
        
    def init(self, webots_node, properties): 
        
        rclpy.init(args=None)

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
        self.middle_distance_publisher = self.__node.create_publisher(Range,'tiago_middle',1)
        #Fine nuova aggiunta *******+++++
        self.light_publisher = self.__node.create_publisher(Float64,'tiago_light_sensor',1)
        
        # CREO UN SUBSCRIBER PER FAR VARIARE I VALORI RELATIVI ALLA ILLUMINAZIONE
        
        self.__node.create_subscription(Float64,'tiago_illuminazione',self.__illuminazione_callback,1)
        
        self.__node.create_service(RequestData, 'abilitazioneTelecamera',self.servizio_attivazioneCamera)


        # CREO UN SUBSCRIBER PER VAR VARIARE L'APERTURA E LA CHIUSURA DELLE PORTE
        
        self.__node.create_subscription(Float64,'posizione_porta',self.__porta_callback,1)
        
        
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
                
        # Print informazioni sul RObot
 
        self.root_children_field = webots_node.robot.getRoot().getField('children')
        
        self.n =  self.root_children_field.getCount()
             
                
    def __cmd_vel_callback(self,twist): # Idea cambiare i valori ad ogni chiamata di questa funzione
        
        global forward_speed,angular_speed
        forward_speed = twist.linear.x
        angular_speed = twist.angular.x
        
    def __illuminazione_callback(self, oggetto): 
                
        global illuminazione
        illuminazione = oggetto.data
        # self.__node.get_logger().info('Illuminazione ricevuta '+ str(illuminazione) + ' !!!' )

    def __porta_callback(self, oggetto):
        global apertura
        apertura = oggetto.data
        # self.__node.get_logger.info('apertura ricevuta ' + str(apertura) + '!!!')
        
    
    def servizio_attivazioneCamera(self,request , response):
        # La request rimane vuota, mentre la response devo riempirla  --> bool success, string message
        
        global attivazione_telecamera
        response.success = True
        response.message = 'messaggio per attivazione telecamera ricevuto'
        attivazione_telecamera = True
        return response
        
    def step(self): 
        
        global command_motor_left, command_motor_right, forward_speed,angular_speed,illuminazione
        global messaggio_ir_right, messaggio_ir_left, messaggio_distanza_middle, messaggio_light
        global attivazione_telecamera
        
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
        # self.__node.get_logger().info('Sensore di illuminazione: ' + str(self.__light_sensor.getValue()))

        
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
        
        self.__motor_left.setVelocity(command_motor_left)
        self.__motor_right.setVelocity(command_motor_right)
                
        if(attivazione_telecamera == True):
            
            # if(self.__light_sensor.getValue()<260.00):

            if(self.__light_sensor.getValue()<20):
                
                # self.__node.get_logger().info('disabilitazione camera ')

                self.__camera.disable()
                self.__camera.recognitionDisable()
            else:
                
                # self.__node.get_logger().info('riabilitazione camera ')
                
                    self.__camera.enable(self.__timestep)
                    self.__camera.recognitionEnable(self.__timestep)
                
        
        # self.__node.get_logger().info('valore sensore illuminazione: ' +  str( self.__light_sensor.getValue()))

            
# Funzioni seguendo le slide di Robotica, velocità non costanti
