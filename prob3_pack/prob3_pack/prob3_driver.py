import rclpy
# import sys
from std_msgs.msg import String

from fp_core_msgs.msg import JointPosition
from fp_core_msgs.msg import JointVelocity
from fp_core_msgs.srv import MoveJoint
from prob3_pack import Funzioni
from rclpy.executors import MultiThreadedExecutor, SingleThreadedExecutor
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
import time


# limiti di posizione, vale sia per valori positivi che negativi
# I limiti non valgono in valore assoluto per le pinze...
# vanno da 0 a 1.0472 perché 0 rappresenta il loro punto di contatto,
# con valori negativi si distruggono...

limite_1_sup = 2.95903
limite_2_sup = 1.91148
limite_3_sup = 1.99927
limite_4_sup = 2.95153
limite_5_sup = 1.9977
limite_6_sup = 2.95833
limite_7_sup = 1.0472   
limite_8_sup = 1.0472


# Inseriti anche i limiti inferiori per motori e pinze 

limite_1_inf = -2.95903
limite_2_inf = -1.91148
limite_3_inf = -1.99927
limite_4_inf = -2.95153
limite_5_inf = -1.9977
limite_6_inf = -2.95833
limite_7_inf = 0.0 
limite_8_inf = 0.0

# limiti di velcità

max_speed_motors= 1.74  # range che va da 0 a 1.74
max_speed_pliers = 1

limits_position_sup = []
limits_position_inf = []
limits_velocity = []



# Riempimento dei vettori limite posizione e velocità

for i in range(8): 

    exec('limits_position_sup.append(limite_' + str(i+1) + '_sup)')
    exec('limits_position_inf.append(limite_' + str(i+1) + '_inf)')
    
    if(i < 6):
        limits_velocity.append(max_speed_motors)
    else:
        limits_velocity.append(max_speed_pliers)
        

motor_1_name = 'motor 1'
motor_2_name = 'motor 2'
motor_3_name = 'motor 3'
motor_4_name = 'motor 4'
motor_5_name = 'motor 5'
motor_6_name = 'motor 6'
motor_7_name = 'motor 7'
motor_8_name = 'motor 7 left'

motor_1_sensor_name = 'motor 1 sensor'
motor_2_sensor_name = 'motor 2 sensor'
motor_3_sensor_name = 'motor 3 sensor'
motor_4_sensor_name = 'motor 4 sensor'
motor_5_sensor_name = 'motor 5 sensor'
motor_6_sensor_name = 'motor 6 sensor'
motor_7_sensor_name = 'motor 7 sensor'
motor_8_sensor_name = 'motor 7 left sensor'


# abilitazione delle modifiche jointPosition
class MyRobotDriver:
    
    # In questo caso webots_node contiene il riferimento al Supervisor
    # Mentre properties è un dizionario creato dai tag XML
    
    def init(self, webots_node, properties):


        #   Sfortunatamento non posso prendere una istanza del genitore ROS node
        # quindi ne creiamo uno... rclpy.init(args=None) , rclpy.create_node('Nome del nodo')
        
        
        self.__robot = webots_node.robot
        self.__timestep = int (self.__robot.getBasicTimeStep())
        # self.__camera = self.__robot.getDevice("camera")
        # self.__camera.enable(self.__timestep)
        
        
        # Prova inserimento telecamera tra le pinze del robot: 
        
        
        # self.__cliente = self.create_client(MoveJoint ,'Movimento')
                
        # Inizializzazione vettore dei nomi dei motori    
        self.motors_name = []
        self.sensors_name = []
        
        self.arrivo = []
        
        self.arrivo.append(False)    #1
        self.arrivo.append(False)    #2
        self.arrivo.append(False)    #3
        self.arrivo.append(False)    #4
        self.arrivo.append(False)    #5
        self.arrivo.append(False)    #6
        self.arrivo.append(False)    #7
        self.arrivo.append(False)    #8

        for i in range(8): 

            # Riempimento vettore nomi motori
            exec('self.motors_name.append(motor_'+ str(i+1) +'_name)')
            
            # RIempimento vettore nomi_sensori_motori
            exec('self.sensors_name.append(motor_'+ str(i+1) +'_sensor_name)')

    
        #  Vettore oggetti motori 
        self.__motors_device = []
        
        # Vettore oggetti sensori 
        self.__sensors_device = []
        
        # Vettore posizioni iniziali motori
        self.__positions = []
        
        # Vettore velocità iniziali motori
        self.__velocities = []
        
        for i in range(8): 
            

            #   Riempimento motori_device
            self.__motors_device.append(self.__robot.getDevice(self.motors_name[i]))
            
            #   Riempimento sensori_device
            self.__sensors_device.append(self.__robot.getDevice(self.sensors_name[i]))
            
            #   Abilitazione sensori
            self.__sensors_device[i].enable(self.__timestep)  
            
            # Riempimento vettore posizioni iniziali dei motori appena viene avviato P-Rob3  
            self.__positions.append(float('inf'))
            
            # Rempimento vettore velocità iniziali dei motori appena viene avviato P-Rob3  
            self.__velocities.append(0.0)
            
            # Setup  motori_device , sensori_device 
            self.__motors_device[i].setPosition(self.__positions[i])
            self.__motors_device[i].setVelocity(self.__velocities[i])
                
        self.__joint_position = JointPosition()
        self.__joint_velocity = JointVelocity()
        
        
        rclpy.init(args=None)
        
        self.__node = rclpy.create_node('prob3_driver')
        

        #self.executor = MultiThreadedExecutor(num_threads = 6 )
        #self.executor.add_node(self.__node)

        for i in range(8): 
            self.__node.get_logger().info('i : ' + str(i))
        
        self.__node.get_logger().info('  - properties: ' + str(properties))
        self.__node.get_logger().info('  - robot name: ' + str(self.__robot.getName()))
        self.__node.get_logger().info('  - basic timestep: ' + str(int(self.__robot.getBasicTimeStep())))
        self.__node.get_logger().info('  - is supervisor? ' + str(self.__robot.getSupervisor()))
        self.__node.get_logger().info('Prova valori properties: ')
        
        self.__node.get_logger().info((properties.get('parametro1')))
        self.__node.get_logger().info((properties.get('parametro2')))
        self.__node.get_logger().info((properties.get('parametro3')))
        
        #Creazione di una configurazione che va a multithread
        #self.group = ReentrantCallbackGroup()
        #self.__node.create_subscription(JointPosition, 'Joints_position', self.__joint_position_callback, qos_profile =1, callback_group= self.group)
        #self.__node.create_subscription(JointVelocity, 'Joints_velocity', self.__joint_velocity_callback, qos_profile = 1 , callback_group= self.group)
        #self.__node.create_subscription(JointVelocity, 'Sensor_values', self.__sensor_values_callback, qos_profile = 1)
        
        
        self.__node.create_subscription(JointPosition, 'Joints_position', self.__joint_position_callback, qos_profile =1)

        self.__node.create_subscription(JointVelocity, 'Joints_velocity', self.__joint_velocity_callback, qos_profile = 1)

        
        self.__node.create_subscription(String, 'Sensor_values', self.__sensor_values_callback, qos_profile=1)
        
        #   Creazione di nodi servizio all'interno del costruttore 
        #self.__node.create_service(MoveJoint, 'MoveJoint',self.prova_servizio_callback, callback_group = self.group)
        self.__node.create_service(MoveJoint, 'MoveJoint',self.servizio_movejoint)

    # ------------------------------------------------------- FUNZIONE SERVICE --------------------------------------------------
    def servizio_movejoint(self,request , response):
        
        print('sono dentro al servizio')
        actuator_ids = request.actuator_ids
        position = request.position
        velocity = request.velocity
        acceleration = request.acceleration
        block = request.block
        relative = request.relative    
        
        # Controlli parametri in ingresso
        # Sono locali, quindi ad ogni chiamata sono tutti settati a false
        
        response.message = response.message + ' \n '
        
        check_id = []   # Controllo che gli id inseriti siano tutti corretti
        check_posizione = [] # Controllo che la posizione inserita non superi i limiti permetti dal robot
        check_velocita = False
        range_percorso = []    # Spostamento in valore assoluto, quindi distanza percorsa
        final_goal = []
        
        for i in range(8):  
            check_id.append(False)
            check_posizione.append(False)
            range_percorso.append(0.0)
            final_goal.append(False)
        
        # indice e valore posizione massimo
        
        valore_max = 0.0
        scala = 0.0
        
        self.__node.get_logger().info('Parametri di ingresso : ')
        self.__node.get_logger().info(str(actuator_ids))
        self.__node.get_logger().info(str(position ))
        self.__node.get_logger().info(str(velocity))
        self.__node.get_logger().info(str(acceleration))
        self.__node.get_logger().info(str(block))
        self.__node.get_logger().info(str(relative)) 
        
        if(len(actuator_ids) >0  and  len(position)>0  and  (len(actuator_ids) == len(position))):
            
            response.success = True
            #controllo il contenuto all'interno del vettore
            for i in range(len(actuator_ids)):
                
                if(actuator_ids[i]>= 1 and actuator_ids[i]<=8):     #Id inserito rispettato
                    
                    # response.message = response.message + str('Id [' + str( actuator_ids[i]) + '] corretto, \n')
                    
                    response.success = response.success and True

                    self.__node.get_logger().info('Id [' + str(actuator_ids[i]) + '] inserito correttamente')
                    
                    check_id[actuator_ids[i]-1] = True    #   Check sull'Id superato, Id vanno da 1 a 8
                    
                    # Limite di posizione deve essere rispettato sia a destra che a sinistra
                
                    if(relative == False):
                        
                        if( (position[i] <= limits_position_sup[actuator_ids[i]-1]) and (position[i]>= limits_position_inf[actuator_ids[i]-1] ) ):   
                            
                            # response.message = response.message + str('posizione  [' +  str(position[i]) + ']  corretto \n ')
                            
                            response.success = response.success and True

                            # Indice valido per gli ingressi inseriti dall'utente
                            
                            # Calcolo spostamento massimo, si inzia con 0.0
                            # |(posizione_richiesta - posizione_letta_dai_sensori)|
                            
                            range_percorso[actuator_ids[i]-1] = abs(position[i] - self.__sensors_device[actuator_ids[i]-1].getValue())

                            if(valore_max < range_percorso[actuator_ids[i]-1]): 
                                
                                # valore_max = position[i]
                                valore_max = abs(position[i] - self.__sensors_device[actuator_ids[i]-1].getValue())
                                self.__node.get_logger().info('valore_max = ' + str(position[i]))

                                
                            self.__node.get_logger().info('posizione['+ str(position[i])+'] per id [' + str(actuator_ids[i]) + '] inserito correttamente')
                            self.__node.get_logger().info('rispetta il limite superiore ['+ str(limits_position_sup[actuator_ids[i]-1]) + ']')
                            self.__node.get_logger().info('rispetta il limite inferiore ['+ str(limits_position_inf[actuator_ids[i]-1]) + ']')
                            self.__node.get_logger().info('\n')
                    
                            check_posizione[actuator_ids[i]-1] = True    #   Check sulla posizione, richiesta superato 
                            
                            
                
                            if(position[i] >= self.__sensors_device[actuator_ids[i]-1].getValue() ):
                                
                                self.__node.get_logger().info('posizione target[' + str(position[i])+'] > posizione rilevato dal sensore['+ 
                                str(self.__sensors_device[actuator_ids[i]-1].getValue())+ ']')
                                
                                # check_limite[actuator_ids[i]-1] = True    #   Check sull'Id superato 

                                        
                                # Devo testare altre condizioni 
                                if( (position[i] - self.__sensors_device[actuator_ids[i]-1].getValue() ) >0.01):

                               # if( abs(position[i] - self.__sensors_device[actuator_ids[i]-1].getValue() ) >0.01):
                                    
                                    self.__positions[actuator_ids[i]-1] = position[i]
                                    self.__motors_device[actuator_ids[i]-1].setPosition(self.__positions[actuator_ids[i]-1])
                                    self.arrivo[actuator_ids[i] -1 ] = False #Ho settato una posizione quindi devo nuovermi , quindi arrivo è Falso
                                    self.__node.get_logger().info('settate le posizioni')
                                    final_goal[actuator_ids[i]-1] = False
                    
                                    
                                else: 
                                    self.__node.get_logger().info('Obiettivo di posizione ' + str(self.__sensors_device[actuator_ids[i]-1].getValue()) + 'raggiunto')
                                    check_posizione[actuator_ids[i]-1] = False
                                    #Prova posizione
                                    final_goal[actuator_ids[i]-1] = True
                                    
                                    self.arrivo[actuator_ids[i] -1 ] = True # Posizione raggiunta


                                    
                            elif(position[i]< self.__sensors_device[actuator_ids[i]-1].getValue()):
                                
                                self.__node.get_logger().info('posizione target[' + str(position[i])+'] < posizione rilevato dal sensore['+str(self.__sensors_device[actuator_ids[i]-1].getValue())+ ']')
                                # check_limite[actuator_ids[i]-1] = True
                                    
                                # if( abs(self.__sensors_device[actuator_ids[i]-1].getValue() - position[i]) > 0.01):
                                
                                if( (position[i] - self.__sensors_device[actuator_ids[i]-1].getValue() ) < -0.01):

                                    self.__positions[actuator_ids[i]-1] = position[i]
                                    self.__motors_device[actuator_ids[i]-1].setPosition(self.__positions[i])
                                    self.arrivo[actuator_ids[i] -1 ] = False #Ho settato una posizione quindi devo nuovermi , quindi arrivo è Falso
                                    final_goal[actuator_ids[i]-1] = False


                                else: 
                                    self.__node.get_logger().info('Obiettivo di posizione ' + str(self.__sensors_device[actuator_ids[i]-1].getValue()) + 'raggiunto')
                                    check_posizione[actuator_ids[i]-1] = False
                                    self.arrivo[actuator_ids[i] -1 ] = True # Posizione raggiunta
                                    final_goal[actuator_ids[i]-1] = True


                        else: 
                            
                            # response.message = response.message + str('posizione  [' +  str(position[i]) + ']  supera il limite ['+ str(limits_position[actuator_ids[i]-1]) +'] \n ')
                            response.success = response.success and False

                            
                            self.__node.get_logger().info('posizione['+ str(position[i])+'] per id [' +str(actuator_ids[i])+ '] non rispetta i limiti prestabiliti ')
                            self.__node.get_logger().info('non rispetta o il limite superiore['+ str(limits_position_sup[actuator_ids[i]-1]) + ']')
                            self.__node.get_logger().info('non rispetta o il limite inferiore['+ str(limits_position_inf[actuator_ids[i]-1]) + ']')

                            
                    else: # Caso di relative = True
                        
                        if( ( (position[i] + self.__sensors_device[actuator_ids[i]-1].getValue() ) <= limits_position_sup[actuator_ids[i]-1]) and  ((position[i] + self.__sensors_device[actuator_ids[i]-1].getValue() )>= limits_position_inf[actuator_ids[i]-1] ) ):  

                            # response.message = response.message + str( 'posizione  [' +  str(position[i]) + '+' +  str(self.__sensors_device[actuator_ids[i]-1].getValue()) + ']  corretto \n ')
                            response.success = response.success and True
                            print('sono dentro la posizione relativa')
                        
                            range_percorso[actuator_ids[i]-1] = abs(position[i]) # perchè è relative

                            if(valore_max < range_percorso[actuator_ids[i]-1]): 
                                
                                # valore_max = position[i]
                                valore_max = abs(position[i])
                                self.__node.get_logger().info('valore_max = ' + str(position[i]))

                                
                            self.__node.get_logger().info('posizione relativa ['+ str(position[i]) + '+' +  str(self.__sensors_device[actuator_ids[i]-1].getValue()) + '] per id [' + str(actuator_ids[i]) + '] inserito correttamente')
                            self.__node.get_logger().info('rispetta il limite superiore['+ str(limits_position_sup[actuator_ids[i]-1]) + ']')
                            self.__node.get_logger().info('rispetta il limite  inferiore['+ str(limits_position_inf[actuator_ids[i]-1]) + ']')

                            if(abs(position[i] + self.__sensors_device[actuator_ids[i]-1].getValue()) > 0.01 ):
                                
                                check_posizione[actuator_ids[i]-1] = True    #   Check sulla posizione richiesta superato  
 
                                self.__node.get_logger().info('posizione target[' + str(position[i])+'] > posizione rilevato dal sensore['+ 
                                str(self.__sensors_device[actuator_ids[i]-1].getValue())+ ']')
                                
                                self.__positions[actuator_ids[i]-1] = position[i]
                                self.__motors_device[actuator_ids[i]-1].setPosition(self.__positions[actuator_ids[i]-1] + self.__sensors_device[actuator_ids[i]-1].getValue())
                                self.__node.get_logger().info('settate le posizioni relative')
                            else:
                                self.__node.get_logger().info('posizione da settare troppo piccola')
                            
                            
                        else: 
                            
                            # response.message = response.message + str('posizione  [' +  str(position[i]) + '+' +  str(self.__sensors_device[actuator_ids[i]-1].getValue()) +']  supera il limite ['+ str(limits_position[actuator_ids[i]-1]) +'] |')
                            response.success = response.success and False
                            
                            self.__node.get_logger().info('posizione['+ str(position[i])+ '+' +  str(self.__sensors_device[actuator_ids[i]-1].getValue())+'] per id [' +str(actuator_ids[i])+ '] non rispetta i limiti prestabiliti ')
                            self.__node.get_logger().info('o non rispetta il limite superiore['+ str(limits_position_sup[actuator_ids[i]-1]) + ']')
                            self.__node.get_logger().info('o non rispetta il limite inferiore['+ str(limits_position_inf[actuator_ids[i]-1]) + ']')
      
                else:  
                    
                    # response.message = 'Id [' +  actuator_ids[i] + ']  non è compreso tra 1 e 8, \n'
                    self.__node.get_logger().info('Id [' + str(actuator_ids[i]) + '] inserito non rispetta il range ids prestabilito')
                    response.success = response.success and False
            self.__node.get_logger().info('\n')
            
            # Settaggio delle velocità le condizioni sono stato rispettate se sono qua dentro 
            
            if(velocity == 0.0): # Condizione al livello del primo ciclo for 
                
                
                # response.message = response.message + str( '['+str(actuator_ids[i])+']' + str('Velocità  [' +  str(velocity) + '], il robot non si muoverà \n'))
                self.__node.get_logger().info('Velocità inserita [' + str(velocity) + '] , il robot non si muoverà...')
                response.success = response.success and False
                check_velocita = True # Velocità ammessa
            #   NUOVO
            
            elif(velocity > max_speed_motors or velocity> max_speed_pliers):
                
                if(velocity > max_speed_motors and velocity < max_speed_pliers):
                    
                    self.__node.get_logger().info(str( '['+ str(actuator_ids[i])+']' + str('Velocità [' +  str(velocity) + '] supera il limite motori ['+ str(max_speed_motors) + '] \n')))
                    # response.message = response.message + str( '['+ str(actuator_ids[i])+']' + str('Velocità [' +  str(velocity) + '] supera il limite motori ['+ str(max_speed_motors) + '] \n'))
                    
                elif(velocity < max_speed_motors and velocity > max_speed_pliers):
                    
                    self.__node.get_logger().info(str( '['+str(actuator_ids[i])+']' + str('Velocità [' +  str(velocity) + '] supera il limite pinze ['+ str(max_speed_pliers) + '] \n')))
                    # response.message = response.message + str( '['+str(actuator_ids[i])+']' + str('Velocità [' +  str(velocity) + '] supera il limite pinze ['+ str(max_speed_pliers) + '] \n'))

                else:
                    self.__node.get_logger().info(str( '['+str(actuator_ids[i])+']' + str('Velocità [' +  str(velocity) + '] supera i limiti pinze e motori ['+ str(max_speed_pliers) + ',' + str(max_speed_motors) + '] \n')))
                    # response.message = response.message + str( '['+str(actuator_ids[i])+']' + str('Velocità [' +  str(velocity) + '] supera i limiti pinze e motori ['+ str(max_speed_pliers) + ',' + str(max_speed_motors) + '] \n'))

                response.success = response.success and False
                check_velocita = False # Velocità non rispetta i limiti 
                
            #   Faccio muovere i motori in maniera proporzionale... la velocità inserita è corretta
            
            else:   
                check_velocita =True
                # response.message = response.message +  str('Velocità [' +  str(velocity) + '] inserita \n')
                response.success = response.success and True
                
                
                print('è stata inserita una velocità per tutti i motori...')

            
                for i in range(len(actuator_ids)):  # Chiamata funzione ...
                    
                    # if(Funzioni.arrivo1(position[actuator_ids[i]-1],self.__sensors_device[actuator_ids[i]-1].getValue())):
                    if(Funzioni.ordinamento(check_id[actuator_ids[i]-1], check_posizione[actuator_ids[i]-1] , check_velocita)):

                                
                                if(valore_max == 0.0): # Inseriti posizioni correnti 
                                    scala = 0.0

                                else:
                                    scala = abs(range_percorso[actuator_ids[i]-1]/valore_max) # posizioni negative, la setVelocity prende solo valori positivi
                                
                                self.__node.get_logger().info('pronto per il settaggio della velocità id [' + str(actuator_ids[i] ) + ']')
                                self.__node.get_logger().info('limite di velocità id [' + str(actuator_ids[i] ) + '] = ' + str(limits_velocity[actuator_ids[i]-1]))
                                self.__node.get_logger().info('valore di riduzione scala per id ['+ str(actuator_ids[i] ) +'] = ' + str(scala))
                                
                                if(acceleration > 0.0):   # consigliabile comunque utilizzare accelerazioni superiori ad 1
                                    # self.__motors_device[actuator_ids[i]-1].setAcceleration(acceleration*scala)   ---> valore di scala diminuisce accelerazione
                                    self.__motors_device[actuator_ids[i]-1].setAcceleration(acceleration)  
                                    self.__node.get_logger().info('è consigliabile utilizzare accelerazione standard,')


                                self.__velocities[actuator_ids[i]-1] = scala * velocity
                                self.__motors_device[actuator_ids[i]-1].setVelocity(self.__velocities[actuator_ids[i]-1])  
                                response.success = response.success and False 
                                response.message = response.message + 'valore del sensore ' + str(i+1)+ ': ' + str(self.__sensors_device[i].getValue()) + ' \n '

                    else:
                        self.__node.get_logger().info('Non tutti i dati inseriti sono validi ... ')
                        response.success = response.success and True
                    
                    
                

                
        elif(len(actuator_ids) != len(position)):
            
            self.__node.get_logger().info('dimensione vettore ids e posizioni non coincide')
            self.__node.get_logger().info('dimensione actuator_ids ['+ str(len(actuator_ids)) +']   -   dimensione position ['+ str(len(position)) +']')
            response.success = False
        else:
            self.__node.get_logger().info('non hai ancora inserito niente!')
            response.success = False
                
        # Con la programmazione multithread viene letto anche il ciclo while all'interno dello spin di ROs2
        # while(contatore_prova < 100):
        #     self.__node.get_logger().info(" contatore_prova: " + str(contatore_prova))
        #     contatore_prova = contatore_prova +1
            
        self.__node.get_logger().info("Inizio ciclo while ... dopo questo messaggio il robot inizieràa a muoversi")

        
        response.success = response.success and Funzioni.control_check(final_goal)
        #response.message = response.message + 'valore di success: '+ str(response.success) + 'final goal ' + str(final_goal) +' \n '

        return response

    # ------------------------------------------------------- FUNZIONE CALLBACK SUBSCRIBER POSITION  --------------------------------------------------

    
    def __joint_position_callback(self,jointPosition):
        # self.__joint.name = position.name
        # self.__joint.position = position.position
        self.__joint_position = jointPosition

        
        if(len(self.__joint_position.name) >0  and  len(self.__joint_position.position)>0  and  (len(self.__joint_position.name) == len(self.__joint_position.position)) ):  #Se sono stati inseriti dei dati 
            
            #controllo il contenuto all'interno del vettore
            
            for i in range(len(self.__joint_position.name)):
                
                for j in range(len(self.motors_name)):
                    
                    if(jointPosition.name[i] == self.motors_name[j]):
                        
                        print('sono dentro if : \n ')
                        print(jointPosition.name[i] + ' confrontato con ' + self.motors_name[j])
                        
                        if(self.__joint_position.position[j] <= limits_position_sup[j] and self.__joint_position.position[j] >= limits_position_inf[j]):
                                                                
                                if(self.__joint_position.position[j] > self.__sensors_device[j].getValue() ):
                                    print('posizione target > posizione rilevato dal sensore ')
                                    
                                    if( abs(self.__joint_position.position[j] - self.__sensors_device[j].getValue() ) >0.01):
                                        self.__positions[j] = self.__joint_position.position[j]
                                        self.__motors_device[j].setPosition(self.__positions[j])
                                    else: 
                                        print('obiettivo raggiunto')
                                        self.__velocities[j] = 0
                                        self.__motors_device[j].setVelocity(self.__velocities[j])
                                
                                elif(self.__joint_position.position[j]< self.__sensors_device[j].getValue()):
                                    print('posizione targer < posizione rilevato dal sensore ')
                                    
                                    if( abs(self.__sensors_device[j].getValue() - self.__joint_position.position[j]) > 0.01):
                                        self.__positions[j] = self.__joint_position.position[j]
                                        self.__motors_device[j].setPosition(self.__positions[j])
                                    else: 
                                        print('obiettivo raggiunto')
                                        self.__velocities[j] = 0
                                        self.__motors_device[j].setVelocity(self.__velocities[j])
                                    
            
                        else:
                            print('il valore inserito non rispetta i limiti \n ')
                            print('non è stato possibile inserire i valori \n ')
                        
                    
                    if(jointPosition.name[i] == self.motors_name[j] ):     
        # self.__joint.position = position.position
                        print('Trovato una compatibilità con i nomi')
                    else:
                        print('Non cè una compatibilità con i nomi')
        elif(len(self.__joint_position.name) != len(self.__joint_position.position)):
            print('la dimensione dei due vettori non coincide')
        else:
            print('non hai ancora inserito niente ... ')
            
    # -------------------------------------------------------  FUNZIONE CALLBACK SUBSCRIBER VELOCITY --------------------------------------------------
   
    def __joint_velocity_callback(self, jointVelocity):

        self.__joint_velocity = jointVelocity
        if(len(self.__joint_velocity.name) >0  and  len(self.__joint_velocity.velocity)>0  and  (len(self.__joint_velocity.name) == len(self.__joint_velocity.velocity)) ):  #Se sono stati inseriti dei dati 
            
            #controllo il contenuto all'interno del vettore
            for i in range(len(self.__joint_velocity.name)):
                
                for j in range(len(self.motors_name)):
                    
                    if(jointVelocity.name[i] == self.motors_name[j]):
                        
                        if(self.__joint_velocity.velocity[j] <= limits_velocity[j]):
                            
                                self.__velocities[j] = self.__joint_velocity.velocity[j]
                                self.__motors_device[j].setVelocity(self.__velocities[j])

                        else:
                            print('il valore inserito non rispetta i limiti \n ')
                            print('non è stato possibile inserire i valori \n ')
   
        elif(len(self.__joint_velocity.name) != len(self.__joint_velocity.velocity)):
            print('la dimensione dei due vettori non coincide')

        else:
            print('valori inseriti non validi')
            
            
    def __sensor_values_callback(self):
        
        #+ str(actuator_ids[i] )
        messaggio = ''
        for i in range(len(self.__sensors_device)):
            messaggio = 'valore del sensore '+ str(i-1) + ':' + str(self.__sensors_device[i]) + '\n'
            
        self.__node.get_logger().info(messaggio)
        
          
    

    def step(self): # Funziona la lettura dei dati ottenuti dal sensore ....
        rclpy.spin_once(self.__node, timeout_sec=0)
        #self.executor.spin()
        
        # try:
        #     executor = MultiThreadedExecutor(num_threads = 6 )
        #     executor.add_node(self.__node)
        #     executor.spin()
        # finally:
        #     executor.shutdown()
            



    

        