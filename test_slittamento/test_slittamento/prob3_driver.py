import rclpy
# import sys
from fp_core_msgs.msg import JointPosition
from fp_core_msgs.srv import MoveJoint
from prob3_pack import Funzioni

msg= JointPosition()

limite_1_sup = 2.95903
limite_2_sup = 1.91148
limite_3_sup = 1.99927
limite_4_sup = 2.95153
limite_5_sup = 1.9977
limite_6_sup = 2.95833
limite_7_sup = 1.0472   
limite_8_sup = 1.0472


limite_1_inf = -2.95903
limite_2_inf = -1.91148
limite_3_inf = -1.99927
limite_4_inf = -2.95153
limite_5_inf = -1.9977
limite_6_inf = -2.95833
limite_7_inf = 0.0 
limite_8_inf = 0.0
TIME_STEP = 32 # milliseconds


# limiti di velcità

max_speed_motors= 1.74  # range che va da 0 a 1.74
max_speed_pliers = 1.0


limits_position_sup = []
limits_position_inf = []
limits_velocity = []
target_assoluto=[]

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

motors_name = []
sensors_name = []
arrivo = []
homing = False

actuator_ids = []
position = [] 
velocity = 0.0 # Settaggio della velocità iniziale per non far muovere il roboy in simulazione
acceleration = 2.0  # Valore ideale di accelerazione 
block = False       # Settaggio di default
relative = False    # Movimento di default -> variabili assolute


goal = False


for i in range(8): # Settaggio valori iniziali per la chiamata di simulazione

    # Riempimento vettore nomi motori
    
    target_assoluto.append(0.0)
    exec('limits_position_sup.append(limite_' + str(i+1) + '_sup)')
    exec('limits_position_inf.append(limite_' + str(i+1) + '_inf)')
    
    
    exec('motors_name.append(motor_'+ str(i+1) +'_name)')
      
    # RIempimento vettore nomi_sensori_motori
    exec('sensors_name.append(motor_'+ str(i+1) +'_sensor_name)')
    
    if(i < 6):
        limits_velocity.append(max_speed_motors)
        
    else:
        limits_velocity.append(max_speed_pliers)

    if(i <=6):
        msg.name.append(sensors_name[i])
        msg.position.append(0.0)
    else:
        msg.name.append('motor 8 sensor')
        msg.position.append(0.0)
        
    actuator_ids.append(i+1) # perchè gli ID vanno da 1 a 8 
    position.append(0.0) # valore che avrà il robot appena prima che l'utente chiama il servizio
    # range_assoluto.append(0.0)
    arrivo.append(False)

    # range_assoluto.append(0.0)

    
class Prob3Driver:
        
    def init(self, webots_node, properties): 
        
        
        rclpy.init(args=None)

        self.__robot = webots_node.robot
        self.__timestep = TIME_STEP

        #  Vettore oggetti motori 
        self.__motors_device = []
        
        # Vettore oggetti sensori 
        self.__sensors_device = []
        
        # Vettore posizioni iniziali motori, viene aggiornato ad ogni chiamate
        # per tenere d'occhio la posizione reale del robot
        self.__positions = []
        self.__rel_initial_positions = []
        
        # Vettore velocità iniziali motori
        
        self.__velocities = []
        
        for i in range(8): 
            
            #   Riempimento motori_device
            self.__motors_device.append(self.__robot.getDevice(motors_name[i]))
            
            #   Riempimento sensori_device
            self.__sensors_device.append(self.__robot.getDevice(sensors_name[i]))
            
            #   Abilitazione sensori
            self.__sensors_device[i].enable(self.__timestep)  
            
            # Riempimento vettore posizioni iniziali dei motori appena viene avviato P-Rob3  
            # -- POSIZIONE,POSIZIONE RELATIVA_INIZIALE  E VELCITA ALL'AVVIO DEL PLUGIN
            
            self.__positions.append(float('inf'))
            self.__rel_initial_positions.append(float('inf'))            
            self.__velocities.append(0.0)
            
            # Setup  motori_device , sensori_device 
            self.__motors_device[i].setPosition(self.__positions[i])
            self.__motors_device[i].setVelocity(self.__velocities[i])
            
            
            #Da fare delle prove
        
            self.__motors_device[i].setControlPID(10, 0, 0)

        # self.__motors_device[0].setPosition(1.0)
        # self.__motors_device[0].setVelocity(0.1)
            
        
        self.__node = rclpy.create_node('prob3_driver')
        
        self.__node.get_logger().info('------------P-Rob3 robot preso-------------')

        self.__node.get_logger().info('Inizio di prob3_driver')
        
        
        self.__node.create_service(MoveJoint, 'MoveJoint',self.servizio_movejoint)

        self.publisher_sensor_values = self.__node.create_publisher(JointPosition,'Sensors_values',10)   
        
        self.__node.get_logger().info('  P-Rob3 is supervisor? ' + str(self.__robot.getSupervisor()))
        
        
        
        # Nuove aggiunte 
        
        # self.__gripper_middle_distance_sensor = self.__robot.getDevice('gripper_middle_distance_sensor')
        # self.__gripper_middle_distance_sensor.enable(self.__timestep)

    def servizio_movejoint(self,request , response): # Codice solo per riceve new target
        
        
        global goal,arrivo,position,velocity,acceleration,block,relative

        self.__node.get_logger().info('------------------------------------')
        self.__node.get_logger().info('Parametri di ingresso : ')
        self.__node.get_logger().info(str(request.actuator_ids))
        self.__node.get_logger().info(str(request.position ))
        self.__node.get_logger().info(str(request.velocity))
        self.__node.get_logger().info(str(request.acceleration))
        self.__node.get_logger().info(str(request.block))
        self.__node.get_logger().info(str(request.relative)) 
        self.__node.get_logger().info('------------------------------------')
        
        
        token_1 = Funzioni.controll_data(request,limits_position_inf,limits_position_sup,limits_velocity,self.__rel_initial_positions)

                
        goal = token_1.check_posizione # True se i dati inseriti sono validi, altrimenti false
            
  
        response.message = response.message + ' \n '
                
        scala = 0.0
        valore_max = 0.0
        
        velocity = request.velocity
        acceleration = request.acceleration
        block = request.block
        relative = request.relative  
        
        target_assoluto = []
        range_assoluto = []
        
        
        if(goal == True):
            
        
            for i in range(len(actuator_ids)):  # Ciclo che deve essere eseguito sempre
                
                # Prova calcolo velocità di ogni motore 
                actuator_ids[i] = request.actuator_ids[i]
                position[i] = request.position[i]
                
                range_assoluto.append(0.0)
                target_assoluto.append(0.0)
                
                if(velocity == 0.0):
                    
                    self.__node.get_logger().info('la velocita  -> 0.0')
                    
                    arrivo[i] = True
                    self.__rel_initial_positions[i] = round(self.__sensors_device[actuator_ids[i]-1].getValue(),4) # Se questo funziona ho già le posizioni iniziali
                    self.__positions[i] = round(self.__sensors_device[actuator_ids[i]-1].getValue(),4) 

                    self.__velocities[actuator_ids[i]-1] = request.velocity
                    
                    target_assoluto[i] = round(self.__sensors_device[actuator_ids[i]-1].getValue(),4)
                    
                                        
                    self.__motors_device[actuator_ids[i]-1].setPosition(target_assoluto[i])
                    self.__motors_device[actuator_ids[i]-1].setVelocity(request.velocity)
                

                    self.__node.get_logger().info(str(self.__rel_initial_positions[actuator_ids[i]-1]))
             
                                        
         
                else:
                  
                    arrivo[i] = False
                    
                    self.__rel_initial_positions[i] = self.__sensors_device[actuator_ids[i]-1].getValue()
                    self.__positions[i] = self.__sensors_device[actuator_ids[i]-1].getValue()
                    
  
                    if(relative == False):
                                
                        range_assoluto[i] = abs(position[i] - round(self.__sensors_device[i].getValue(),4))
                        target_assoluto[i] = position[i]
                    else:
                        
                        range_assoluto[i] = abs(position[i] ) # Range ASSOLUTO da percorrere
                        
                        # target_assoluto[i] = position[i] + self.__motors_device[i].getTargetPosition()
                        target_assoluto[i] = position[i] + round(self.__sensors_device[actuator_ids[i]-1].getValue(),4)

                    if(valore_max < range_assoluto[i]): 
                                        
                        valore_max = abs(range_assoluto[i]) 
                        
                    # Range assoluto coincide con valore_max 
                                
            if(velocity == 0.0):
                
                self.__node.get_logger().info('Sono dentro velocity ==0')

                self.__node.get_logger().info('vettore posizioni settate: ' + str(target_assoluto))
                self.__node.get_logger().info('vettore velocita  settate: ' + str(self.__velocities) )
                self.__node.get_logger().info('vettore accelerazioni  settate: ' + str(acceleration) ) 
                goal = False

            
            if(velocity != 0.0):
                
                for i in range(len(actuator_ids)):    #Ciclo for per il sessaggio dei parametri 
                    
                
                        if(valore_max == 0.0): # Inseriti posizioni correnti     
                            scala = 0.0
                            
                        else:
                            scala = abs(range_assoluto[actuator_ids[i]-1 ]/valore_max) # posizioni negative, la setVelocity prende solo valori positivi
                
                        self.__velocities[actuator_ids[i]-1] = scala * velocity
                            
                        # Settaggio dei parametri per far muovere il robot
                        

                            
                        self.__motors_device[actuator_ids[i]-1].setPosition(round(target_assoluto[actuator_ids[i]-1],4))

                        self.__motors_device[actuator_ids[i]-1].setVelocity(self.__velocities[actuator_ids[i]-1]) 
                            
                        self.__motors_device[actuator_ids[i]-1].setAcceleration(acceleration)
                        
                self.__node.get_logger().info('Sono dentro velocity !=0')

                self.__node.get_logger().info('vettore posizioni settate: ' + str((target_assoluto)) )
                self.__node.get_logger().info('vettore velocita  settate: ' + str(self.__velocities) )
                self.__node.get_logger().info('vettore accelerazioni  settate: ' + str(acceleration) )
                
            

                                                                  
        response.success = token_1.check_posizione
        response.message = token_1.messaggio
            
        return response
    
    def Fase_di_Homing(self):
        
        range_assoluto = []

        for i in range(len(self.__positions)): # Setto le velocita per l'homing
            
            
            range_assoluto.append(0.0)
            self.__velocities[i] = 0.1
            
             # In questo modo lo faccio una volta sola     
                       
            self.__motors_device[i].setPosition(range_assoluto[i])
            self.__motors_device[i].setVelocity(self.__velocities[i])
            self.__motors_device[i].setAcceleration(acceleration) # costante
          
    
    def step(self): 
                
        rclpy.spin_once(self.__node, timeout_sec=0)
                    
        global goal,homing
        
        rclpy.spin_once(self.__node, timeout_sec=0)
        

        for i in range(len(sensors_name)):  # Per pubblicare i valori dei sensori 
            
            msg.position[i] = self.__sensors_device[i].getValue()
            
        self.publisher_sensor_values.publish(msg) # Il sensore pubblica i valori del sensore in questo step...
            
            

            
        # Fase di HOMING
            
        if ( homing == False and goal == False ):
            
            self.__node.get_logger().info(" timestep Homing : " + str(self.__robot.getTime()))

            self.Fase_di_Homing()
            

            if(Funzioni.waiting_arrive(arrivo)):
                
                print('Sono arrivato alla posizione di setup')
                homing = True
                self.__node.get_logger().info(" timestep Fine Homing : " + str(self.__robot.getTime()))

            else:

                for i in range(len(self.__positions)):

                    if(round(abs(self.__sensors_device[i].getValue() - self.__motors_device[i].getTargetPosition()),4) == 0.0):
                        
                        arrivo[i] = True
                        self.__node.get_logger().info("sensore ["+str(i)+"]  arrivato.")
        
        elif(homing == True and goal == True): # Richiesta dal client accettata
            
            
            self.__node.get_logger().info("sono dentro goal = true e homing = true")
            
            if(Funzioni.waiting_arrive(arrivo)): # Sono arrivato alla posizione target.
                 goal = False
                        
            else:
                                                        
                for i in range(len(actuator_ids)):
                                    
                    if(round(abs(self.__sensors_device[actuator_ids[i]-1].getValue() - self.__motors_device[actuator_ids[i]-1].getTargetPosition()),4) == 0.0):
                        
                        arrivo[actuator_ids[i]-1] = True
                        self.__node.get_logger().info("sensore ["+str(i+1)+"]  arrivato alla posizione target")
                        
                    
        elif(homing == False and goal == False )  :
            
            print('in attesta di una richiesta da parte del client') 
        
                    