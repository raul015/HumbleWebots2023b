import rclpy
# import sys

from fp_core_msgs.msg import JointPosition
from fp_core_msgs.msg import JointVelocity
from fp_core_msgs.srv import MoveJoint
from prob3_pack import Funzioni



# limiti di posizione, vale sia per valori positivi che negativi
# I limiti non valgono in valore assoluto per le pinze...
# vanno da 0 a 1.0472 perché 0 rappresenta il loro punto di contatto,
# con valori negativi si distruggono...

msg= JointPosition()

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

for i in range(8): # Settaggio valori iniziali per la chiamata di simulazione

    # Riempimento vettore nomi motori
    
    target_assoluto .append(0.0)
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
        
    
    arrivo.append(False)

class MyRobotDriver:
        
    def init(self, webots_node, properties): 
        
        #APPENA LANCIATO SUCCEDE QUESTO 
        
        self.homing = False

        self.__robot = webots_node.robot
        self.__timestep = int(32)
        
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

        
        rclpy.init(args=None)
        
        self.__node = rclpy.create_node('prob3_driver')
    
        self.__node.get_logger().info('Inizio di prob3_driver')
        
        self.__node.create_service(MoveJoint, 'MoveJoint',self.servizio_movejoint)
        
        # JointPosition
        #string[] name
        #float64[] position
        
        self.publisher_sensor_values = self.__node.create_publisher(JointPosition,'Sensors_values',10)

    #   ------------------------------------------------------- FUNZIONE SERVICE --------------------------------------------------
    
    def servizio_movejoint(self,request , response): # Codice solo per riceve new target
        
        print('sono dentro al servizio')
        actuator_ids = request.actuator_ids
        position = request.position
        velocity = request.velocity
        acceleration = request.acceleration
        block = request.block
        relative = request.relative    
        
        response.message = response.message + ' \n '
        
        range_assoluto = []    # Spostamento in valore assoluto, quindi distanza percorsa
        # target_assoluto = []
        
        valore_max = 0.0
        scala = 0.0
        
        for i in range(8):  
            
            arrivo[i] = False
            range_assoluto.append(0.0)
            self.__rel_initial_positions[i] = self.__motors_device[i].getTargetPosition() # Se questo funziona ho già le posizioni iniziali
            self.__positions[i] = self.__motors_device[i].getTargetPosition()
            
            # Prova calcolo velocità di ogni motore 
            if(relative == False):
                        
                range_assoluto[i] = abs(position[i] - self.__sensors_device[i].getValue())

                if(valore_max < range_assoluto[i]): 
                                
                    valore_max = abs(position[i] - self.__sensors_device[i].getValue())
                    
            else:
                
                range_assoluto[i] = abs(position[i] ) # Range ASSOLUTO da percorrere
         
         
                if(valore_max < range_assoluto[i]):  # confronto posizioni target

                    valore_max = abs(range_assoluto[i])
                                
            self.__motors_device[i].setVelocity(0.0) 

            # target_assoluto .append(0.0)
                    
        self.__node.get_logger().info('------------------------------------')
        self.__node.get_logger().info('Parametri di ingresso : ')
        self.__node.get_logger().info(str(actuator_ids))
        self.__node.get_logger().info(str(position ))
        self.__node.get_logger().info(str(velocity))
        self.__node.get_logger().info(str(acceleration))
        self.__node.get_logger().info(str(block))
        self.__node.get_logger().info(str(relative)) 
        self.__node.get_logger().info('------------------------------------')

        # Controlli teorici --> quindi messaggio di errore solo teorico, quindi non permette di andare avanti teoricamente !!!
        
        #self.__rel_initial_positions settata nella fase di homing
        
        token = Funzioni.controllo_dati_ingresso(actuator_ids,position,velocity,limits_position_sup,limits_position_inf,limits_velocity,relative,self.__rel_initial_positions,arrivo)

        if(token.check_posizione == True): # Controllo che le posizioni target siano valide...
            
            #controllo il contenuto all'interno del vettore
            
            for i in range(len(actuator_ids)):
                                                        
                    if(relative == False):
                        
                            # range_assoluto[actuator_ids[i]-1] = abs(position[i] - self.__sensors_device[actuator_ids[i]-1].getValue())

                            # if(valore_max < range_assoluto[actuator_ids[i]-1]): 
                                
                            #     valore_max = abs(position[i] - self.__sensors_device[actuator_ids[i]-1].getValue())
                                                
                            if(round(abs(self.__sensors_device[actuator_ids[i]-1].getValue() - position[i]),4) == 0.0):
                                                                    
                                arrivo[actuator_ids[i] -1 ] = True # Posizione target non raggiunta
                        
                                    
                            else: 
                                #Prova posizione
                                arrivo[actuator_ids[i] -1 ] = False # Posizione target raggiunta
                                self.__motors_device[actuator_ids[i]-1].setPosition(position[i])

                    else:   # Relative true ------------------------- Controllo questa parte 
                                                
                            # range_assoluto[actuator_ids[i]-1 ] = abs(position[actuator_ids[i]-1] ) # Range ASSOLUTO da percorrere

                            # self.__node.get_logger().info('prima di calcolo target assoluto')
                            # self.__node.get_logger().info(' position [' + str(position[actuator_ids[i] -1 ]) + '] + rel_ini_pos ['+str( self.__rel_initial_positions[actuator_ids[i] -1 ] )+']')

                                
                            # pos assoluta         # posizine relative          # posizioine iniziale 
                            target_assoluto[actuator_ids[i] -1 ] = position[actuator_ids[i] -1 ] + self.__rel_initial_positions[actuator_ids[i] -1 ] # posizione_target + posizione iniziale - PROBLEMA

                            #Valore max inizialmente a 0
                            
                            # if(valore_max < range_assoluto[actuator_ids[i] -1 ]):  # confronto posizioni target

                            #     valore_max = abs(range_assoluto[actuator_ids[i]-1])
                                
                            #     self.__node.get_logger().info('--------------------------------------------------')

                            #     self.__node.get_logger().info('valore_max = ' + str(range_assoluto[actuator_ids[i] -1 ]))
                            #     self.__node.get_logger().info('target_assoluto[' + str(round(target_assoluto[actuator_ids[i] -1 ],3)) +'] = position [' + str(position[actuator_ids[i] -1 ]) + '] + rel_ini_pos ['+str( self.__rel_initial_positions[actuator_ids[i] -1 ] )+']')
                            #     self.__node.get_logger().info('condizione: ' + str((target_assoluto[actuator_ids[i] -1 ]) - self.__sensors_device[i].getValue()))


                            #     self.__node.get_logger().info('--------------------------------------------------')

                            
                            if( round(  abs(target_assoluto[actuator_ids[i] -1 ] - self.__sensors_device[actuator_ids[i]-1].getValue()) ,4) ==  0.0 ):
                                
                                arrivo[actuator_ids[i]-1] = True
            
                            else:

                                arrivo[actuator_ids[i] -1 ] = False # Posizione raggiunta
                                self.__motors_device[actuator_ids[i]-1].setPosition(round(target_assoluto[actuator_ids[i]-1],3))
                                                                                         
                        
                    if(valore_max == 0.0): # Inseriti posizioni correnti 
                        scala = 0.0

                    else:
                        scala = abs(range_assoluto[actuator_ids[i] -1 ]/valore_max) # posizioni negative, la setVelocity prende solo valori positivi

                                
                    if(acceleration > 0.0):   # consigliabile comunque utilizzare accelerazioni superiori ad 1
                            
                        self.__motors_device[actuator_ids[i]-1].setAcceleration(acceleration)  
                        self.__node.get_logger().info('è consigliabile utilizzare accelerazione standard')

                    self.__velocities[actuator_ids[i]-1] = scala * velocity
                        
            while(Funzioni.waiting_arrive(arrivo) == False and self.__robot.step(32) != -1):
                            
                for j in range(len(actuator_ids)):
                    
                    self.__motors_device[actuator_ids[j]-1].setVelocity(self.__velocities[actuator_ids[j]-1]) 
                    # response.message = response.message + 'valore del sensore ' + str(j+1)+ ': ' + str(self.__sensors_device[actuator_ids[j]-1].getValue()) + ' \n '
                    if(round(abs(self.__sensors_device[actuator_ids[j]-1].getValue() - self.__motors_device[actuator_ids[j]-1].getTargetPosition()),4) == 0.0):
                        arrivo[actuator_ids[j]-1] = True
                        self.__node.get_logger().info("sensore ["+str(j)+"]  arrivato.")
                    # Per leggere i sensori mentre il robot si muove
                    #msg.name[j] = sensors_name[j]
                    msg.position[j] = self.__sensors_device[j].getValue()  
                self.__node.get_logger().info("")
                
                # Invio dei dati...
                self.publisher_sensor_values.publish(msg) 

            for j in range(len(actuator_ids)):
                self.__motors_device[actuator_ids[j]-1].setVelocity(0.0) 
                arrivo[actuator_ids[j]-1] = False
                response.success = True
                self.__rel_initial_positions[actuator_ids[j]-1] = self.__motors_device[actuator_ids[j]-1].getTargetPosition()
             
                                         
        else:
            print(token.messaggio)
            self.__node.get_logger().info(str(token.messaggio))
            response.success = False
            
            response.message = 'errore'
                
        self.__node.get_logger().info(str(response.success))

        return response
    
                
            
    def Fase_di_Homing(self):
        
        Homing_check = True  # Fisso a ture
        
        pos_homing = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        vel_homing = 0.1

        self.__node.get_logger().info(" timestep INIZIO Homing : " + str(self.__robot.getTime()))

        while(Funzioni.waiting_arrive(arrivo) == False and self.__robot.step(32) != -1 ):
            
            for i in range(len(self.__positions)):
                
                self.__node.get_logger().info("dentro ciclo while")

                self.__motors_device[i].setPosition(pos_homing[i])
                self.__motors_device[i].setVelocity(vel_homing)
                if(round(abs(self.__sensors_device[i].getValue() - self.__motors_device[i].getTargetPosition()),4) == 0.0):
                    arrivo[i] = True
                    self.__node.get_logger().info("sensore ["+str(i)+"]  arrivato.")

                else:
                    
                    Homing_check = Homing_check and False
                    
                self.__node.get_logger().info("sensore ["+str(i)+"] : " + str(self.__sensors_device[i].getValue() ))       
                self.__node.get_logger().info("step : " + str(self.__robot.getTime()))

        
        
        self.__node.get_logger().info(" timestep FINE  Homing : " + str(self.__robot.getTime()))
        
        self.homing = True
        
        return Homing_check
                                      
    
    def step(self): # Funziona la lettura dei dati ottenuti dal sensore ....
        rclpy.spin_once(self.__node, timeout_sec=0)
        

        for j in range(len(sensors_name)):
            #msg.name[j] = sensors_name[j]
            msg.position[i] = self.__sensors_device[j].getValue()
        
        self.publisher_sensor_values.publish(msg)
        if(self.homing == False):
            self.Fase_di_Homing()
            
