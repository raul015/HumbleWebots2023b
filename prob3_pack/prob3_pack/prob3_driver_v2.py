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

# Riempimento dei vettori limite posizione e velocità

# for i in range(8): 
#     target_assoluto .append(0.0)
#     exec('limits_position_sup.append(limite_' + str(i+1) + '_sup)')
#     exec('limits_position_inf.append(limite_' + str(i+1) + '_inf)')
    
#     if(i < 6):
#         limits_velocity.append(max_speed_motors)
#     else:
#         limits_velocity.append(max_speed_pliers)
        

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
    
    if(i < 6):
        limits_velocity.append(max_speed_motors)
    else:
        limits_velocity.append(max_speed_pliers)
        

    exec('motors_name.append(motor_'+ str(i+1) +'_name)')
      
    # RIempimento vettore nomi_sensori_motori
    exec('sensors_name.append(motor_'+ str(i+1) +'_sensor_name)')
    
    arrivo.append(False)

class MyRobotDriver:
        
    def init(self, webots_node, properties): 
        
        #APPENA LANCIATO SUCCEDE QUESTO 
        
        self.homing = False

                        
        self.__robot = webots_node.robot
        self.__timestep = int (self.__robot.getBasicTimeStep())
        
        
        # self.__camera = self.__robot.getDevice("camera")
        # self.__camera.enable(self.__timestep)
        # Prova inserimento telecamera tra le pinze del robot: 
        # self.__cliente = self.create_client(MoveJoint ,'Movimento')
                
        # Inizializzazione vettore dei nomi dei motori    
        
        # self.motors_name = []
        # self.sensors_name = []
        
        # self.arrivo = []
        
        # self.arrivo.append(False)    #1
        # self.arrivo.append(False)    #2
        # self.arrivo.append(False)    #3
        # self.arrivo.append(False)    #4
        # self.arrivo.append(False)    #5
        # self.arrivo.append(False)    #6
        # self.arrivo.append(False)    #7
        # self.arrivo.append(False)    #8
        
        
        # for i in range(8): 

        #     # Riempimento vettore nomi motori
        #     exec('self.motors_name.append(motor_'+ str(i+1) +'_name)')
            
        #     # RIempimento vettore nomi_sensori_motori
        #     exec('self.sensors_name.append(motor_'+ str(i+1) +'_sensor_name)')

    
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
            
            # Riempimento vettore posizioni iniziali dei motori appena viene avviato P-Rob3   -- POSIZIONE,POSIZIONE RELATIVA_INIZIALE  E VELCITA ALL'AVVIO DEL PLUGIN
            
            self.__positions.append(float('inf'))
            self.__rel_initial_positions.append(float('inf'))            
            self.__velocities.append(0.0)
            
            # Setup  motori_device , sensori_device 
            self.__motors_device[i].setPosition(self.__positions[i])
            self.__motors_device[i].setVelocity(self.__velocities[i])
            
            
            #Da fare delle prove
            
            self.__motors_device[i].setControlPID(8, 0, 0)

        self.__joint_position = JointPosition()
        self.__joint_velocity = JointVelocity()
        
        rclpy.init(args=None)
        
        self.__node = rclpy.create_node('prob3_driver')
                
        self.__node.get_logger().info('Inizio di prob3_driver')
        
        self.__node.create_subscription(JointPosition, 'Joints_position', self.__joint_position_callback, qos_profile =1)

        self.__node.create_subscription(JointVelocity, 'Joints_velocity', self.__joint_velocity_callback, qos_profile = 1)

        self.__node.create_service(MoveJoint, 'MoveJoint',self.servizio_movejoint)

    #   ------------------------------------------------------- FUNZIONE SERVICE --------------------------------------------------
    
    def servizio_movejoint(self,request , response):
        
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
            
            range_assoluto.append(0.0)  #

            if(relative == False):
                
                range_assoluto[i] = abs(position[i] - self.__sensors_device[i].getValue())
                
                if(valore_max < range_assoluto[i]): 
                                
                    valore_max = abs(position[i] - self.__sensors_device[i].getValue())
            else:
                
                range_assoluto[i] = abs(position[i] ) # Range ASSOLUTO da percorrere
                
                if(valore_max < range_assoluto[i]):  # confronto posizioni target

                                
                    valore_max = abs(range_assoluto[i])
                                
                
                
            # target_assoluto .append(0.0)
                    
        # valore_max = 0.0
        # scala = 0.0
        
        self.__node.get_logger().info('Parametri di ingresso : ')
        self.__node.get_logger().info(str(actuator_ids))
        self.__node.get_logger().info(str(position ))
        self.__node.get_logger().info(str(velocity))
        self.__node.get_logger().info(str(acceleration))
        self.__node.get_logger().info(str(block))
        self.__node.get_logger().info(str(relative)) 
        self.__node.get_logger().info('------------------------------------')

        
        #   controllo che la lunghezza dati, id giusti, posizioni giuste , velocità giuste per tutti i motori...
        #token = Funzioni.controllo_dati_ingresso(actuator_ids,position,velocity,limits_position_sup,limits_position_inf,limits_velocity,relative,self.__sensors_device)
        
        
        
        # Controlli teorici --> quindi messaggio di errore solo teorico, quindi non permette di andare avanti teoricamente !!!
        
        self.__node.get_logger().info('* * * ** * * ')

        self.__node.get_logger().info(str( self.__rel_initial_positions[actuator_ids[i] -1 ] ))
        self.__node.get_logger().info('* * * ** * * ')


        token = Funzioni.controllo_dati_ingresso(actuator_ids,position,velocity,limits_position_sup,limits_position_inf,limits_velocity,relative,self.__rel_initial_positions,arrivo)

        if(token.check_posizione == True):
            
            #controllo il contenuto all'interno del vettore
            
            for i in range(len(actuator_ids)):
                                                        
                    if(relative == False):
                        
                            # range_assoluto[actuator_ids[i]-1] = abs(position[i] - self.__sensors_device[actuator_ids[i]-1].getValue())

                            # if(valore_max < range_assoluto[actuator_ids[i]-1]): 
                                
                            #     valore_max = abs(position[i] - self.__sensors_device[actuator_ids[i]-1].getValue())
                                                
                            if(abs(position[i] - self.__sensors_device[actuator_ids[i]-1].getValue()) >=  0.001):
                                                                    
                                self.__positions[actuator_ids[i]-1] = position[i]
                                self.__motors_device[actuator_ids[i]-1].setPosition(self.__positions[actuator_ids[i]-1])
                                arrivo[actuator_ids[i] -1 ] = False # Posizione target non raggiunta
                                        
                            else: 
                                #Prova posizione
                                arrivo[actuator_ids[i] -1 ] = True # Posizione target raggiunta
                            
                    else:   # Relative true
                                                
                            # range_assoluto[actuator_ids[i]-1 ] = abs(position[actuator_ids[i]-1] ) # Range ASSOLUTO da percorrere

                            self.__node.get_logger().info('prima di calcolo target assoluto')
                            self.__node.get_logger().info(' position [' + str(position[actuator_ids[i] -1 ]) + '] + rel_ini_pos ['+str( self.__rel_initial_positions[actuator_ids[i] -1 ] )+']')

                                
                            # pos assoluta         # posizine relative          # posizioine iniziale 
                            target_assoluto[actuator_ids[i] -1 ] = position[actuator_ids[i] -1 ] + self.__rel_initial_positions[actuator_ids[i] -1 ] # posizione_target + posizione iniziale - PROBLEMA

                            #Valore max inizialmente a 0
                            
                            # if(valore_max < range_assoluto[actuator_ids[i] -1 ]):  # confronto posizioni target

                                
                                # valore_max = abs(range_assoluto[actuator_ids[i]-1])
                                
                                # self.__node.get_logger().info('--------------------------------------------------')

                                # self.__node.get_logger().info('valore_max = ' + str(range_assoluto[actuator_ids[i] -1 ]))
                                # self.__node.get_logger().info('target_assoluto[' + str(round(target_assoluto[actuator_ids[i] -1 ],3)) +'] = position [' + str(position[actuator_ids[i] -1 ]) + '] + rel_ini_pos ['+str( self.__rel_initial_positions[actuator_ids[i] -1 ] )+']')
                                # self.__node.get_logger().info('condizione: ' + str((target_assoluto[actuator_ids[i] -1 ]) - self.__sensors_device[i].getValue()))


                                # self.__node.get_logger().info('--------------------------------------------------')


                            if( abs( (target_assoluto[actuator_ids[i] -1 ]) - self.__sensors_device[actuator_ids[i]-1].getValue() ) >=  0.0001 ):
                                
                                self.__motors_device[actuator_ids[i]-1].setPosition(round(target_assoluto[actuator_ids[i]-1],3))
                                arrivo[actuator_ids[i]-1] = False
            
                            else:

                                #Aggiornamento posizione relativa 
                                arrivo[actuator_ids[i] -1 ] = True # Posizione raggiunta
                                # self.__motors_device[i].setVelocity(0.0)

                                # self.__positions[actuator_ids[i]-1] = round(self.__sensors_device[actuator_ids[i]-1].getValue(),2) # Setto la posizione reale del robot
                                #self.__motors_device[actuator_ids[i]-1].setPosition(position[i] + self.__relative_positions[actuator_ids[i]-1])
                    
                    if(Funzioni.waiting_arrive(arrivo) == True ): # Tutti i motori sono arrivati
                        
                        response.success = True        
                        valore_max = 0.0   #Sto resettando il valore max, altrimenti rimane quello più grande
      
                        self.__node.get_logger().info('valori di response success e message ')
                        self.__node.get_logger().info(str(response.success))
                        self.__node.get_logger().info(response.message)


                        self.__node.get_logger().info('Aggiornamento condizioni iniziali')

                        if(relative == True):
                            
                            for j in range(len(actuator_ids)):
                                
                                self.__rel_initial_positions[actuator_ids[j]-1] = round(target_assoluto[actuator_ids[j] -1 ],3) # aggirnamento delle condizioni iniziale per il prossimo ciclo
                                self.__node.get_logger().info(str(self.__rel_initial_positions[actuator_ids[j] -1 ] ))

                                #Aggiornamento delle condizioni iniziali relative 
                                
                        else:
                            for j in range(len(actuator_ids)):
                                
                                self.__rel_initial_positions[actuator_ids[j] -1 ] = position[actuator_ids[j] -1 ] # Qui avrò sempre valori a due cifre, per ora in quanto sono cordinate assolute                    
                    
                        break
                    else:
                        
                        if(valore_max == 0.0): # Inseriti posizioni correnti 
                            scala = 0.0

                        else:
                            scala = abs(range_assoluto[actuator_ids[i] -1 ]/valore_max) # posizioni negative, la setVelocity prende solo valori positivi

                                
                        if(acceleration > 0.0):   # consigliabile comunque utilizzare accelerazioni superiori ad 1
                            
                            self.__motors_device[actuator_ids[i]-1].setAcceleration(acceleration*scala)  
                            self.__node.get_logger().info('è consigliabile utilizzare accelerazione standard')

                        self.__velocities[actuator_ids[i]-1] = scala * velocity
                        if(arrivo[actuator_ids[i]-1] == True):
                            self.__motors_device[actuator_ids[i]-1].setVelocity(0.0)
                        else:
                            self.__motors_device[actuator_ids[i]-1].setVelocity(self.__velocities[actuator_ids[i]-1]) 
                        
                        response.success = False
             
            for i in range(len(actuator_ids)): 
                response.message = response.message + 'valore del sensore ' + str(i+1)+ ': ' + str(self.__sensors_device[actuator_ids[i]-1].getValue()) + ' \n '
                if(arrivo[actuator_ids[i]-1] == True):
                    self.__motors_device[actuator_ids[i]-1].setVelocity(0.0)
                                          
        else:
            print(token.messaggio)
            self.__node.get_logger().info(str(token.messaggio))
            response.success = False
            
            response.message = 'errore'
                
        self.__node.get_logger().info(str(response.success))

        return response
    
    

    # ------------------------------------------------------- FUNZIONE CALLBACK SUBSCRIBER POSITION  --------------------------------------------------

    
    def __joint_position_callback(self,jointPosition):
        # self.__joint.name = position.name
        # self.__joint.position = position.position
        self.__joint_position = jointPosition

        
        if(len(self.__joint_position.name) >0  and  len(self.__joint_position.position)>0  and  (len(self.__joint_position.name) == len(self.__joint_position.position)) ):  #Se sono stati inseriti dei dati 
            
            #controllo il contenuto all'interno del vettore
            
            for i in range(len(self.__joint_position.name)):
                
                for j in range(len(motors_name)):
                    
                    if(jointPosition.name[i] == motors_name[j]):
                        
                        print('sono dentro if : \n ')
                        print(jointPosition.name[i] + ' confrontato con ' + motors_name[j])
                        
                        if(self.__joint_position.position[j] <= limits_position_sup[j] and self.__joint_position.position[j] >= limits_position_inf[j]):
                                                                
                                if(self.__joint_position.position[j] > self.__sensors_device[j].getValue() ):
                                    print('posizione target > posizione rilevato dal sensore ')
                                    
                                    if( abs(self.__joint_position.position[j] - self.__sensors_device[j].getValue() ) >=0.001):
                                        self.__positions[j] = self.__joint_position.position[j]
                                        self.__motors_device[j].setPosition(self.__positions[j])
                                    else: 
                                        print('obiettivo raggiunto')
                                        self.__velocities[j] = 0
                                        self.__motors_device[j].setVelocity(self.__velocities[j])
                                
                                elif(self.__joint_position.position[j]< self.__sensors_device[j].getValue()):
                                    print('posizione targer < posizione rilevato dal sensore ')
                                    
                                    if( abs(self.__sensors_device[j].getValue() - self.__joint_position.position[j]) >= 0.001):
                                        self.__positions[j] = self.__joint_position.position[j]
                                        self.__motors_device[j].setPosition(self.__positions[j])
                                    else: 
                                        print('obiettivo raggiunto')
                                        self.__velocities[j] = 0
                                        self.__motors_device[j].setVelocity(self.__velocities[j])
                                    
            
                        else:
                            print('il valore inserito non rispetta i limiti \n ')
                            print('non è stato possibile inserire i valori \n ')
                        
                    
                    if(jointPosition.name[i] == motors_name[j] ):     
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
                
                for j in range(len(motors_name)):
                    
                    if(jointVelocity.name[i] == motors_name[j]):
                        
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
            
            
    def Fase_di_Homing(self):
        
        Homing_check = True  # Fisso a ture
        
        pos_homing = 0.0
        vel_homing = 0.1

        for i in range(len(self.__positions)):
            
            self.__motors_device[i].setPosition(pos_homing)
            self.__motors_device[i].setVelocity(vel_homing)
            
            if(abs(pos_homing - self.__sensors_device[i].getValue()) >= 0.001 ):
                
                self.__motors_device[i].setPosition(pos_homing)
                self.__motors_device[i].setVelocity(vel_homing)
                
                self.__node.get_logger().info( '['+str(i)+' ]dentro >= 000.1 : ')
                self.__node.get_logger().info(str(self.__sensors_device[i].getValue()))

                Homing_check = Homing_check and False
                
            else:
                
                self.__node.get_logger().info('dentro else, fine homing')
                self.__motors_device[i].setVelocity(self.__velocities[i])
                self.__rel_initial_positions[i] = 0.0
                self.__positions[i] = 0.0
                self.__motors_device[i].setVelocity(self.__positions[i])
            


        if(Homing_check == True):
            self.homing = True
        
        return Homing_check
                                      
    
    def step(self): # Funziona la lettura dei dati ottenuti dal sensore ....
        rclpy.spin_once(self.__node, timeout_sec=0)
        
        if(self.homing == False):
            self.Fase_di_Homing()
        #self.executor.spin()
        
        # try:
        #     executor = MultiThreadedExecutor(num_threads = 6 )
        #     executor.add_node(self.__node)
        #     executor.spin()
        # finally:
        #     executor.shutdown()

        