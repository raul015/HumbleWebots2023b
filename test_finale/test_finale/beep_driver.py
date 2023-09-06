import rclpy
import time
from random import randint
from fp_core_msgs.srv import SetFloat
from std_msgs.msg import Float64
from std_msgs.msg import Int64
#from fp_core_msgs.srv import RequestData
import optparse
import math
import time

TIME_STEP = 32 # milliseconds

BODY_PARTS_NUMBER = 13
WALK_SEQUENCES_NUMBER = 8
ROOT_HEIGHT = 0.3
CYCLE_TO_DISTANCE_RATIO = 0.32

speed = 1.15
current_height_offset = 0

oggetto_xyz = []


# Definisco i target che l'uomo deve raggiungere quando inizio la simulazione...

lista_target_posizione = []
lista_target_angolo = []

target_posizione_linea = [0.3, -2.85, 0.3]
target_posizione_1 = [0.3, 3.75, 0.3]
target_posizione_2 = [-3.83, 3.75, 0.3]
target_posizione_3 = [-3.83, -0.75, 0.3]
target_posizione_4 = [0.246, -0.75, 0.3]
target_posizione_5 = [0.25, -3.5656, 0.3]

lista_target_posizione.append(target_posizione_linea)
lista_target_posizione.append(target_posizione_1)
lista_target_posizione.append(target_posizione_2)
lista_target_posizione.append(target_posizione_3)
lista_target_posizione.append(target_posizione_4)
lista_target_posizione.append(target_posizione_5)

target_angolo_linea = 1.57
target_angolo_1 = 3.14
target_angolo_2 = -1.57
target_angolo_3 = 0.0
target_angolo_4 = -1.57
target_angolo_5 = 1.57

lista_target_angolo.append(target_angolo_linea)
lista_target_angolo.append(target_angolo_1)
lista_target_angolo.append(target_angolo_2)
lista_target_angolo.append(target_angolo_3)
lista_target_angolo.append(target_angolo_4)
lista_target_angolo.append(target_angolo_5)

indice_target = 0 

point_list = ["-3.89 0.3","-3.89 0.5"] # Y E X

tempo = 0
angle = 0.0
action = 0
no_of_steps = 0
current_step = 0


x_prima = 0.3
y_prima = -3.89
contatore_pausa = 0
arrivo = False

stop = 0 # MI prendo 0.032 ms *10 per ogni target raggiunto 

apertura_percorso1 = 0.0 # Da portae a -1.5 
apertura_percorso2 = 0.0 # Da portae a  1.5 

apertura_ufficio = 0.0  # Da potare a 1.5
apertura_entrata = 0.0  # Da portare a 0.0

porta_1 = []
porta_1_check = False
porta_2 = []
porta_2_check = False
#Creo un vettore per gestire le luci presenti in simulazione 

luci = []
illuminazione = 4.0 # Valore di defaul della illuminazione.
messaggio_illuminazione = Float64()
ultimo = False
avvio = False

class BeepDriver:
        
    def init(self, webots_node, properties): 
        
                
        rclpy.init(args=None)
                
        self.__robot = webots_node.robot
        self.__timestep = TIME_STEP

        self.__node = rclpy.create_node('oggetto_driver')
                        
        self.__node.get_logger().info('oggetto is supervisor? ' + str(self.__robot.getSupervisor()))
        
        self.contenuto = dir(webots_node.robot)
        self.__node.get_logger().info('Struttura oggetto: \n ' + str(self.contenuto))
        
        self.root_children_field = webots_node.robot.getRoot().getField('children')
        self.n =  self.root_children_field.getCount()
        
        # Client-Server per apertura e chiusura delle porte presenti in simulazione 
        
        self.__node.create_service(SetFloat, 'PortaPercorso1',self.servizio_moveporta1)
        self.__node.create_service(SetFloat, 'PortaPercorso2',self.servizio_moveporta2)

        self.__node.create_subscription(Float64,'tiago_illuminazione',self.__illuminazione_callback,1)

        # self.client_camera = self.__node.create_client(RequestData,'abilitazioneTelecamera')

        self.light_publisher = self.__node.create_publisher(Float64,'tiago_illuminazione',1)
        
        self.__node.create_subscription(Int64,'stato_linea',self.__statoLinea,1)

        
        for k in range(self.n):
            
            nodo_k = self.root_children_field.getMFNode(k)
            self.__node.get_logger().info(str(nodo_k.getTypeName())+ '  ' + str(k))
            
            if(nodo_k.getTypeName() == 'Robot'):    # Cerco il nodo Door in base al nome
                
                self.__node.get_logger().info('trovato Robot alla posizione  '+ str(k))
                uomo_name = nodo_k.getField('name')
                
                if(uomo_name.getSFString() == 'Beep'):
                    self.__node.get_logger().info('è stato trovato il robot umano...')
                    figlio_1 = nodo_k.getField('children')
                    n_figlio_1 = figlio_1.getCount()
                    self.__node.get_logger().info('numero nodi interni a children 1: ' + str(n_figlio_1))
                    self.__node.get_logger().info('dir children 1 : ' + str(dir(figlio_1)))
                    
                    rotation_oggetto = nodo_k.getField('rotation')
                    translation_oggetto = nodo_k.getField('translation')
                    
                    oggetto_xyz.append(rotation_oggetto)
                    oggetto_xyz.append(translation_oggetto)
                
                    self.__node.get_logger().info('trovato rotation intero human : ' + str(rotation_oggetto.getSFRotation()))
                    self.__node.get_logger().info('trovato translation intero human: ' + str(translation_oggetto .getSFVec3f()))
                                        
                                        
            elif(nodo_k.getTypeName() == 'Door'):  
                    
                    if(nodo_k.getField('name').getSFString() == 'doorPercorso1'):
                        
                        self.__node.get_logger().info('Sono dentro percorso 1')
                        
                        self.__node.get_logger().info('trovato Door alla posizione  '+ str(k))
                        door_position1 = nodo_k.getField('position')
                        door_position1.setSFFloat(apertura_percorso1)
                        self.__node.get_logger().info('settato la posizione della porta a 0.5')

                        porta_1.append(door_position1)
                        
                    elif(nodo_k.getField('name').getSFString() == 'doorPercorso2'):
                        
                        self.__node.get_logger().info('Sono dentro percorso 2')
                        
                        self.__node.get_logger().info('trovato Door alla posizione  '+ str(k))
                        door_position2 = nodo_k.getField('position')
                        door_position2.setSFFloat(apertura_percorso2)
                        self.__node.get_logger().info('settato la posizione della porta a 0.5')
                        porta_2.append(door_position2)

            elif(self.root_children_field.getMFNode(k).getTypeName() == 'PointLight'):  # Cerco il nodo PointLihgt in base al nome
                
                # self.__node.get_logger().info('trovato un PointLight alla posizione  '+ str(k))
                prova_luce = self.root_children_field.getMFNode(k)
                campo_luce = prova_luce.getField('intensity')
                campo_luce.setSFFloat(illuminazione) # Setto l'illuminazione iniziale a 4
                
                luci.append(prova_luce.getField('intensity')) # qui inserisco tutte le PointLight presenti in simulazione
                
    #           -------------------------- Fino a qua ho solo preso i giunti di interesse per muovere l'uomo in simulazione ------------------------------

        self.Start_up()   # Inizializzazione 
        
        
    def servizio_moveporta1(self,request , response): # Codice solo per riceve new target
            
            global apertura_percorso1,porta_1_check

            self.__node.get_logger().info('------------------------------------')
            self.__node.get_logger().info('Richiesta move porta 1 ricevuta')
            self.__node.get_logger().info(str(request.data))
            self.__node.get_logger().info('------------------------------------')
            
            apertura_percorso1 = 1.6
            response.success = True
            response.message = 'messaggio porta 1 ricevuto'
            porta_1_check = True
            
            return response

            
    def servizio_moveporta2(self,request , response): # Codice solo per riceve new target
            
            global apertura_percorso2,porta_2_check,apertura_percorso1

            self.__node.get_logger().info('------------------------------------')
            self.__node.get_logger().info('richiesta move porta 1 ricevuta ')
            self.__node.get_logger().info(str(request.data))
            self.__node.get_logger().info('------------------------------------')
            apertura_percorso1 = 0.0
            apertura_percorso2 = -1.6
            response.success = True
            response.message = 'messaggio porta 2 ricevuto'
            
            #Inserisco del codice per far chiudere anche la seconda porta 
            
            #servizio_moveporta1(self,request , response

            richiesta  = SetFloat.Request()
            richiesta.data = 0.0
            
            self.servizio_moveporta1(richiesta,response)
            porta_2_check = True
            
            #Invio il comando per l'attivazione della telecamera
            
            # self.send_request_camera()
            
            return response
                    
        
    def __illuminazione_callback(self, oggetto): 
                
        global illuminazione,luci
        illuminazione = oggetto.data
        
        for i in range(len(luci)):
            
            luci[i].setSFFloat(illuminazione)
            
    def __statoLinea(self,oggetto):
        
        global avvio,apertura_percorso2
        
        if(oggetto.data == 0):
            print('non faccio nulla')

        elif(oggetto.data == 1):
            avvio = True
            apertura_percorso2 = 0.0
        # def send_request_camera(self):
        
    #     request = RequestData()
        
    #     future = self.client_camera.call_async(request)
    #     rclpy.spin_until_future_complete(self, future)
            
            
    def Start_up(self):
        
        global point_list
        
        self.number_of_waypoints = len(point_list)
        self.waypoints = []
        
        for i in range(0,self.number_of_waypoints):
            self.waypoints.append([])
            self.waypoints[i].append(float(point_list[i].split()[0]))   #   y 
            self.waypoints[i].append(float(point_list[i].split()[1]))   #   x
        
        # Compute waypoints distance 
            
        self.waypoints_distance = []
        
        for i in range(0,self.number_of_waypoints):
            
            x = self.waypoints[i][1] - self.waypoints[(i + 1) % self.number_of_waypoints][1]    #   y
            y = self.waypoints[i][0] - self.waypoints[(i + 1) % self.number_of_waypoints][0]    #   x
            if i == 0:
                self.waypoints_distance.append(math.sqrt(y * y + x * x))
            else:
                self.waypoints_distance.append(self.waypoints_distance[i - 1] + math.sqrt(y * y + x * x))

            self.time = self.__robot.getTime()
            self.__node.get_logger().info('self.time = ' + str(self.time))

    def Convert(self):
        temp=point_list[-1]
        X,Y=temp.split(' ')
        X=float(X)
        Y=float(Y)
        return X,Y
    
    def funzione_target(self):
        
        global indice_target,angle, point_list, change
        
        if indice_target == 0:
            
            X,Y=self.Convert()
            point_list[-2]=point_list[-1]
            change=str(X + 0.05)+' '+ str(Y)
            point_list[-1]=change
            self.Start_up()
            angle= 1.57
            
        
        if indice_target == 1:
            
            X,Y=self.Convert()
            point_list[-2]=point_list[-1]
            change=str(X + 0.05)+' '+ str(Y)
            point_list[-1]=change
            self.Start_up()
            angle=3.14

        elif indice_target == 2:
            
            X,Y=self.Convert()
            point_list[-2]=point_list[-1]
            change=str(X)+' '+ str(Y-0.05)
            point_list[-1]=change
            self.Start_up()
            angle= -1.57
                
        elif indice_target == 3:
            
            X,Y=self.Convert()
            point_list[-2]=point_list[-1]
            change=str(X - 0.05)+' '+ str(Y)
            point_list[-1]=change
            self.Start_up()
            angle=0.0
                
        elif indice_target == 4:
            
            X,Y=self.Convert()
            point_list[-2]=point_list[-1]
            change=str(X)+' '+ str(Y+ 0.05)
            point_list[-1]=change
            self.Start_up()
            angle= -1.57
            
        elif indice_target == 5:
            
            X,Y=self.Convert()
            point_list[-2]=point_list[-1]
            change=str(X - 0.05)+' '+ str(Y)
            point_list[-1]=change
            self.Start_up()
            angle = 1.57
    
    
    def step(self):       
        
        global  speed, current_height_offset, oggetto_xyz, point_list, tempo, angle
        global  action, no_of_steps, current_step,x_prima,y_prima,arrivo
        global lista_target_posizione, indice_target, contatore_pausa
        
        global apertura_percorso1,apertura_percorso2,porta_1,porta_2, illuminazione, messaggio_illuminazione,ultimo
        
        rclpy.spin_once(self.__node, timeout_sec=0)
        
        tempo = self.__robot.getTime()
        
        
        porta_1[0].setSFFloat(apertura_percorso1)
        porta_2[0].setSFFloat(apertura_percorso2)
        
        if(porta_1_check == True):
            

            messaggio_illuminazione.data = illuminazione
            self.light_publisher.publish(messaggio_illuminazione)
            
            if(illuminazione <= 0.1):
                
                illuminazione = illuminazione + 0.1
                ultimo = False
            
            elif(illuminazione >= 10):
                
                illuminazione = illuminazione - 0.1
                ultimo = True
            elif(illuminazione>= 0.0 and illuminazione <=10 ):
                
                if(ultimo == False):
                    illuminazione = illuminazione + 0.1
                else:
                    illuminazione = illuminazione - 0.1


               
        
        if(avvio == True):
        
            if(contatore_pausa == 0):
                
                self.__node.get_logger().info('tempo di simulazione' + str(tempo))
                self.__node.get_logger().info('posizione  ' + str(oggetto_xyz[1].getSFVec3f()))
                self.__node.get_logger().info('rotazione  ' + str(oggetto_xyz[0].getSFRotation()))

                
                self.funzione_target()
                
                distance = tempo * speed
                relative_distance = distance - int(distance / self.waypoints_distance[self.number_of_waypoints - 1]) * self.waypoints_distance[self.number_of_waypoints - 1]


            
                for i in range(self.number_of_waypoints):
                    if self.waypoints_distance[i] > relative_distance:
                        break
                    
                distance_ratio = 0
                
                if (i == 0):
                    distance_ratio = relative_distance/self.waypoints_distance[i]
                else:
                    
                    distance_ratio = (relative_distance - self.waypoints_distance[i - 1])/(self.waypoints_distance[i] - self.waypoints_distance[i - 1])
                    
                # distance_ratio = relative_distance/self.waypoints_distance[i]
                
                x = distance_ratio * self.waypoints[(i + 1) % self.number_of_waypoints][1] + \
                        (1 - distance_ratio) * self.waypoints[i][1]
                y = distance_ratio * self.waypoints[(i + 1) % self.number_of_waypoints][0] + \
                        (1 - distance_ratio) * self.waypoints[i][0]
                        
                root_translation = [x,y, ROOT_HEIGHT]
                    
                x_prima = x
                y_prima = y
                
                
                if(indice_target == 0): # Sono sulla linea 

                    self.__node.get_logger().info('Siamo al primo target')
                    
                    
                    if(abs(y- lista_target_posizione[indice_target][1]) <= 0.05 ):
                                                            
                            indice_target = indice_target +1    #Passo al prossimo target
                            contatore_pausa = 800
                            # contatore_pausa = 10


                            
                    else:
                        
                        oggetto_xyz[1].setSFVec3f(root_translation)
                        
            
                if(indice_target == 1): 
                    
                    
                    self.__node.get_logger().info('Siamo al primo target')
                    
                    
                    if(abs(y- lista_target_posizione[indice_target][1]) <= 0.05 ):
                                                            
                            rotation = [0, 0, 1, 3.14]
                            oggetto_xyz[0].setSFRotation(rotation)                                                                        
                
                            indice_target = indice_target +1    #Passo al prossimo target
                            contatore_pausa = 10

                            
                    else:
                        
                        oggetto_xyz[1].setSFVec3f(root_translation)
                        
            
                elif(indice_target == 2):
                    
                    self.__node.get_logger().info('Siamo al secondo target')
                    
                    if(abs(x - lista_target_posizione[indice_target][0]) <= 0.05 ):
                                                            
                            rotation = [0, 0, 1, -1.57]
                            oggetto_xyz[0].setSFRotation(rotation)
                                                                                


                            indice_target = indice_target +1    #Passo al prossimo target
                            contatore_pausa = 10

                            
                    else:
                        
                        oggetto_xyz[1].setSFVec3f(root_translation)
                        
                        
                
                elif(indice_target == 3):
                    
                    self.__node.get_logger().info('Siamo al terzo target')
                    
                    if(abs(y- lista_target_posizione[indice_target][1]) <= 0.05 ):
                                                            
                            rotation = [0, 0, 1, 0.0]
                            oggetto_xyz[0].setSFRotation(rotation)
                                                                                

                            
                            indice_target = indice_target +1    #Passo al prossimo target
                            contatore_pausa = 10

                            
                    else:
                        
                        oggetto_xyz[1].setSFVec3f(root_translation)

                elif(indice_target == 4):
                    
                    self.__node.get_logger().info('Siamo al quarto target')
                    
                    if(abs(x - lista_target_posizione[indice_target][0]) <= 0.05 ):
                                                            
                            rotation = [0, 0, 1, -1.57]
                            oggetto_xyz[0].setSFRotation(rotation)
                                                                
                            indice_target = indice_target +1    #Passo al prossimo target
                            contatore_pausa = 10

                            
                    else:
                        
                        oggetto_xyz[1].setSFVec3f(root_translation)

                elif(indice_target == 5):
                    
                    self.__node.get_logger().info('Siamo al quinto target')
                                
                    if(abs(y - lista_target_posizione[indice_target][1]) <= 0.05 ):
                                                            
                            rotation = [0, 0, 1, 1.57]
                            oggetto_xyz[0].setSFRotation(rotation)
                            spostamento = [0.72, -3.5656, 0.3]
                            oggetto_xyz[1].setSFVec3f(spostamento)
                            
                                                                    
                                            
                            indice_target = indice_target +1    #Passo al prossimo target
                            contatore_pausa = 10
                            
                    else:
                        
                        oggetto_xyz[1].setSFVec3f(root_translation)
                        
                elif(indice_target == 6):
                    
                    self.__node.get_logger().info('Percorso umani finito')
                    
                    
                    # rotation = [0, 0, 1, 1.57]
                    # oggetto_xyz[0].setSFRotation(rotation)
                    
                    
            else:
                
                self.__node.get_logger().info('In attesa per il nuovo target')
                contatore_pausa = contatore_pausa -1


                        