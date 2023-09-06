import rclpy
import time
from random import randint

import optparse
import math
import time

TIME_STEP = 32 # milliseconds

BODY_PARTS_NUMBER = 13
WALK_SEQUENCES_NUMBER = 8
ROOT_HEIGHT = 1.27
CYCLE_TO_DISTANCE_RATIO = 0.032

speed = 1.15
current_height_offset = 0

# Posizione dei vari giunti
joints_position_field = [] 

# lista contente i nomi degli angoli che prendo in considerazione
joint_names = [
            "leftArmAngle", "leftLowerArmAngle", "leftHandAngle",
            "rightArmAngle", "rightLowerArmAngle", "rightHandAngle",
            "leftLegAngle", "leftLowerLegAngle", "leftFootAngle",
            "rightLegAngle", "rightLowerLegAngle", "rightFootAngle",
            "headAngle"
        ]

# questi coefficienti sono coefficienti empirici che risultano in un'andatura realistica
height_offsets = [
            -0.02, 0.04, 0.08, -0.03, -0.02, 0.04, 0.08, -0.03
        ]
# questi coefficienti sono coefficienti empirici che si traducono in un'andatura a piedi realistica

angles = [
            [-0.52, -0.15, 0.58, 0.7, 0.52, 0.17, -0.36, -0.74],    # left arm
            [0.0, -0.16, -0.7, -0.38, -0.47, -0.3, -0.58, -0.21],   # left lower arm
            [0.12, 0.0, 0.12, 0.2, 0.0, -0.17, -0.25, 0.0],         # left hand
            [0.52, 0.17, -0.36, -0.74, -0.52, -0.15, 0.58, 0.7],    # right arm
            [-0.47, -0.3, -0.58, -0.21, 0.0, -0.16, -0.7, -0.38],   # right lower arm
            [0.0, -0.17, -0.25, 0.0, 0.12, 0.0, 0.12, 0.2],         # right hand
            [-0.55, -0.85, -1.14, -0.7, -0.56, 0.12, 0.24, 0.4],    # left leg
            [1.4, 1.58, 1.71, 0.49, 0.84, 0.0, 0.14, 0.26],         # left lower leg
            [0.07, 0.07, -0.07, -0.36, 0.0, 0.0, 0.32, -0.07],      # left foot
            [-0.56, 0.12, 0.24, 0.4, -0.55, -0.85, -1.14, -0.7],    # right leg
            [0.84, 0.0, 0.14, 0.26, 1.4, 1.58, 1.71, 0.49],         # right lower leg
            [0.0, 0.0, 0.42, -0.07, 0.07, 0.07, -0.07, -0.36],      # right foot
            [0.18, 0.09, 0.0, 0.09, 0.18, 0.09, 0.0, 0.09]          # head
        ]

joint_rotation_objects = []

joint_translation_objects = []

joint_name_objects = []

human_xyz = []

lista_obbiettivi = []


lista_obbiettivi.append(-0.76)
lista_obbiettivi.append(-3.58)


# Definisco i target che l'uomo deve raggiungere quando inizio la simulazione...

lista_target_posizione = []
lista_target_angolo = []

target_posizione_linea = [0.436, -2.85, 1.27]
target_posizione_1 = [0.436, 3.95, 1.27]
target_posizione_2 = [-4.16, 3.95, 1.27]
target_posizione_3 = [-4.16, -0.57, 1.27]
target_posizione_4 = [0.560, -0.57, 1.27]
target_posizione_5 = [0.560, -3.721, 1.27]

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


# Da verificare il vero significato di point_list 


# Posizioni iniziali 

# point_list = ["-2.11 0.43 " , "-2.11 0.45" ]
# point_list = ["0.43 -2.11 " , "0.43 1.27" ] # X E Y in ordine

# point_list = ["0.43 1.27" , "-2.11 1.27" ] # X E Y in ordine
point_list = ["-3.571 0.43","-3.571 0.43"] # Y E X

# point_list = ["-2.11 0.43" , "-2.11 0.46" ]
tempo = 0
# angle = 1.57
angle = 0.0
action = 0
no_of_steps = 0
current_step = 0


x_prima = 0.43
y_prima = -3.581
contatore_pausa = 0
arrivo = False

stop = 0 # MI prendo 0.032 ms *10 per ogni target raggiunto 

class Pedestrian:
        
    def init(self, webots_node, properties): 
        
                
        rclpy.init(args=None)
                
        self.__robot = webots_node.robot
        self.__timestep = TIME_STEP

        self.__node = rclpy.create_node('pedestrian_driver')
                        
        self.__node.get_logger().info('  Pedestrian is supervisor? ' + str(self.__robot.getSupervisor()))
        
        self.contenuto = dir(webots_node.robot)
        self.__node.get_logger().info('Struttura Pedestrian:  \n ' + str(self.contenuto))
        
        self.root_children_field = webots_node.robot.getRoot().getField('children')
        self.n =  self.root_children_field.getCount()
        
        for k in range(self.n):
            
            nodo_k = self.root_children_field.getMFNode(k)
            self.__node.get_logger().info(str(nodo_k.getTypeName())+ '  ' + str(k))
            
            if(nodo_k.getTypeName() == 'Robot'):    # Cerco il nodo Door in base al nome
                
                self.__node.get_logger().info('trovato Robot alla posizione  '+ str(k))
                uomo_name = nodo_k.getField('name')
                if(uomo_name.getSFString() == 'pedestrian'):
                    self.__node.get_logger().info('è stato trovato il robot umano...')
                    figlio_1 = nodo_k.getField('children')
                    n_figlio_1 = figlio_1.getCount()
                    self.__node.get_logger().info('numero nodi interni a children 1: ' + str(n_figlio_1))
                    self.__node.get_logger().info('dir children 1 : ' + str(dir(figlio_1)))
                    
                    rotation_human = nodo_k.getField('rotation')
                    translation_human = nodo_k.getField('translation')
                    
                    human_xyz.append(rotation_human)
                    human_xyz.append(translation_human)
                

                    self.__node.get_logger().info('trovato rotation intero human : ' + str(rotation_human.getSFRotation()))
                    self.__node.get_logger().info('trovato translation intero human: ' + str(translation_human .getSFVec3f()))
                                        
                
                    #getMFNode da utilizzare forse per entrare dentro Solid
                    
                    for l in range(n_figlio_1):

                        nodo_l = figlio_1.getMFNode(l)
                        self.__node.get_logger().info( 'nodo_l : ' + str(nodo_l.getTypeName()))
                        
                        if(nodo_l.getTypeName() == 'Solid'):    # Cerco il nodo Solid in base al nome
                
                            figlio_2 = nodo_l.getField('children')
                            n_figlio_2 = figlio_2.getCount()
                            self.__node.get_logger().info('numero nodi interni a children 2: ' + str(n_figlio_2))
                            self.__node.get_logger().info('dir children 2: ' + str(dir(figlio_2)))


                            for z in range(n_figlio_2):
                                
                                nodo_z = figlio_2.getMFNode(z)
                                self.__node.get_logger().info( 'nodo_z : ' + str(nodo_z.getTypeName()))
                                
                                if(nodo_z.getTypeName() == 'Solid'):    # ogni Solid in questo livello coincide un giunto che posso pilotare
                                    
                                    if(nodo_z.getField('name').getSFString() == 'left arm'):
                                        # Qui sono dentro LEFT_UPPER_ARM
                                        
                                        rotation_left_upper_arm = nodo_z.getField('rotation')
                                        translation_left_upper_arm = nodo_z.getField('translation')

                                        self.__node.get_logger().info('trovato rotation in left_upper_arm : ' + str(rotation_left_upper_arm.getSFRotation()))
                                        self.__node.get_logger().info('trovato translation in left_upper_arm : ' + str(translation_left_upper_arm.getSFVec3f()))
                                        
                                        figlio_3_left_arm =  nodo_z.getField('children')
                                        n_figlio_3_left_arm = figlio_3_left_arm.getCount()
                                        
                                        joint_rotation_objects.append(rotation_left_upper_arm)   # elemento 1
                                        joint_translation_objects.append(translation_left_upper_arm)
                                        
                                        joint_name_objects.append(nodo_z.getField('name'))

                                        
                                        for x in range(n_figlio_3_left_arm):
                                            
                                            nodo_x = figlio_3_left_arm.getMFNode(x)
                                            if(nodo_x.getTypeName() == 'Solid'):    # Cerco il nodo Solid in base al nome
                                            
                                                # Qui sono dentro LEFT_LOWER_ARM
                                                rotation_left_arm = nodo_x.getField('rotation')
                                                translation_left_arm = nodo_x.getField('translation')
                                                
                                                self.__node.get_logger().info('trovato rotation in left arm : ' + str(rotation_left_arm.getSFRotation()))
                                                self.__node.get_logger().info('trovato translation in left arm : ' + str(translation_left_arm.getSFVec3f()))
                                                
                                                figlio_4_left_hand =  nodo_x.getField('children')
                                                n_figlio_4_left_hand = figlio_4_left_hand.getCount()
                                                
                                                joint_rotation_objects.append(rotation_left_arm)    # elemento 2
                                                joint_translation_objects.append(translation_left_arm)
                                                
                                                joint_name_objects.append(nodo_x.getField('name'))


                                                
                                                for c in range(n_figlio_4_left_hand):
                                                        nodo_c = figlio_4_left_hand.getMFNode(c)
                                                        if(nodo_c.getTypeName() == 'Solid'):
                                                            
                                                            rotation_left_hand = nodo_c.getField('rotation')
                                                            translation_left_hand = nodo_c.getField('translation')
                                                            
                                                            self.__node.get_logger().info('trovato rotation in left hand : ' + str(rotation_left_hand.getSFRotation()))
                                                            self.__node.get_logger().info('trovato translation in left hand : ' + str(translation_left_hand.getSFVec3f()))

                                                            
                                                            joint_rotation_objects.append(rotation_left_hand)   # elemento 3
                                                            joint_translation_objects.append(translation_left_hand)
                                                
                                                            joint_name_objects.append(nodo_c.getField('name'))

                                        
                                    elif(nodo_z.getField('name').getSFString() == 'right arm'):
                                        # Qui sono dentro RIGHT_UPPER_ARM
                                        
                                        rotation_right_upper_arm = nodo_z.getField('rotation')
                                        translation_right_upper_arm = nodo_z.getField('translation')
                                        
                                        self.__node.get_logger().info('trovato rotation in right_upper_arm : ' + str(rotation_right_upper_arm.getSFRotation()))
                                        self.__node.get_logger().info('trovato translation in right_upper_arm : ' + str(translation_right_upper_arm.getSFVec3f()))
                                        
                                        figlio_3_right_arm =  nodo_z.getField('children')
                                        n_figlio_3_right_arm = figlio_3_right_arm.getCount()
                                        
                                        joint_rotation_objects.append(rotation_right_upper_arm)  # elemento 4
                                        joint_translation_objects.append(translation_right_upper_arm)

                                        joint_name_objects.append(nodo_z.getField('name'))
                                        
                                        for x in range(n_figlio_3_right_arm):
                                            
                                            nodo_x = figlio_3_right_arm .getMFNode(x)
                                            if(nodo_x.getTypeName() == 'Solid'):    # Cerco il nodo Solid in base al nome
                                                                
                                                # Qui sono dentro RIGHT_LOWER_ARM
                                                rotation_right_arm = nodo_x.getField('rotation')
                                                translation_right_arm = nodo_x.getField('translation')
                                                
                                                self.__node.get_logger().info('trovato rotation in right arm : ' + str(rotation_right_arm.getSFRotation()))
                                                self.__node.get_logger().info('trovato translation in right arm : ' + str(translation_right_arm.getSFVec3f()))
                                                
                                                figlio_4_right_hand =  nodo_x.getField('children')
                                                n_figlio_4_right_hand = figlio_4_right_hand.getCount()
                                                
                                                joint_rotation_objects.append(rotation_right_arm)   # elemento 5
                                                joint_translation_objects.append(translation_right_arm)
                                                
                                                joint_name_objects.append(nodo_x.getField('name'))


                                                
                                                for c in range(n_figlio_4_right_hand):
                                                        nodo_c = figlio_4_right_hand.getMFNode(c)
                                                        if(nodo_c.getTypeName() == 'Solid'):
                                                            
                                                            rotation_right_hand = nodo_c.getField('rotation')
                                                            translation_right_hand = nodo_c.getField('translation')
                                                            
                                                            self.__node.get_logger().info('trovato rotation in right hand : ' + str(rotation_right_hand.getSFRotation()))
                                                            self.__node.get_logger().info('trovato translation in right hand : ' + str(translation_right_hand.getSFVec3f()))
                                                            
                                                            joint_rotation_objects.append(rotation_right_hand)  # elemento 6
                                                            joint_translation_objects.append(translation_right_hand)
                                                            
                                                            joint_name_objects.append(nodo_c.getField('name'))

                                        
                                    elif(nodo_z.getField('name').getSFString() == 'left leg'):
                                        # Qui sono dentro LEFT_UPPER_LEG
                                        
                                        rotation_left_upper_leg = nodo_z.getField('rotation')
                                        translation_left_upper_leg = nodo_z.getField('translation')
                                        
                                        self.__node.get_logger().info('trovato rotation in left_upper_leg : ' + str(rotation_left_upper_leg.getSFRotation()))
                                        self.__node.get_logger().info('trovato translation in left_upper_leg : ' + str(translation_left_upper_leg.getSFVec3f()))


                                        figlio_3_left_leg =  nodo_z.getField('children')
                                        n_figlio_3_left_leg= figlio_3_left_leg.getCount()
                                        
                                        joint_rotation_objects.append(rotation_left_upper_leg)   # elemento 7
                                        joint_translation_objects.append(translation_left_upper_leg) 
                                        
                                        joint_name_objects.append(nodo_z.getField('name'))
                                        
                                        for x in range(n_figlio_3_left_leg):
                                            
                                            nodo_x = figlio_3_left_leg.getMFNode(x)
                                            if(nodo_x.getTypeName() == 'Solid'):    # Cerco il nodo Solid in base al nome
                                                # Qui sono dentro LEFT_LOWER_LEG
                                                rotation_left_leg = nodo_x.getField('rotation')
                                                translation_left_leg = nodo_x.getField('translation')
                                                
                                                self.__node.get_logger().info('trovato rotation in left leg: ' + str(rotation_left_leg.getSFRotation()))
                                                self.__node.get_logger().info('trovato translation in left leg: ' + str(translation_left_leg.getSFVec3f()))

                                                figlio_4_left_foot =  nodo_x.getField('children')
                                                n_figlio_4_left_foot = figlio_4_left_foot.getCount()
                                                
                                                joint_rotation_objects.append(rotation_left_leg)    # elemento 8  
                                                joint_translation_objects.append(translation_left_leg )
                                                
                                                joint_name_objects.append(nodo_x.getField('name'))

                                                
                                                for c in range(n_figlio_4_left_foot):
                                                        nodo_c = figlio_4_left_foot.getMFNode(c)
                                                        if(nodo_c.getTypeName() == 'Solid'):
                                                            
                                                            rotation_left_foot = nodo_c.getField('rotation')
                                                            translation_left_foot = nodo_c.getField('translation')
                                                            
                                                            self.__node.get_logger().info('trovato rotation in left foot : ' + str(rotation_left_foot.getSFRotation()))
                                                            self.__node.get_logger().info('trovato translation in left foot : ' + str(translation_left_foot.getSFVec3f()))

                                                            joint_rotation_objects.append(rotation_left_foot)   # elemento 9
                                                            joint_translation_objects.append(translation_left_foot) 
                                                            
                                                            joint_name_objects.append(nodo_c.getField('name'))



                                        
                                    elif(nodo_z.getField('name').getSFString() == 'right leg'):
                                        # Qui sono dentro RIGHT_UPPER_LEG
                                        rotation_right_upper_leg = nodo_z.getField('rotation')
                                        translation_right_upper_leg = nodo_z.getField('translation')
                                        
                                        self.__node.get_logger().info('trovato rotation in right_upper_leg : ' + str(rotation_right_upper_leg.getSFRotation()))
                                        self.__node.get_logger().info('trovato translation in right_upper_leg : ' + str(translation_right_upper_leg.getSFVec3f()))


                                        figlio_3_right_leg =  nodo_z.getField('children')
                                        n_figlio_3_right_leg= figlio_3_right_leg.getCount()
                                        
                                        joint_rotation_objects.append(rotation_right_upper_leg)  # elemento 10
                                        joint_translation_objects.append(translation_right_upper_leg)
                                        
                                        joint_name_objects.append(nodo_z.getField('name'))


                                        
                                        for x in range(n_figlio_3_right_leg):
                                            
                                            nodo_x = figlio_3_right_leg.getMFNode(x)
                                            if(nodo_x.getTypeName() == 'Solid'):    # Cerco il nodo Solid in base al nome
                                                # Qui sono dentro RIGHT_LOWER_LEG
                                                rotation_right_leg = nodo_x.getField('rotation')
                                                translation_right_leg = nodo_x.getField('translation')
                                                
                                                self.__node.get_logger().info('trovato rotation in right leg: ' + str(rotation_right_leg.getSFRotation()))
                                                self.__node.get_logger().info('trovato translation in right leg: ' + str(translation_right_leg.getSFVec3f()))

                                                figlio_4_right_foot =  nodo_x.getField('children')
                                                n_figlio_4_right_foot = figlio_4_right_foot.getCount()
                                                
                                                joint_rotation_objects.append(rotation_right_leg)   # elemento 11
                                                joint_translation_objects.append(translation_right_leg)
                                                
                                                joint_name_objects.append(nodo_x.getField('name'))

                                                
                                                for c in range(n_figlio_4_right_foot):
                                                        nodo_c = figlio_4_right_hand.getMFNode(c)
                                                        if(nodo_c.getTypeName() == 'Solid'):
                                                            
                                                            rotation_right_foot = nodo_c.getField('rotation')
                                                            translation_right_foot = nodo_c.getField('translation')
                                                            
                                                            self.__node.get_logger().info('trovato rotation in right foot: ' + str(rotation_right_foot.getSFRotation()))
                                                            self.__node.get_logger().info('trovato translation in right foot: ' + str(translation_right_foot.getSFVec3f()))
                                                            
                                                            joint_rotation_objects.append(rotation_right_foot)  # elemento 12
                                                            joint_translation_objects.append(translation_right_foot)
                                                            
                                                            joint_name_objects.append(nodo_c.getField('name'))

                                        
                                    elif(nodo_z.getField('name').getSFString() == 'head'):
                                        
                                        rotation_head =  nodo_z.getField('rotation')
                                        translation_head =  nodo_z.getField('translation')
                                        
                                        self.__node.get_logger().info('trovato rotation in head: ' + str(rotation_head.getSFRotation()))
                                        self.__node.get_logger().info('trovato translation in head: ' + str(translation_head.getSFVec3f()))
                                        
                                        joint_rotation_objects.append(rotation_head)    # elemento 13
                                        joint_translation_objects.append(translation_head)
                                        
                                        joint_name_objects.append(nodo_k.getField('name'))
                                        
    #           -------------------------- Fino a qua ho solo preso i giunti di interesse per muovere l'uomo in simulazione ------------------------------

    # Linea Init di inizializzazione ancora 
    # Provo a stampare i valori in ordine 
            
        self.__node.get_logger().info('Inizio print del vettore joint_rotation_objects ')
            
        for indice in range(0,BODY_PARTS_NUMBER):
                
            self.__node.get_logger().info( '[' + str(indice) +'] ' + str(joint_rotation_objects[indice]))
            self.__node.get_logger().info( '[' + str(indice) +'] ' + str(joint_name_objects[indice].getSFString()))

            
        self.Start_up()   # Inizializzazione 
        
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
        
        global  speed, current_height_offset, joints_position_field, joint_names
        global  height_offsets, angles, joint_rotation_objects, joint_translation_objects
        global  joint_name_objects, human_xyz, point_list, tempo, angle
        global  action, no_of_steps, current_step,x_prima,y_prima,arrivo
        global lista_target_posizione, indice_target, contatore_pausa
        
        rclpy.spin_once(self.__node, timeout_sec=0)
        
        tempo = self.__robot.getTime()
        
        if(contatore_pausa == 0):
            
            self.__node.get_logger().info('tempo di simulazione' + str(tempo))
            self.__node.get_logger().info('posizione  ' + str(human_xyz[1].getSFVec3f()))
            self.__node.get_logger().info('rotazione  ' + str(human_xyz[0].getSFRotation()))

            # Comandi presi per far avanzare il robot lungo L'asse y della simulazione...
            # Nella repository equivale alla funzione keyboardvalue()
            
            self.funzione_target()

            # INIZIO CALCOLO DELLE FUNZIONI VELOCITA
            
            current_sequence = int(((tempo * speed) / CYCLE_TO_DISTANCE_RATIO) % WALK_SEQUENCES_NUMBER)
    
            ratio = (tempo * speed) / CYCLE_TO_DISTANCE_RATIO - int(((tempo * speed) / CYCLE_TO_DISTANCE_RATIO))
                    
            for i in range(0,BODY_PARTS_NUMBER):
                
                    current_angle = angles[i][current_sequence] * (1 - ratio) + \
                    angles[i][(current_sequence + 1) % WALK_SEQUENCES_NUMBER] * ratio
                    #   Nuova modifica    joints_position_field[i].setSFFloat(current_angle)
                    rotazione_i = [0, 1, 0, current_angle]

                    joint_rotation_objects[i].setSFRotation(rotazione_i)
                    # joint_rotation_objects[i].setSFFloat(current_angle)
                    
            # adjust height
            # Devo utilizzare current_height_offset...
            
            current_height_offset = height_offsets[current_sequence] * (1 - ratio) + \
            height_offsets[(current_sequence + 1) % WALK_SEQUENCES_NUMBER] * ratio
            
            # move everything
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
                    
            root_translation = [x,y, ROOT_HEIGHT + current_height_offset]
                
            x_prima = x
            y_prima = y
            
            
            if(indice_target == 0): # Sono sulla linea 
                
                # [0] = coordinata  x --> Qui la x è la y
                # [1] = coordinata  y --> Qui la y è la x
                # [2] = coordinata  z --> In simulazione rimane costante

                
                self.__node.get_logger().info('Siamo al primo target')
                
                
                if(abs(y- lista_target_posizione[indice_target][1]) <= 0.2 ):
                                                        
                        rotation = [0, 0, 1, 1.27]
                        human_xyz[0].setSFRotation(rotation)                                                                        
                        rotazione_arti = [0, 1, 0, 0.0]

                        for i in range(BODY_PARTS_NUMBER):
                    
                            joint_rotation_objects[i].setSFRotation(rotazione_arti)
                                            
                        indice_target = indice_target +1    #Passo al prossimo target
                        contatore_pausa = 600

                        
                else:
                    
                    human_xyz[1].setSFVec3f(root_translation)
                      
        
            if(indice_target == 1): 
                
                
                self.__node.get_logger().info('Siamo al primo target')
                
                
                if(abs(y- lista_target_posizione[indice_target][1]) <= 0.2 ):
                                                        
                        rotation = [0, 0, 1, 3.14]
                        human_xyz[0].setSFRotation(rotation)                                                                        
                        rotazione_arti = [0, 1, 0, 0.0]

                        for i in range(BODY_PARTS_NUMBER):
                    
                            joint_rotation_objects[i].setSFRotation(rotazione_arti)
                                            
                        indice_target = indice_target +1    #Passo al prossimo target
                        contatore_pausa = 10

                        
                else:
                    
                    human_xyz[1].setSFVec3f(root_translation)
                    
        
            elif(indice_target == 2):
                
                self.__node.get_logger().info('Siamo al secondo target')
                
                if(abs(x - lista_target_posizione[indice_target][0]) <= 0.2 ):
                                                        
                        rotation = [0, 0, 1, -1.57]
                        human_xyz[0].setSFRotation(rotation)
                                                                            
                        rotazione_arti = [0, 1, 0, 0.0]

                        for i in range(BODY_PARTS_NUMBER):
                    
                            joint_rotation_objects[i].setSFRotation(rotazione_arti)
                    
                        
                        indice_target = indice_target +1    #Passo al prossimo target
                        contatore_pausa = 10

                        
                else:
                    
                    human_xyz[1].setSFVec3f(root_translation)
                    
                    
            
            elif(indice_target == 3):
                
                self.__node.get_logger().info('Siamo al terzo target')
                
                if(abs(y- lista_target_posizione[indice_target][1]) <= 0.2 ):
                                                        
                        rotation = [0, 0, 1, 0.0]
                        human_xyz[0].setSFRotation(rotation)
                                                                            
                        rotazione_arti = [0, 1, 0, 0.0]

                        for i in range(BODY_PARTS_NUMBER):
                    
                            joint_rotation_objects[i].setSFRotation(rotazione_arti)
                    
                        
                        indice_target = indice_target +1    #Passo al prossimo target
                        contatore_pausa = 10

                        
                else:
                    
                    human_xyz[1].setSFVec3f(root_translation)

            elif(indice_target == 4):
                
                self.__node.get_logger().info('Siamo al quarto target')
                
                if(abs(x - lista_target_posizione[indice_target][0]) <= 0.2 ):
                                                        
                        rotation = [0, 0, 1, -1.57]
                        human_xyz[0].setSFRotation(rotation)
                                                                            
                        rotazione_arti = [0, 1, 0, 0.0]

                        for i in range(BODY_PARTS_NUMBER):
                    
                            joint_rotation_objects[i].setSFRotation(rotazione_arti)
                                        
                        indice_target = indice_target +1    #Passo al prossimo target
                        contatore_pausa = 10

                        
                else:
                    
                    human_xyz[1].setSFVec3f(root_translation)

            elif(indice_target == 5):
                
                self.__node.get_logger().info('Siamo al quinto target')
                            
                if(abs(y - lista_target_posizione[indice_target][1]) <= 0.2 ):
                                                        
                        rotation = [0, 0, 1, 0.0]
                        human_xyz[0].setSFRotation(rotation)
                                                                
                        rotazione_arti = [0, 1, 0, 0.0]

                        for i in range(BODY_PARTS_NUMBER):
                    
                            joint_rotation_objects[indice_target].setSFRotation(rotazione_arti)
                                        
                        indice_target = indice_target +1    #Passo al prossimo target
                        contatore_pausa = 10
                        
                else:
                    
                    human_xyz[1].setSFVec3f(root_translation)
                    
            elif(indice_target == 6):
                
                self.__node.get_logger().info('Percorso umani finito')
                
                
                rotation = [0, 0, 1, 1.57]
                human_xyz[0].setSFRotation(rotation)
                
                rotazione_arti = [0, 1, 0, 0.0]

                for i in range(BODY_PARTS_NUMBER):
                    
                    joint_rotation_objects[i].setSFRotation(rotazione_arti)

        else:
            
            self.__node.get_logger().info('In attesa per il nuovo target')
            contatore_pausa = contatore_pausa -1


                        