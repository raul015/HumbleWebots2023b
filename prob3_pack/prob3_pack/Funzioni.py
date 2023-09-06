
assoluto = []
assoluto.append(0.0)
assoluto.append(0.0)
assoluto.append(0.0)
assoluto.append(0.0)
assoluto.append(0.0)
assoluto.append(0.0)
assoluto.append(0.0)
assoluto.append(0.0)



class Token():
    
    def __init__(self):
        
        self.messagio = ''
        self.check_posizione = True


def reset_velocity(actuator_ids,motor_device,velocities):
    
    for i in range(len(actuator_ids)):
        velocities[actuator_ids[i]-1] = 0.0
        motor_device[actuator_ids[i]-1].setVelocity(velocities[actuator_ids[i]-1])   

        print('dentro for ')

def waiting_arrive(arrivo):
    
    for i in range(len(arrivo)):
        
        if(arrivo[i] == False):

            return False
        
    return True

# ingressi utente : actuator_ids, position, reset_velocity --> indice i


                            # ID,       INPUT_UTENTE,        LIMITE_SUP,    LIMITE_INF,             LIMITE_VELOCITA,    POS_PARTENZA_
def controllo_dati_ingresso(actuator_ids,position,velocity,limit_position_sup,limit_position_inf,limit_velocity,relative,rel_initial_positions,arrivo):
    
    token = Token()
    
    token.messaggio = ''
    token.check_posizione = True

    
    if(relative == False):
        for i in range(len(actuator_ids)):
                        
            if(len(actuator_ids)>0 and len(position)>0 and len(actuator_ids)<9 and len(position)<9 and len(actuator_ids) == len(position) ):

                print('posizione coerente con le dimensione degli input')
                
                if(actuator_ids[i]>=1 and actuator_ids[i]<=8):
                
                    
                    if(position[i] >= (limit_position_inf[actuator_ids[i]-1]) and position[i] <= (limit_position_sup[actuator_ids[i]-1]) ):
                        print('posizione coerente con i limiti di posizione ')
                    
                        if(velocity > 0.0 and velocity < limit_velocity[actuator_ids[i]-1]) :

                                prova = 0                            
                        else:
                            
                            if(velocity == 0.0):
                                
                                token.messaggio = token.messaggio + 'velocita ' + str(velocity) + 'è a 0.0, quindi non si muoverà \n'
                                token.check_posizione = False

                                return token

                            elif(velocity > limit_velocity[actuator_ids[i]-1]):
                                
                                token.messaggio = token.messaggio + 'velocita ' + str(velocity) + 'supera il limite ' + limit_velocity[actuator_ids[i]-1] +'\n'
                                token.check_posizione = False

                                return token
                    else:
                        
                                        
                        if(position[i] < limit_position_inf[actuator_ids[i]-1] ):
                        
                            token.messaggio = token.messaggio + 'posizione target ' + str( position[i]) + ' del ID '+ str(actuator_ids[i]) +' è sotto il limite inferiore ' + str(limit_position_inf[actuator_ids[i]-1])+'\n'
                    
                        elif(position[actuator_ids[i]-1] > limit_position_sup[actuator_ids[i]-1]):
                        
                            token.messaggio = token.messaggio + 'posizione target ' + str( position[i]) + ' del ID ' + str(actuator_ids[i]) + ' è sopra il limite superiore ' + str(limit_position_sup[actuator_ids[i]-1])+'\n'
                        
                        token.check_posizione = False

                        return token
                else:
                    
                    token.messaggio = token.messaggio + 'id inserito: ' + str((actuator_ids[i])) + ' non valido \n'
                    token.check_posizione = False

                    
                    return token
            else:
                
                if(len(actuator_ids)==0 or len(actuator_ids)>9 ):
                    token.messaggio = token.messaggio + 'lunghezza di actuator_ids non valida:  ' + str(len(actuator_ids[i])) + '\n'
                    
                elif(len(position)==0 or len(position)>9):
                    token.messaggio = token.messaggio + 'lunghezza di position non valida:  ' + str(len(position[i]))+ '\n'
                return token
            
            
                    # ID                INPUT_UTENTE            LIMITE_SUP              LIMITE_INF             LIMITE_VELOCITA      POS_PARTENZA
                    #actuator_ids       position,velocity       limit_position_sup      limit_position_inf      limit_velocity  relative,rel_initial_positions
    elif(relative == True):

        print('Sono dentro relative')

        
        for i in range(len(actuator_ids)):
            
            if(arrivo[actuator_ids[i]-1] == False):  
             
                if(actuator_ids[i]>=1 and actuator_ids[i]<=8):

                                        
                    assoluto[actuator_ids[i]-1]  = (position[actuator_ids[i]-1] + rel_initial_positions[actuator_ids[i]-1])
                    
                    token.messaggio = token.messaggio + str(position[actuator_ids[i]-1]) + ' | ' + str(rel_initial_positions[actuator_ids[i]-1]) + ' non valido \n'


                    #if( ((position[i] + rel_initial_positions[actuator_ids[i]-1]) >= limit_position_inf[actuator_ids[i]-1])  and ( (position[i] + rel_initial_positions[actuator_ids[i]-1]) <= limit_position_sup[actuator_ids[i]-1] )):
                    
                    if((assoluto[actuator_ids[i]-1] >= limit_position_inf[actuator_ids[i]-1])  and ( assoluto[actuator_ids[i]-1] <= limit_position_sup[actuator_ids[i]-1] )):

                        if(velocity > 0.0 and velocity < limit_velocity[actuator_ids[i]-1]) :
                            prova = 0
                            
                        else: 
                            
                            if(velocity == 0.0):
                                
                                token.messaggio = token.messaggio + 'velocita ' + str(velocity) + 'è a 0.0, quindi non si muoverà \n'
                                token.check_posizione = False

                                return token

                            elif(velocity >= limit_velocity[i]):
                                
                                token.messaggio = token.messaggio + 'velocita ' + str(velocity) + 'supera il limite ' + limit_velocity[actuator_ids[i]-1] +'\n'
                                token.check_posizione = False

                                return token
                    else:
                        
                                        
                        if((position[actuator_ids[i]-1] + rel_initial_positions[actuator_ids[i]-1]) < limit_position_inf[actuator_ids[i]-1] ):
                        
                            token.messaggio = token.messaggio + 'posizione target ' + str( position[actuator_ids[i]-1] + rel_initial_positions[actuator_ids[i]-1]) + ' del ID '+ str(actuator_ids[i]) +' è sotto il limite inferiore ' + str(limit_position_inf[actuator_ids[i]-1])+'\n'
                    
                        elif((position[actuator_ids[i]-1] + rel_initial_positions[actuator_ids[i]-1]) > limit_position_sup[actuator_ids[i]-1]):
                        
                            token.messaggio = token.messaggio + 'posizione target ' + str( position[actuator_ids[i]-1] + rel_initial_positions[actuator_ids[i]-1]) + ' del ID ' + str(actuator_ids[i]) + ' è sopra il limite superiore ' + str(limit_position_sup[actuator_ids[i]-1])+'\n'
                        
                        token.messaggio = token.messaggio + str(position[actuator_ids[i]-1] + rel_initial_positions[actuator_ids[i]-1])
                
                        token.check_posizione = False

                        return token
                else:
                    
                    token.messaggio = token.messaggio + 'id inserito: ' + str((actuator_ids[i])) + ' non valido \n'
                    token.check_posizione = False

                    
                    return token
            

    
    return token
    
        # Request Fields 
        # actuator_ids = request.actuator_ids vettore
        # position = request.position vettore 
        # velocity = request.velocity
        # acceleration = request.acceleration
        # block = request.block
        # relative = request.relative   
         
    # Controllo soltanto se la richiesta è fattibile

def controll_data(request,limits_position_inf,limits_position_sup,limits_velocity,rel_initial_positions):
    
    token = Token()
    
    token.messaggio = ''    # Messaggio di uscita dalla richiesta 
    token.check_posizione = True # Sarà True soltanto i dati della request sono validi 
    
    if(request.relative == False):
        
        for i in range(len(request.actuator_ids)):
            
            # Controllo che lunghezza ID e posizioni sia la stessa --> [1,2] [1.0,1.0] True        [1,2] [1.0] False
            if(len(request.actuator_ids)>0 and len(request.position)>0 and len(request.actuator_ids)<9 and len(request.position)<9 and len(request.actuator_ids) == len(request.position) ):

                print('1 controllo passato')
            else:
                
                token.messagio = token.messagio + 'Dimensione degli input non valido \n'
                token.check_posizione = False
            # Controllo che gli ID siano stati scritti correttamente 
            if(request.actuator_ids[i]>=1 and request.actuator_ids[i]<=8):
                    
                print('Id inserito corretti')
                
            else:
                
                token.messagio = token.messagio + 'ID [' + str(request.actuator_ids[i]) + '] non valido \n'
                token.check_posizione = False

                
            # Controllo che le posizioni inserito non superino le posizioni limite del robot    
            if(request.position[i] >= (limits_position_inf[request.actuator_ids[i]-1]) and request.position[i] <= (limits_position_sup[request.actuator_ids[i]-1]) ):
                
                print('Posizione valida, rispetta i limite')
            
            else:
                
                if(request.position[i] < limits_position_inf[request.actuator_ids[i]-1] ):
                    
                    token.messagio = token.messagio + 'ID['+ str(request.actuator_ids[i])+'] Posizione [' + str(request.position[i]) + '] non rispetta il limite inf ['+ str(limits_position_inf[request.actuator_ids[i]-1])+']\n'

                elif(request.position[i] > limits_position_sup[request.actuator_ids[i]-1]):
                            
                    token.messagio = token.messagio + 'ID['+ str(request.actuator_ids[i])+'] Posizione [' + str(request.position[i]) + '] non rispetta il limite sup ['+ str(limits_position_sup[request.actuator_ids[i]-1])+']\n'
                    
                token.check_posizione = False

            if(request.velocity >= 0.0 and request.velocity < limits_velocity[request.actuator_ids[i]-1]):
                
                print('velocità inserite corrette')
            else:
                
                # if(request.velocity == 0.0):
                                    
                #     token.messaggio = token.messaggio + 'velocita ' + str(request.velocity) + 'è a 0.0, quindi non si muoverà \n'

                if(request.velocity > limits_velocity[request.actuator_ids[i]-1]):
                                    
                    token.messaggio = token.messaggio + 'velocita ' + str(request.velocity) + 'supera il limite ' + limits_velocity[request.actuator_ids[i]-1] +'\n'
                                 
                token.check_posizione = False
                
            if(request.acceleration>0.0 and request.acceleration< 5.0 ):
                
                print('accelerazione corretta')
                
            else:
                
                token.messaggio = token.messaggio + 'accelerazione ' + str(request.acceleration) + 'super i limiti ' +'\n'

                token.check_posizione = False

                
                
    else:
        print('Sono dentro relative')
        
        for i in range(len(request.actuator_ids)):
            
            # Sommo la posizione target + la posizione attuale del robot...
            assoluto[request.actuator_ids[i]-1]  = (request.position[i] + rel_initial_positions[request.actuator_ids[i]-1])

            # Controllo che lunghezza ID e posizioni sia la stessa --> [1,2] [1.0,1.0] True        [1,2] [1.0] False
            if(len(request.actuator_ids)>0 and len(request.position)>0 and len(request.actuator_ids)<9 and len(request.position)<9 and len(request.actuator_ids) == len(request.position) ):

                print('1 controllo passato')
                
            else:
                
                token.messagio = token.messagio + 'Dimensione degli input non valido \n'
                token.check_posizione = False
                
            # Controllo che gli ID siano stati scritti correttamente 
            if(request.actuator_ids[i]>=1 and request.actuator_ids[i]<=8):
                    
                print('Id inserito corretti')
                
            else:
                
                token.messagio = token.messagio + 'ID [' + str(request.actuator_ids[i]) + '] non valido \n'
                token.check_posizione = False  
                
            if((assoluto[request.actuator_ids[i]-1] >= limits_position_inf[request.actuator_ids[i]-1])  and (assoluto[request.actuator_ids[i]-1] <= limits_position_sup[request.actuator_ids[i]-1] )):

                print('Posizione valida, rispetta i limite')
            
            else:
                
                if(assoluto[request.actuator_ids[i]-1] < limits_position_inf[request.actuator_ids[i]-1] ):
                    
                    token.messagio = token.messagio + 'ID['+ str(request.actuator_ids[i]) +'] Posizione Assoluta [' + str(assoluto[request.actuator_ids[i]-1]) + '] non rispetta il limite inf ['+ str(limits_position_inf[request.actuator_ids[i]-1])+']\n'

                elif(assoluto[request.actuator_ids[i]-1] > limits_position_sup[request.actuator_ids[i]-1]):
                            
                    token.messagio = token.messagio + 'ID['+ str(request.actuator_ids[i]) +'] Posizione Assoluta[' + str(assoluto[request.actuator_ids[i]-1]) + '] non rispetta il limite sup ['+ str(limits_position_sup[request.actuator_ids[i]-1])+']\n'
                    
                token.check_posizione = False
                
            if(request.velocity >= 0.0 and request.velocity < limits_velocity[request.actuator_ids[i]-1]):
                
                print('velocità inserite corrette')
            else:
                
                # if(request.velocity == 0.0):
                                    
                #     token.messaggio = token.messaggio + 'velocita ' + str(request.velocity) + 'è a 0.0, quindi non si muoverà \n'

                if(request.velocity > limits_velocity[request.actuator_ids[i]-1]):
                    token.messaggio = token.messaggio + 'velocita ' + str(request.velocity) + 'supera il limite ' + limits_velocity[request.actuator_ids[i]-1] +'\n'
                            
                token.check_posizione = False
                
            if(request.acceleration>0.0 and request.acceleration< 5.0 ):
                
                print('accelerazione corretta')
                
            else:
                
                token.messaggio = token.messaggio + 'accelerazione ' + str(request.acceleration) + 'super i limiti ' +'\n'

                token.check_posizione = False
                
            # Controllo che sia True per inviare il messaggio di conferma
            
    if(token.check_posizione == True):
            
        token.messaggio = 'Dati inseriti correttamente'
                
        
    return token

        
            
    

            

                        
            
            

            

                
    
        
    
        