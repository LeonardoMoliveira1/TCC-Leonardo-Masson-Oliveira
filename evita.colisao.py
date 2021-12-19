# código para movimentação do Pioneer e estabelecimento do 
# servidor Python - V-REP

import sim
import simConst
import numpy as np
#import matplotlib as mlp
import math
import sys
import time                #used to keep track of time



sim.simxFinish(-1) # just in case, close all opened connections

clientID=sim.simxStart('127.0.0.1',19997,True,True,5000,5) # Connect to CoppeliaSim

# Abre a cena desse algorítimo no ambiente de simulação #

errorCode = sim.simxLoadScene(clientID,'cena_evita_colisao.ttt',0,sim.simx_opmode_blocking)

# Escolhendo o modo de operação do código e atribuindo parâmetros #

print('Escolha o modo de Operação:\n Modo Padrão: 1   Modo Manual: 2')
automa = float(input('Resposta:'))

if automa == 1:
    kp = 0.7
    v  = 1.5
    t_simu = 20
    
else:
    kp = float(input('esterçamento:'))
    v  = float(input('velocidade inicial:'))	#forward 
    t_simu = float(input('Tempo de Simulação:'))
    

# Inicia a simulação no Ambiente do Coppelia #
    
e = sim.simxStartSimulation(clientID,sim.simx_opmode_blocking)

if clientID!=-1:
    print ('Connected to remote API server')

    # Now send some data to CoppeliaSim in a non-blocking fashion:
    sim.simxAddStatusbarMessage(clientID,'Positivo e operante!',sim.simx_opmode_oneshot)

    # Before closing the connection to CoppeliaSim, make sandure that the last comm sent out had time to arrive. You can guarantee this with (for example):
    sim.simxGetPingTime(clientID)

    # Now close the connection to CoppeliaSim:
    #sim.simxFinish(clientID)

else:
    print ('Failed connecting to remote API server')

PI=math.pi
# Alterando o handle dos motores para poder atribuir parâmetros para os mesmos.

errorCode,left_motor_handle=sim.simxGetObjectHandle(clientID,'Pioneer_p3dx_leftMotor',sim.simx_opmode_oneshot_wait)
errorCode,right_motor_handle=sim.simxGetObjectHandle(clientID,'Pioneer_p3dx_rightMotor',sim.simx_opmode_oneshot_wait)

# Atribuindo uma velocidade alvo para os motores direito e esquerdo.

errorCode=sim.simxSetJointTargetVelocity(clientID,left_motor_handle,0.2,sim.simx_opmode_streaming)
errorCode=sim.simxSetJointTargetVelocity(clientID,right_motor_handle,0.2,sim.simx_opmode_streaming)

# Alterando o handle dos sensores para atribuir parâmetros para os mesmos.

errorCode,sensor1=sim.simxGetObjectHandle(clientID,'Pioneer_p3dx_ultrasonicSensor1',sim.simx_opmode_oneshot_wait)

# Lendo os sensores 
#após a primeira leitura deve-se usar o operation mode as buffer

returnCode,detectionState,detectedPoint,detectedObjectHandle,detectedSurfaceNormalVector=sim.simxReadProximitySensor(clientID,sensor1,sim.simx_opmode_streaming)


#Lendo as saídas dos sensores:
#após primeiro uso, utilizar operation mode como buffer.

returnCode,detectionState,detectedPoint,detectedObjectHandle,detectedSurfaceNormalVector=sim.simxReadProximitySensor(clientID,sensor1,sim.simx_opmode_streaming)

# pegando o handle do sensor

errorCode,cam_handle=sim.simxGetObjectHandle(clientID,'Vision_sensor',sim.simx_opmode_oneshot_wait)

# pegando a imagem do sensor
# utilizar operation mode as buffer depois da primeira inicialização.

returnCode,resolution,image=sim.simxGetVisionSensorImage(clientID,cam_handle,0,sim.simx_opmode_streaming)

##############



sensor_h=[] #lista vazia para armazenar os handles dos sensores.
sensor_val=np.array([]) #numpy list para armezar os valores dos sensores.

#orientation of all the sensors: 
sensor_loc=np.array([-PI/2, -50/180.0*PI,-30/180.0*PI,-10/180.0*PI,10/180.0*PI,30/180.0*PI,50/180.0*PI,PI/2,PI/2,130/180.0*PI,150/180.0*PI,170/180.0*PI,-170/180.0*PI,-150/180.0*PI,-130/180.0*PI,-PI/2]) 

#for loop to retrieve sensor arrays and initiate sensors
for x in range(1,16+1):
        errorCode,sensor_handle=sim.simxGetObjectHandle(clientID,'Pioneer_p3dx_ultrasonicSensor'+str(x),sim.simx_opmode_oneshot_wait)
        sensor_h.append(sensor_handle) #keep list of handles        
        errorCode,detectionState,detectedPoint,detectedObjectHandle,detectedSurfaceNormalVector=sim.simxReadProximitySensor(clientID,sensor_handle,sim.simx_opmode_streaming)                
        sensor_val=np.append(sensor_val,np.linalg.norm(detectedPoint)) #transforma uma medida 3d em uma medida 1d.

t= time.time()

while (time.time()-t)<t_simu:
    #Loop Execution
    sensor_val=np.array([])    
    for x in range(1,16+1):
        errorCode,detectionState,detectedPoint,detectedObjectHandle,detectedSurfaceNormalVector=sim.simxReadProximitySensor(clientID,sensor_h[x-1],sim.simx_opmode_buffer)                
        sensor_val=np.append(sensor_val,np.linalg.norm(detectedPoint)) #get list of values

    
    #controller specific
    sensor_sq=sensor_val[0:8]*sensor_val[0:8] #square the values of front-facing sensors 1-8/ dá mais valor pros valores pequennos
        
    min_ind=np.where(sensor_sq==np.min(sensor_sq)) #encontra o sensor mais próximo de um obstáculo.
    min_ind=min_ind[0][0]  #transforma o array em um número.
    
    if sensor_sq[min_ind]<0.2:
        steer=-1/sensor_loc[min_ind]
    else:
        steer=0
            
    
   # v=1	#forward velocity

   #
    
    vl=v+kp*steer
    vr=v-kp*steer
    
    # print ("V_l =",vl)
    # print ("V_r =",vr)

    errorCode=sim.simxSetJointTargetVelocity(clientID,left_motor_handle,vl, sim.simx_opmode_streaming)
    errorCode=sim.simxSetJointTargetVelocity(clientID,right_motor_handle,vr, sim.simx_opmode_streaming)


    time.sleep(0.2) #loop executes once every 0.2 seconds (= 5 Hz)
    
    #Post ALlocation
errorCode=sim.simxSetJointTargetVelocity(clientID,left_motor_handle,0, sim.simx_opmode_streaming)
errorCode=sim.simxSetJointTargetVelocity(clientID,right_motor_handle,0, sim.simx_opmode_streaming)
    

print ('Program ended')

# Now close the connection to CoppeliaSim:
# sim.simxFinish(clientID)
