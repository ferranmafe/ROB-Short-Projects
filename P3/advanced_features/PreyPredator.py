#!/usr/bin/python
#coding: utf8

import time
import serial
import math
import commands

# Función que envia comandos al Neato durante un tiempo especificado por el usuario
# (facilitada para el Short Project 2 por Antonio)
def envia(ser, missatge,temps=0.1, show_time = False):
    first_time=time.time()
    rbuffer = ''
    resp = ''
    ser.write(missatge + chr(10))
    time.sleep(temps) # giving a breath to pi
    while ser.inWaiting() > 0:
        resp = ser.readline()
        rbuffer = rbuffer + resp
    if show_time:
        t = time.time() - first_time
        print("Round time: ",t)
    return rbuffer
    
def median(lst):
    n = len(lst)
    if n < 1:
            return None
    if n % 2 == 1:
            return sorted(lst)[n//2]
    else:
            return sum(sorted(lst)[n//2-1:n//2+1])/2.0
            
# Funcion que encuentra el objeto más cercano al neato, descartando muros
def getMinDist(info):
    global robotFrontDist, groupSize, numSimSens, wallTh

    # Nos definimos un vector auxiliar con 360 posiciones (una para cada grado que devuelve el scanner del Neato.
    # Nos quedamos con los valores con valores coherentes, y el resto quedan como None
    aux = [None] * 360
    for i in range(2, 362):
        if 0 < int(info[i][1]) < 16602: aux[int(info[i][0])] = int(info[i][1])

    # Una vez tenemos los valores, los discretizamos haciendo grupos de tamaño groupSize. Cada sensor resultante
    # de la discretización marcará como distancia la media de los valores usados para discretizar. Si en algun
    # caso para un sensor todos los valores fueran None, el sensor tendría como valor None.
    # Nota: el indice del vector de sensores indica a que angulo hace referencia el valor almacenado en ese indice,
    # siendo el angulo del cada sensor su índice en el vector multiplicado por groupSize
        
    sensors = []
    for i in range(len(aux)/groupSize):
        values = []
        for j in range(groupSize):
            mod = ((i * groupSize) + j) % 360
            if aux[mod] is not None:
                values.append(aux[mod])
        if (len(values) > 0): 
            print values
            print float(sum(values)) / max(len(values), 1)
            sensors.append(float(sum(values)) / max(len(values), 1))
        else: sensors.append(None)
    
    for i in range(len(sensors)):
        if sensors[i] is None:
            print str(i) + " None"
        else:
            print str(i) + " " + str(sensors[i])
    # Una vez tenemos los sensores discretizados recorremos el vector de sensores para buscar muros.
    # Para cada sensor, recorremos tantos sensores como indique la variable (numSimSens), y si todos los
    # sensores por los que pasamos son similares (No son None y su diferencia es menor al threshold) consideramos
    # que están leyendo muro y lo guardamos en la variable wallSet
    wallSet = set([])
    sensorsChecked = [False] * len(sensors)
    while not all(item for item in sensorsChecked):
        i = getNextNotCheckedIndex(sensorsChecked)
        if sensors[i] is not None:
            act = sensors[i]
            muro = True
            j = 1
            while j < numSimSens and muro:
                mod = (i + j) % len(sensors)
                if sensors[mod] is None:
                    muro = False
                elif abs(sensors[mod] - act) > wallTh:
                    muro = False
                else:
                    act = sensors[mod]
                    j += 1
                    
            if muro:
                for j in range(numSimSens):
                    wallSet.add((i + j)% len(sensors))
        
        sensorsChecked[i] = True

    # Una vez tenemso los muros, recuperamos los sensores que no detectan muro y no son nulos
    final_sensors = []
    for i in range(len(sensors)):
        if i not in wallSet and sensors[i] is not None:
            final_sensors.append((i * groupSize, sensors[i]))
    

    # Finalmente, obtenemos la lectura más pequeña de los sensores leidos. Retornamos la tupla
    # (angulo, distancia)
    if (len(final_sensors) > 0):
        minim = final_sensors[0]
        for i in range(len(final_sensors)):
            if minim[1] > final_sensors[i][1]:
                minim = final_sensors[i]
        return minim
        
    else: return (0, robotFrontDist)

# Funcion que dado un vector de booleanos, retorna el indice del elemento a False. Si no hay ninguno retorna -1
def getNextNotCheckedIndex(array):
    for i in range(len(array)):
        if not array[i]: return i
    return -1

# Función que dado un string de información de los laseres dada por el neato, retorna una matriz con esta informacion
def informationToArray (info):
    aux = info.split('\n')
    output = []
    for elem in aux:
        output.append(elem.split(','))
    return output

# Obtenemos la distancia minima como la media de los 3 valores minimos en el rango establecido
def getMinDistWithoutAvoidingWalls(info):
    global robotFrontDist
    aux = []
    for i in range(2, 362):
        if 0 < int(info[i][1]) < 16627: aux.append((int(info[i][0]), (int(info[i][1]))))

    if len(aux) > 0:
        minim = aux[0]
        for i in range(1, len(aux)):
            if minim[1] > aux[i][1]: minim = aux[i]

        return minim
    # Si todos sus sensores detectan -1 el robot puede tener el laser apagado lo hacemos pararse
    else: return (0, robotFrontDist)

if __name__ == '__main__':
    global ser
    # Abrimos el puerto serie y activamos el modo de test y el sonido
    ser = serial.Serial(port='/dev/ttyACM0', baudrate=115200, timeout=0.1)
    envia(ser,'TestMode On', 0.2)
    envia(ser,'PlaySound 1', 0.3)

    # Activamos los motores y los laseres
    envia(ser,'SetMotor RWheelEnable LWheelEnable', 0.2)
    envia(ser, 'SetLDSRotation On', 0.3)

    # Velocidad máxima del robot
    maxSpeed = 300
    # Distáncia máxima a recorrer en una solo comando para moverse al Neato
    maxDist = 400

    # Constante que limita cuan cerca está el robot del obstaculo
    robotFrontDist = 245
    # Distancia a la que el robot empieza a huir
    robotAvoidDist = 1000
    # Numero de angulos a discretizar como un sensor
    groupSize = 10
    # Numero de sensores con valores similares para ser considerados muro
    numSimSens = 5
    # Threshold para marcar cuan diferentes han de ser dos lecturas consecutivas para ser considerado que no es un muro
    wallTh = 500

    # Inicializamos cada motor a una distáncia de maxDist
    leftMotorDist = maxDist
    rightMotorDist = maxDist
    frontDist = 400

    # Pedimos al usuario que rol quiere desempeñar
    playerType = raw_input('Please, type if you want to be "Prey" or "Predator":\n')

    while not (playerType == "Predator" or playerType == "Prey"):
        playerType = raw_input('Role not recognized. Please, type if you want to be "Prey" or "Predator":\n')

    try:
        while 1:
            # Recogemos los datos del laser y obtenemos la minima distáncia
            data = informationToArray(envia(ser, 'GetLDSScan', 0.05))

            # Si el rol es depredador
            if playerType == "Predator":
                minDist = getMinDist(data)
                print("Min Dist: {}".format(minDist))
                # Modificamos los valores de k para acercarnos a la distáncia mínima
                
                if (minDist[0] <= 180):
                    k = minDist[0]/180.
                else:
                    k = -(360 - minDist[0])/180.
                # Computamos las ecuaciones lineales para calcular cuanto se ha de mover cada rueda
                rightMotorDist = maxDist * k + (minDist[1] - robotFrontDist)
                leftMotorDist = -maxDist * k + (minDist[1] - robotFrontDist)
                print("leftMotorDist: {} rightMotorDist: {}".format(rightMotorDist, leftMotorDist))

                # Enviamos el resultado al neato para que se mueva
                envia(ser, 'SetMotor LWheelDist ' + str(leftMotorDist) + ' RWheelDist ' + str(rightMotorDist) + ' Speed ' + str(maxSpeed), 0.05)
                
            # Si el rol es presa
            elif playerType == "Prey":
                minDist = getMinDistWithoutAvoidingWalls(data)
                print("Min Dist: {}".format(minDist))
                # Modificamos los valores de k para alejarnos de la distáncia mínima
                if (minDist[0] <= 180):
                    k = 1 - (minDist[0] / 180.)
                else:
                    k = -(1 - (360 - minDist[0]) / 180.)
                
                # Computamos las ecuaciones lineales para calcular cuanto se ha de mover cada rueda
                if minDist[0] < robotAvoidDist:
                    rightMotorDist = -maxDist * k + frontDist
                    leftMotorDist = maxDist * k + frontDist
                    print("leftMotorDist: {} rightMotorDist: {}".format(rightMotorDist, leftMotorDist))

                    # Enviamos el resultado al neato para que se mueva
                    envia(ser, 'SetMotor LWheelDist ' + str(leftMotorDist) + ' RWheelDist ' + str(rightMotorDist) + ' Speed ' + str(maxSpeed), 0.05)

    except KeyboardInterrupt:
        envia(ser, 'SetLDSRotation Off', 0.2)
