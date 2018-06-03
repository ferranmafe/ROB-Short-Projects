#!/usr/bin/python
#coding: utf8

import time
import serial
import math
import commands

# Envio de comandos al robot
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

# Obtenemos la distancia minima como la media de los 3 valores minimos en el rango establecido
def getMinDist(info, minVal, maxVal):
    aux = []
    if (minVal < maxVal):
        for i in range(minVal, maxVal):
            if 0 < int(info[i][1]) < 16627: aux.append(int(info[i][1]))
    else:
        for i in range(minVal, 359):
            if 0 < int(info[i][1]) < 16627: aux.append(int(info[i][1]))
        for i in range(2, maxVal):
            if 0 < int(info[i][1]) < 16627: aux.append(int(info[i][1]))
    aux.sort()
    if len(aux) >= 3:
        return sum(aux[:3]) / 3
    elif len(aux) != 0:
        return sum(aux) / len(aux)
    else: return -1

# Convertimos la informacion del laser en una matriz
def informationToArray (info):
    aux = info.split('\n')
    output = []
    for elem in aux:
        output.append(elem.split(','))
    return output

if __name__ == '__main__':
    global ser
    # Open the Serial Port.
    ser = serial.Serial(port='/dev/ttyACM0', baudrate=115200, timeout=0.1)
    envia(ser,'TestMode On', 0.2)
    envia(ser,'PlaySound 1', 0.3)

    # Habilitamos movimiento y sensor laser
    envia(ser,'SetMotor RWheelEnable LWheelEnable', 0.2)
    envia(ser, 'SetLDSRotation On', 0.3)

    ##### ObstacleAvoidance params #####
    # Velocidad y ditancia maxima a recorrer al enviar una orden
    maxObsSpeed = 250
    # Parametros que controlan el orden de magnitud de cada ki definida en la ecuacion lineal de movimiento
    maxObsCentralDist = 350
    maxObsLateralDist = 100
    # Distancias threshold que provocan modificaciones en el movimiento
    thLateralDistObs = 500
    thCentralDistObs = 700
    # Inicializamos distancias a recorrer por las ruedas a la maxima para que vaya recto
    leftMotorDist = maxObsCentralDist
    rightMotorDist = maxObsCentralDist

    try:
        # Bucle inicial para evitar movimientos erroneos del robot mientras el laser
        # devuelva valores incorrectos (no se ha inicializado)
        init = 1
        while init:
            # Obtenemos la informacion de los sensores: central, diagonal izquierdo y diagonal derecho
            sensorInformation = informationToArray(envia(ser, 'GetLDSScan', 0.05))
            left = getMinDist(sensorInformation, 20, 60)
            central = getMinDist(sensorInformation, 340, 20)
            right = getMinDist(sensorInformation, 300, 340)
            # Mientras todos sean -1 el laser no se ha iniciado y seguimos parados,
            # cuando se inicialice salimos del bucle
            if  central != -1 or left != -1 or right != -1:
                init = 0

        # Ejecucion infinita de esquivar obstaculos
        while 1:
            # Obtenemos la informacion de los sensores: central, diagonal izquierdo y diagonal derecho
            sensorInformation = informationToArray(envia(ser, 'GetLDSScan', 0.05))
            left = getMinDist(sensorInformation, 20, 60)
            central = getMinDist(sensorInformation, 340, 20)
            right = getMinDist(sensorInformation, 300, 340)

            # Calculamos k1 responsable de la velocidad al ir recto (cuando no hay obstaculo)
            # depende inversamente de la distancia con el obstaculo
            if central != -1: k1 = min(1, central/thCentralDistObs)
            else: k1 = 1

            # Calculamos k2 responsable de girar a la derecha para evitar obstaculo izquierda
            k2 = left / thLateralDistObs
            if not 0 <= k2 <= 1: k2 = 0

            # Calculamos k3 responsable de girar a la izquierda para evitar obstaculo derecha
            k3 = right / thLateralDistObs
            if not 0 <= k3 <= 1: k3 = 0

            # Si hay obstaculo delante pero los laterales no lo detectan entonces giramos
            # hacia el lado que tengamos mas disponible, es decir cuya distancia minima sea mas grande
            if k2 == 0 and k3 == 0 and k1 != 1:
                if left > right:
                    k3 = 1 - (central / thCentralDistObs)
                else:
                    k2 = 1 - (central / thCentralDistObs)

            # Imprimimos informacion de control
            print str(left) + " " + str(central) + " " + str(right) + "\n"

            # Calculamos cuanto se mueve cada rueda con ecuaciones lineales
            leftMotorDist = maxObsCentralDist * k1 + maxObsLateralDist * k2 - maxObsLateralDist * k3
            rightMotorDist = maxObsCentralDist * k1 - maxObsLateralDist * k2 + maxObsLateralDist * k3

            # Enviamos la orden al robot
            comando = 'SetMotor LWheelDist ' + str(leftMotorDist) + ' RWheelDist ' + str(rightMotorDist) + ' Speed ' + str(maxObsSpeed)
            envia(ser, comando, 0.05)

    except KeyboardInterrupt:
        envia(ser,'SetMotor LWheelDisable RWheelDisable', 0.05)
        envia(ser, 'SetLDSRotation Off', 0.05)
