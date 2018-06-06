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

# Obtenemos la distancia media de los valores leidos por un rango de angulos del sensor
def getMedianDist(info, minVal, maxVal):
    aux = []
    if (minVal < maxVal):
        for i in range(minVal, maxVal):
            if 0 < int(info[i][1]) < 16627: aux.append(int(info[i][1]))
    else:
        for i in range(minVal, 359):
            if 0 < int(info[i][1]) < 16627: aux.append(int(info[i][1]))
        for i in range(2, maxVal):
            if 0 < int(info[i][1]) < 16627: aux.append(int(info[i][1]))

    if len(aux) > 0: return float(sum(aux)) / max(len(aux), 1)
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

    ##### WallFollowing params #####
    # Velocidad y ditancia maxima a recorrer al enviar una orden
    maxWallSpeed = 100
    constWallDist = 300
    # Parametros que controlan el orden de magnitud de cada ki definida en la ecuacion lineal de movimiento
    maxWallCentralDist = 700
    maxWallDiagDist = 200
    maxWallLateralDist = 100
    # Distancias threshold que provocan modificaciones en el movimiento
    thLateralDistWall = 500
    thCentralDistWall = 900
    thDiagDistWall = 700
    thRangeWall = 20
    ###############################

    # Inicializamos distancias a recorrer por las ruedas a la maxima para que vaya recto
    leftMotorDist = maxWallCentralDist
    rightMotorDist = maxWallCentralDist

    try:
        # Bucle inicial para evitar movimientos erroneos del robot mientras el laser
        # devuelva valores incorrectos (no se ha inicializado)
        init = 1
        while init:
            # Obtenemos la informacion de los sensores: central, lateral izquierdo y diagonal izquierdo
            sensorInformation = informationToArray(envia(ser, 'GetLDSScan', 0.05))
            leftDist = getMedianDist(sensorInformation, 60, 110)
            diagDist = getMedianDist(sensorInformation, 20, 60)
            centralDist = getMedianDist(sensorInformation, 340, 20)
            # Mientras todos sean -1 el laser no se ha iniciado y seguimos parados,
            # cuando se inicialice salimos del bucle
            if  leftDist != -1 or centralDist != -1 or diagDist != -1:
                init = 0

        # Ejecucion inicial en busqueda de un muro, mientras las distancias no sean inferiores
        # a los threshold definidos el robot no ha encontrado ningun muro
        while not ((-1 < leftDist < thLateralDistWall) or (-1 < centralDist < thCentralDistWall) or  (-1 < diagDist < thDiagDistWall)):
            # Obtenemos la informacion de los sensores: central, lateral izquierdo y diagonal izquierdo
            sensorInformation = informationToArray(envia(ser, 'GetLDSScan', 0.05))
            leftDist = getMedianDist(sensorInformation, 60, 110)
            diagDist = getMedianDist(sensorInformation, 20, 60)
            centralDist = getMedianDist(sensorInformation, 340, 20)

            # Imprimimos informacion de control
            print "Distancias-> central:" + str(centralDist) + " diagonal:" +  str(diagDist) + " lateral:" + str(leftDist)

            # Enviamos la orden al robot que se mueva recto
            envia(ser, 'SetMotor LWheelDist ' + str(leftMotorDist) + ' RWheelDist ' + str(rightMotorDist) + ' Speed ' + str(maxWallSpeed), 0.05)

        # Ejecucion infinita de seguimiento de muro una vez hemos encontrado el primer muro
        while 1:
            # Obtenemos la informacion de los sensores: central, lateral izquierdo y diagonal izquierdo
            sensorInformation = informationToArray(envia(ser, 'GetLDSScan', 0.05))
            leftDist = getMedianDist(sensorInformation, 60, 110)
            diagDist = getMedianDist(sensorInformation, 20, 60)
            centralDist = getMedianDist(sensorInformation, 340, 20)

            # Calculamos la constante k1 responsable de girar a la derecha cuando haya un muro delante
            if -1 < centralDist < thCentralDistWall: k1 = 1 - centralDist/thCentralDistWall
            elif -1 < diagDist < thDiagDistWall: k1 = 1 - diagDist/thDiagDistWall
            else: k1 = 0

            # Calculamos k2 responsable de mantener la distancia con el muro lateral
            if thLateralDistWall - thRangeWall < leftDist < thLateralDistWall + thRangeWall: k2 = 0
            elif -1 < leftDist <= thLateralDistWall - thRangeWall: k2 = - (1 - leftDist/thLateralDistWall)
            elif -1 == leftDist: k2 = 0.5
            else: k2 = (1 - leftDist/thLateralDistWall)

            # Calculamos k3 responsable de girar a la izquierda cuando desaparece el muro que seguimos
            if diagDist < thDiagDistWall and diagDist != -1: k3 = 0
            else: k3 = 0.7

            # Imprimimos informacion de control
            print "Distancias-> central:" + str(centralDist) + " diagonal:" +  str(diagDist) + " lateral:" + str(leftDist)
            print "K's -> k1: " + str(k1) + " k2: " + str(k2) + " k3: " + str(k3)

            # Calculamos cuanto se mueve cada rueda con ecuaciones lineales, notar que hay una distancia fija pq las 3 Ki pueden ser 0
            leftMotorDist =  constWallDist + maxWallCentralDist * k1 - maxWallLateralDist * k2 - maxWallDiagDist * k3
            rightMotorDist = constWallDist - maxWallCentralDist * k1 + maxWallLateralDist * k2 + maxWallDiagDist * k3

            print "leftMotorDist: " + str(leftMotorDist) + " rightMotorDist: " + str(rightMotorDist)

            # Enviamos la orden al robot
            envia(ser, 'SetMotor LWheelDist ' + str(leftMotorDist) + ' RWheelDist ' + str(rightMotorDist) + ' Speed ' + str(maxWallSpeed), 0.05)

    except KeyboardInterrupt:
        envia(ser, 'SetLDSRotation Off', 0.05)
