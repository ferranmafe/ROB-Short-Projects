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

# Obtenemos la distancia minima y el angulo en e lque se produce
def getMinDist(info):
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

    ##### MeetAnObject params #####
    maxSpeed = 200
    maxDist = 400

    robotFrontDist = 245

    leftMotorDist = maxDist
    rightMotorDist = maxDist

    try:
        while 1:
            # Obtenemos la informacion del laser y guardamos la distancia y angulo minimos
            data = informationToArray(envia(ser, 'GetLDSScan', 0.05))
            minDist = getMinDist(data)

            # Calculamos la k responsable del giro del robot
            # Si es menor que 180 la distancia minimoa giramos hacia la izquierda
            if (minDist[0] <= 180):
                k = minDist[0]/180.
            # Si es mayor que 180 hacia la derecha por eso aparece el negativo delante
            else:
                k = -(360 - minDist[0])/180.

            # Ecuaciones lineales con la parte de giro en funcion de la k, y una
            # parte encargada de hacerlo moverse hacia delante hasta que la distancia minima detectada
            # sea menor que la distancia entre el laser y el frente del robot (para que no se choque infinitamente)
            rightMotorDist = maxDist * k + (minDist[1] - robotFrontDist)
            leftMotorDist = -maxDist * k + (minDist[1] - robotFrontDist)

            # Imprimimos informacion de control
            print("Min Dist: {}".format(minDist))
            print("leftMotorDist: {} rightMotorDist: {}".format(rightMotorDist, leftMotorDist))

            # Enviamos la orden al robot
            envia(ser, 'SetMotor LWheelDist ' + str(leftMotorDist) + ' RWheelDist ' + str(rightMotorDist) + ' Speed ' + str(maxSpeed), 0.05)

    except KeyboardInterrupt:
        envia(ser, 'SetLDSRotation Off', 0.05)
