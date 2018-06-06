#!/usr/bin/python
#coding: utf8

import os
import sys, tty, termios
from math import sin, cos, fmod, atan2, sqrt, pow, fabs

import time
import serial
import math
import commands

# Funciones matrices para la odometria
def matrix_sum(A, B):
    C = []
    for i in range(len(A)):
        row = []
        for j in range(len(A[i])):
            row.append(A[i][j] + B[i][j])
        C.append(row)
    return C

def matrix_dif(A, B):
    C = []
    for i in range(len(A)):
        row = []
        for j in range(len(A[i])):
            row.append(A[i][j] - B[i][j])
        C.append(row)
    return C

def matrix_zeros(rows, columns):
    C = []
    for i in range(rows):
        aux = []
        for j in range(columns):
            aux.append(0)
        C.append(aux)
    return C

def matrix_mul(A, B):
    C = matrix_zeros(len(A),len(B[0]))
    for i in range(len(C)):
        for j in range(len(C[0])):
            for k in range(len(A[0])):
                C[i][j] += A[i][k] * B[k][j]
    return C

def transpose(A):
    C = matrix_zeros(len(A[0]), len(A))
    for i in range(len(C)):
        for j in range(len(C[0])):
            C[i][j] = A[j][i]
    return C

def diag(V):
    C = matrix_zeros(len(V), 1)
    for i in range(len(C)):
            C[i][0] = V[i][i]
    return C

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

# Obtenemos las distancias recorridas por cada rueda del robot
def get_motors():
    msg = envia(ser, 'GetMotors LeftWheel RightWheel').split('\n')
    L = int(msg[4].split(',')[1])
    R = int(msg[8].split(',')[1])
    return (L, R)

# Calculamos la odometria
def odometry(Lnew, Rnew):
    global L_ini, R_ini, S, V, Pk0
    global pose_est1, pose_t, Pk1, x_w, y_w, sum_theta

    L, R = Lnew - L_ini, Rnew - R_ini

    delta_d = (L + R) / 2
    delta_th = (R - L) / (2 * S)

    F_x = [[1., 0., -(delta_d * sin(sum_theta + delta_th))], [0., 1., (delta_d * cos(sum_theta+delta_th))], [0., 0., 1.]];
    F_v = [[cos(sum_theta + delta_th), -delta_d * sin(sum_theta + delta_th)], [sin(sum_theta+delta_th), delta_d*cos(sum_theta+delta_th)], [0., 1.]];

    # Pose_est1 = Pose_est1 + F_x*(Pose_t - Pose_est1) + F_v*diag(V);
    pose_est1 = matrix_sum(pose_est1, matrix_sum(matrix_mul(F_x, matrix_dif(pose_t, pose_est1)), matrix_mul(F_v, diag(V))))
    # Pk1 = F_x*Pk1*F_x' + F_v*V*F_v';
    Pk1 = matrix_sum(matrix_mul(F_x, matrix_mul(Pk1, transpose(F_x))), matrix_mul(F_v, matrix_mul(V,transpose(F_v))))

    x_w = x_w + (delta_d + V[0][0]) * cos(sum_theta + delta_th + V[1][1])
    y_w = y_w + (delta_d + V[0][0]) * sin(sum_theta + delta_th + V[1][1])
    sum_theta = sum_theta + delta_th + V[1][1]
    if (sum_theta < 0):
        sum_theta += 2 * 3.1415
    elif (sum_theta > 2 * 3.1415):
        sum_theta -= 2 * 3.1415
    pose_t = transpose([[x_w, y_w, sum_theta]])

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

# Funcion que engloba toda la actualizacion de la odometria (llamar a cada iteracion del bucle)
def updateOdometry():
    global L_ini, R_ini, x_w, y_w, sum_theta
    L, R = get_motors()
    # Calculamos la odometria (con incertidumbre)
    odometry(L, R)
    # Actualizamos los valores L_ini, R_ini con L y R para poder calcular el siguiente incremento
    L_ini, R_ini = L, R
    print str(x_w) + " " + str(y_w) + " " + str(sum_theta)

# Funcion que implementa un wall following (muro izquierdo) modificado para parar cuando ha recuperado direccion inicial
# La idea es evitar el obstaculo resiguiendolo como si fuera un muro
def followObstacleRight():
    print "Entro follow Right"
    global maxWallSpeed, constWallDist, maxWallCentralDist, maxWallDiagDist, maxWallLateralDist
    global thLateralDistWall, thCentralDistWall, thDiagDistWall, thRangeWall, y_w, y_w_th

    avoided = False

    leftMotorDist = maxWallCentralDist
    rightMotorDist = maxWallCentralDist

    # Obtenemos la informacion de los sensores: central, lateral izquierdo y diagonal izquierdo
    sensorInformation = informationToArray(envia(ser, 'GetLDSScan', 0.05))
    rightDist = getMedianDist(sensorInformation,  250, 300)
    diagDist = getMedianDist(sensorInformation,  300, 340)
    centralDist = getMedianDist(sensorInformation, 340, 20)

    # Ejecucion inicial en busqueda de un muro, mientras las distancias no sean inferiores
    # a los threshold definidos el robot no ha encontrado ningun muro
    while not ((-1 < rightDist < thLateralDistWall) or (-1 < centralDist < thCentralDistWall) or  (-1 < diagDist < thDiagDistWall)):
        updateOdometry()
        # Obtenemos la informacion de los sensores: central, lateral derecho y diagonal derecho
        sensorInformation = informationToArray(envia(ser, 'GetLDSScan', 0.05))
        rightDist = getMedianDist(sensorInformation,  250, 300)
        diagDist = getMedianDist(sensorInformation,  300, 340)
        centralDist = getMedianDist(sensorInformation, 340, 20)

        # Imprimimos informacion de control
        print "Distancias: " + str(centralDist) + " " + str(rightDist)

        # Enviamos la orden al robot que se mueva recto
        envia(ser, 'SetMotor LWheelDist ' + str(leftMotorDist) + ' RWheelDist ' + str(rightMotorDist) + ' Speed ' + str(maxWallSpeed), 0.05)


    while not avoided or y_w > y_w_th:
        updateOdometry()

        if (y_w > y_w_th):
            avoided = True

        # Obtenemos la informacion de los sensores: central, lateral derecho y diagonal derecho
        sensorInformation = informationToArray(envia(ser, 'GetLDSScan', 0.05))
        rightDist = getMedianDist(sensorInformation,  250, 300)
        diagDist = getMedianDist(sensorInformation,  300, 340)
        centralDist = getMedianDist(sensorInformation, 340, 20)

        # Calculamos la constante k1 responsable de girar a la izquierda cuando haya un muro delante
        if -1 < centralDist < thCentralDistWall: k1 = 1 - centralDist/thCentralDistWall
        elif -1 < diagDist < thDiagDistWall: k1 = 1 - diagDist/thDiagDistWall
        else: k1 = 0

        # Calculamos k2 responsable de mantener la distancia con el muro lateral
        if thLateralDistWall - thRangeWall < rightDist < thLateralDistWall + thRangeWall: k2 = 0
        elif -1 < rightDist <= thLateralDistWall - thRangeWall: k2 = - (1 - rightDist/thLateralDistWall)
        elif -1 == rightDist: k2 = 0.5
        else: k2 = (1 - rightDist/thLateralDistWall)

        # Calculamos k3 responsable de girar a la derecha cuando desaparece el muro que seguimos
        if diagDist < thDiagDistWall and diagDist != -1: k3 = 0
        else: k3 = 0.7

        # Imprimimos informacion de control
        print "Distancias-> central:" + str(centralDist) + " diagonal:" +  str(diagDist) + " lateral:" + str(rightDist)
        print "K's -> k1: " + str(k1) + " k2: " + str(k2) + " k3: " + str(k3)

        # Calculamos cuanto se mueve cada rueda con ecuaciones lineales, notar que hay una distancia fija pq las 3 Ki pueden ser 0
        leftMotorDist =  constWallDist - maxWallCentralDist * k1 + maxWallLateralDist * k2 + maxWallDiagDist * k3
        rightMotorDist = constWallDist  + maxWallCentralDist * k1 - maxWallLateralDist * k2 - maxWallDiagDist * k3

        envia(ser, 'SetMotor LWheelDist ' + str(leftMotorDist) + ' RWheelDist ' + str(rightMotorDist) + ' Speed ' + str(maxWallSpeed), 0.05)

    print "Salgo follow Right"

# Funcion que implementa un wall following (muro derecho) modificado para parar cuando ha recuperado direccion inicial
# La idea es evitar el obstaculo resiguiendolo como si fuera un muro
def followObstacleLeft():
    print "Entro follow Left"
    global maxWallSpeed, constWallDist, maxWallCentralDist, maxWallDiagDist, maxWallLateralDist
    global thLateralDistWall, thCentralDistWall, thDiagDistWall, thRangeWall, y_w, y_w_th

    avoided = False

    leftMotorDist = maxWallCentralDist
    rightMotorDist = maxWallCentralDist

    # Obtenemos la informacion de los sensores: central, lateral izquierdo y diagonal izquierdo
    sensorInformation = informationToArray(envia(ser, 'GetLDSScan', 0.05))
    leftDist = getMedianDist(sensorInformation, 60, 110)
    diagDist = getMedianDist(sensorInformation, 20, 60)
    centralDist = getMedianDist(sensorInformation, 340, 20)

    # Ejecucion inicial en busqueda de un muro, mientras las distancias no sean inferiores
    # a los threshold definidos el robot no ha encontrado ningun muro
    while not ((-1 < leftDist < thLateralDistWall) or (-1 < centralDist < thCentralDistWall) or  (-1 < diagDist < thDiagDistWall)):
        updateOdometry()
        # Obtenemos la informacion de los sensores: central, lateral izquierdo y diagonal izquierdo
        sensorInformation = informationToArray(envia(ser, 'GetLDSScan', 0.05))
        leftDist = getMedianDist(sensorInformation, 60, 110)
        diagDist = getMedianDist(sensorInformation, 20, 60)
        centralDist = getMedianDist(sensorInformation, 340, 20)

        # Imprimimos informacion de control
        print "Distancias-> central:" + str(centralDist) + " diagonal:" +  str(diagDist) + " lateral:" + str(leftDist)

        # Enviamos la orden al robot que se mueva recto
        envia(ser, 'SetMotor LWheelDist ' + str(leftMotorDist) + ' RWheelDist ' + str(rightMotorDist) + ' Speed ' + str(maxWallSpeed), 0.05)


    while not avoided or y_w < -y_w_th:
        updateOdometry()

        if y_w < -y_w_th:
            avoided = True

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

        # Enviamos la orden al robot
        envia(ser, 'SetMotor LWheelDist ' + str(leftMotorDist) + ' RWheelDist ' + str(rightMotorDist) + ' Speed ' + str(maxWallSpeed), 0.05)

    print "Salgo follow Left"

if __name__ == '__main__':
    global ser
    # Open the Serial Port.
    ser = serial.Serial(port='/dev/ttyACM0', baudrate=115200, timeout=0.05)
    envia(ser, 'TestMode On')
    envia(ser, 'PlaySound 1')

    # Habilitamos movimiento y sensor laser
    envia(ser,'SetMotor RWheelEnable LWheelEnable', 0.2)
    envia(ser, 'SetLDSRotation On', 0.3)

    ##### Odometry params #####
    S = 121.5 # en mm
    V = [[0.01 ** 2, 0.],[0., 0.001 ** 2]]
    Pk0 = [[0.0001, 0.0001, 0.0001],[0.0001, 0.0001, 0.0001],[0.0001, 0.0001, 0.0001]]

    x_w = 0
    y_w = 0
    sum_theta = 0

    pose_est1 = transpose([[x_w, y_w, sum_theta]])
    pose_t = transpose([[x_w, y_w, sum_theta]])
    Pk1 = Pk0
    ###########################
    ##### ObstacleAvoidance params #####
    # Velocidad y ditancia maxima a recorrer al enviar una orden
    maxObsSpeed = 100
    # Parametros que controlan el orden de magnitud de cada ki definida en la ecuacion lineal de movimiento
    maxObsCentralDist = 350
    maxObsLateralDist = 100
    # Distancias threshold que provocan modificaciones en el movimiento
    thLateralDistObs = 500
    thCentralDistObs = 700
    ################################
    ##### WallFollowing params #####
    # Velocidad y ditancia maxima a recorrer al enviar una orden
    maxWallSpeed = 100
    constWallDist = 300
    # Parametros que controlan el orden de magnitud de cada ki definida en la ecuacion lineal de movimiento
    maxWallCentralDist = 700
    maxWallDiagDist = 200
    maxWallLateralDist = 150
    # Distancias threshold que provocan modificaciones en el movimiento
    thLateralDistWall = 500
    thCentralDistWall = 900
    thDiagDistWall = 700
    thRangeWall = 20
    ###############################
    ##### Repositioning param #####
    y_w_th = 50 # 5 centimetros de margen
    sum_theta_th = 5 * 3.1415 / 180 # Threshold de 3 grados


    #Obtenemos distancia inicial de cada rueda
    L_ini, R_ini = get_motors()

    # Inicializamos distancias a recorrer al maximo para que vaya recto
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

        # Comportamiento ideal hay 2 casoss:
        # - Ir recto como obstacleAvoidance, hasta que detectemos un objeto entonces girar
        #   hacia el lado correspondiente y entrar en modo WallFollowing hasta que la distancia
        #   de la componente y_w se recupere a 0.
        # - Si el angulo orientado es mayor que un threshold respecto al angulo inicial (zero grados)
        #   hay que reorientarse
        while 1:
            # Siempre acutalizamos la odometria
            updateOdometry()
            # Comportamiento de obstacleAvoidance si el angulo girado del robot es 0 +- threshold
            if sum_theta < sum_theta_th or sum_theta > (2 * 3.1415 - sum_theta_th):
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
                # Hay obstaculo y lo evitamos girando a la izquierda (seguiremos objeto a la derecha)
                if k3 != 0:
                    leftMotorDist = maxObsCentralDist * k1 + maxObsLateralDist * k2 - maxObsLateralDist * k3
                    rightMotorDist = maxObsCentralDist * k1 - maxObsLateralDist * k2 + maxObsLateralDist * k3
                    comando = 'SetMotor LWheelDist ' + str(leftMotorDist) + ' RWheelDist ' + str(rightMotorDist) + ' Speed ' + str(maxObsSpeed)
                    #envia(ser, comando, 0.05)
                    followObstacleRight()
                # Hay obstaculo y lo evitamos girando a la derecha (seguiremos objeto a la izquierda)
                elif k2 != 0:
                    leftMotorDist = maxObsCentralDist * k1 + maxObsLateralDist * k2 - maxObsLateralDist * k3
                    rightMotorDist = maxObsCentralDist * k1 - maxObsLateralDist * k2 + maxObsLateralDist * k3
                    comando = 'SetMotor LWheelDist ' + str(leftMotorDist) + ' RWheelDist ' + str(rightMotorDist) + ' Speed ' + str(maxObsSpeed)
                    #envia(ser, comando, 0.05)
                    followObstacleLeft()
                # No hay obstaculo
                else:
                    leftMotorDist = maxObsCentralDist * k1 + maxObsLateralDist * k2 - maxObsLateralDist * k3
                    rightMotorDist = maxObsCentralDist * k1 - maxObsLateralDist * k2 + maxObsLateralDist * k3
                    comando = 'SetMotor LWheelDist ' + str(leftMotorDist) + ' RWheelDist ' + str(rightMotorDist) + ' Speed ' + str(maxObsSpeed)
                    envia(ser, comando, 0.05)

            # Nos reorientamos si no estamos a 0 grados pq significa que ya hemos evitado obstaculo
            else:
                # Si el angulo es mas pequeño que 180 giramos a la derecha
                if sum_theta < 3.1415:
                    k = sum_theta / 3.1415
                # Si el angulo es mas grande giramos a la izquierda
                else:
                    k = -(2 * 3.1415 - sum_theta)/3.1415

                leftMotorDist = maxObsCentralDist * k
                rightMotorDist = -maxObsCentralDist * k
                comando = 'SetMotor LWheelDist ' + str(leftMotorDist) + ' RWheelDist ' + str(rightMotorDist) + ' Speed ' + str(maxObsSpeed)
                envia(ser, comando, 0.05)

    except KeyboardInterrupt:
        envia(ser, 'SetLDSRotation Off', 0.2)
