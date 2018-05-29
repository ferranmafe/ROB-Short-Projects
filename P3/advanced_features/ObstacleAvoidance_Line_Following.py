#!/usr/bin/python
#coding: utf8

import os
import sys, tty, termios
from math import sin, cos, fmod, atan2, sqrt, pow

import time
import serial
import math
import commands

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

def get_motors():
    msg = envia(ser, 'GetMotors LeftWheel RightWheel').split('\n')
    L = int(msg[4].split(',')[1])
    R = int(msg[8].split(',')[1])
    return (L, R)

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
    pose_t = transpose([[x_w, y_w, sum_theta]])

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

    aux.sort()
    if len(aux) >= 3:
        return sum(aux[:3]) / 3
    elif len(aux) != 0:
        return sum(aux) / len(aux)
    else: return -1

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
    envia(ser, 'SetLDSRotation On', 0.4)

    global ser
    ser = serial.Serial(port='/dev/ttyACM0', baudrate=115200, timeout=0.05)
    envia(ser, 'TestMode On')
    envia(ser, 'PlaySound 1')
    envia(ser ,'SetMotor RWheelEnable LWheelEnable')
    envia(ser, 'SetLDSRotation On')

    S = 121.5 # en mm
    V = [[0.01 ** 2, 0.],[0., 0.001 ** 2]]
    Pk0 = [[0.0001, 0.0001, 0.0001],[0.0001, 0.0001, 0.0001],[0.0001, 0.0001, 0.0001]]

    x_w = 0
    y_w = 0
    sum_theta = 0

    pose_est1 = transpose([[x_w, y_w, sum_theta]])
    pose_t = transpose([[x_w, y_w, sum_theta]])
    Pk1 = Pk0

    maxSpeed = 250
    maxDist = 350
    latDist = 100
    thresholdDistance = 500
    midThresholdDistance = 700

    #Obtenemos distancia inicial de cada rueda
    L_ini, R_ini = get_motors()

    leftMotorDist = maxDist
    rightMotorDist = maxDist
    try:
        init = 1
        while init:
            sensorInformation = informationToArray(envia(ser, 'GetLDSScan', 0.05))
            left = getMedianDist(sensorInformation, 20, 60)
            central = getMedianDist(sensorInformation, 340, 20)
            right = getMedianDist(sensorInformation, 300, 340)
            if  central != -1 or left != -1 or right != -1:
                init = 0
                envia(ser,'SetMotor RWheelEnable LWheelEnable', 0.2)
                str(left) + " " + str(central) + " " + str(right) + "\n"

        while 1:
            L, R = get_motors()
            # Calculamos la odometria (con incertidumbre)
            odometry(L, R)
            # Actualizamos los valores L_ini, R_ini con L y R para poder calcular el siguiente incremento
            L_ini, R_ini = L, R
            
            sensorInformation = informationToArray(envia(ser, 'GetLDSScan', 0.05))
            left = getMedianDist(sensorInformation, 20, 60)
            central = getMedianDist(sensorInformation, 340, 20)
            right = getMedianDist(sensorInformation, 300, 340)

            print str(left) + " " + str(central) + " " + str(right) + "\n"

            if central != -1: k1 = min(1, central/midThresholdDistance)
            else: k1 = 1

            k2 = left / thresholdDistance
            if not 0 <= k2 <= 1: k2 = 0

            k3 = right / thresholdDistance
            if not 0 <= k3 <= 1: k3 = 0

            if k2 == 0 and k3 == 0 and k1 != 1:
                if left > right:
                    k3 = 1 - (central / midThresholdDistance)
                else:
                    k2 = 1 - (central / midThresholdDistance)

            leftMotorDist = maxDist * k1 + latDist * k2 - latDist * k3
            rightMotorDist = maxDist * k1 - latDist * k2 + latDist * k3

            comando = 'SetMotor LWheelDist ' + str(leftMotorDist) + ' RWheelDist ' + str(rightMotorDist) + ' Speed ' + str(maxSpeed)
            envia(ser, comando, 0.05)

    except KeyboardInterrupt:
        envia(ser, 'SetLDSRotation Off', 0.2)
