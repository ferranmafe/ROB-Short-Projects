#!/usr/bin/python
#coding: utf8

"""
Imports
"""
import time

import serial

"""
Imports de Teclado
"""
import os
import sys, tty, termios
from select import select
from math import sin, cos, fmod


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


def getch():
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)

    try:

        tty.setraw(sys.stdin.fileno())
        ch = sys.stdin.read(1)

    finally:

        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
    print ch
    return ch

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

# Llamada a la funcion main
if __name__ == '__main__':

    global ser
    # Open the Serial Port.
    ser = serial.Serial(port='/dev/ttyACM0', baudrate=115200, timeout=0.1)

    envia(ser,'TestMode On', 0.2)

    envia(ser,'PlaySound 1', 0.3)

    envia(ser,'SetMotor RWheelEnable LWheelEnable', 0.2)

    # Parametros Robot.
    S = 121.5		# en mm
    V = [[0.01 ** 2, 0.],[0., 0.001 ** 2]]
    Pk0 = [[0.0001, 0.0001, 0.0001],[0.0001, 0.0001, 0.0001],[0.0001, 0.0001, 0.0001]]
    distancia_L = 0	# en mm
    distancia_R = 0	# en mm

    speed = 0 		# en mm/s
    tita_dot = 0
    tiempo = 20
    direccion = 0
    
    ini = 0
    end = 0
    ini = time.time()
    
    x_w = 0
    y_w = 0
    sum_theta = 0

    pose_est1 = transpose([[x_w, y_w, sum_theta]])
    pose_t = transpose([[x_w, y_w, sum_theta]])
    Pk1 = Pk0

    print "########################"
    print "Speed = " + str(speed)
    print "Tita_dot = " + str(tita_dot)

    if direccion == 0:
        print "Direction: fordward."
    else:
        print "Direction: backward."

    print "q to exit."
    print "########################"

    # Tecla a leer.
    tecla = ''
    comando = ''

    while tecla != "q":

        end = time.time()
        R = (((speed * pow(-1, direccion) ) + (S * tita_dot)) * (end-ini)) * pow(-1, direccion)
        L = (((speed * pow(-1, direccion) ) + (-S * tita_dot)) * (end-ini)) * pow(-1, direccion)
        ini = end
        
        delta_d = (L + R)/2
        delta_th = (L - R)/(2 * S)

        x_w = x_w + (delta_d + V[0][0]) * cos(sum_theta + delta_th + V[1][1])
        y_w = y_w + (delta_d + V[0][0]) * sin(sum_theta + delta_th + V[1][1])
        sum_theta = fmod(sum_theta + delta_th + V[1][1], 2 * 3.1415)

        pose_t = transpose([[x_w, y_w, sum_theta]])

        F_x = [[1., 0., -(delta_d * sin(sum_theta + delta_th))], [0., 1., -(delta_d * cos(sum_theta+delta_th))], [0., 0., 1.]];
        F_v = [[cos(sum_theta + delta_th), -delta_d * sin(sum_theta + delta_th)], [sin(sum_theta+delta_th), delta_d*cos(sum_theta+delta_th)], [0., 1.]];

        # Pose_est1 = Pose_est1 + F_x*(Pose_t - Pose_est1) + F_v*diag(V);
        pose_est1 = matrix_sum(pose_est1, matrix_sum(matrix_mul(F_x, matrix_dif(pose_t, pose_est1)), matrix_mul(F_v, diag(V))))
        # Pk1 = F_x*Pk1*F_x' + F_v*V*F_v';
        Pk1 = matrix_sum(matrix_mul(F_x, matrix_mul(Pk1, transpose(F_x))), matrix_mul(F_v, matrix_mul(V,transpose(F_v))))
        # Leemos la tecla.
        print "Write command: "
        tecla = getch()

        if tecla == '8' or tecla == '2':

            if tecla == '8':
                speed = speed + 50
            else:
                speed = speed - 50

            if speed >= 0:
                direccion = 0
            else:
                direccion = 1

            if speed == 0:

                envia(ser,'SetMotor LWheelDisable RWheelDisable', 0.2)
                envia(ser,'SetMotor RWheelEnable LWheelEnable', 0.2)

            else:
                distancia_R = (((speed * pow(-1, direccion) ) + (S * tita_dot)) * tiempo) * pow(-1, direccion)
                distancia_L = (((speed * pow(-1, direccion) ) + (-S * tita_dot)) * tiempo) * pow(-1, direccion)

                comando = 'SetMotor LWheelDist ' + str(distancia_L) + ' RWheelDist ' + str(distancia_R) + ' Speed ' + str(speed * pow(-1, direccion))
                envia(ser,comando, 0.2)

        elif tecla == '4' or tecla == '6':

            if tecla == '4':
                tita_dot = tita_dot + (3.1415/10)
            else:
                tita_dot = tita_dot - (3.1415/10)

            distancia_R = (((speed * pow(-1, direccion) ) + (S * tita_dot)) * tiempo) * pow(-1, direccion)
            distancia_L = (((speed * pow(-1, direccion) ) + (-S * tita_dot)) * tiempo) * pow(-1, direccion)

            comando = 'SetMotor LWheelDist ' + str(distancia_L) + ' RWheelDist ' + str(distancia_R) + ' Speed ' + str(speed * pow(-1, direccion))
            envia(ser,comando, 0.2)

        elif tecla == '5':

            direccion = 0
            speed = 0
            tita_dot = 0
            distancia_L = 0
            distancia_R = 0

            envia(ser,'SetMotor LWheelDisable RWheelDisable', 0.2)
            envia(ser,'SetMotor RWheelEnable LWheelEnable', 0.2)

        if tecla == 'p':

            #print "\n########################"
            #print 'Comando enviado: ' + comando
            print "########################"
            print "########################"
            print "Speed = " + str(speed)
            print "Tita_dot = " + str(tita_dot)

            if direccion == 0:
                print "Direction: fordward."
            else:
                print "Direction: backward."
            print "########################"
            print "POSE_INTEGRATION"
            print "DespX = " + str(pose_t[0][0])
            print "DespY = " + str(pose_t[1][0])
            print "AngleTh = " + str(pose_t[2][0])
            print "########################"
