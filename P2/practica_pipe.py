# To import the function "envia" from the file "test_NeatoCommands.py"

import sys
import serial
import time
from multiprocessing import Process, Queue
import http_viewer
import select
import os
import sys, tty, termios
from select import select
from math import sin, cos, fmod


###################### !!! W A R N I N G !!! ########################
port_web_server = int(sys.argv[1])
#####################################################################
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

def getch():
    global timeout
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    ch = ''
    try:
        tty.setraw(sys.stdin.fileno())
        ch = sys.stdin.read(1)
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
    print ch
    return ch

def odometry(L, R):
    global L_ini, R_ini, S, V, Pk0, speed, tita_dot, sum_theta
    global pose_est1, pose_t, Pk1, tiempo, direccion, x_w, y_w

    delta_d = (L + R) / 2
    delta_th = (L - R) / (2 * S)

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

def get_next_key(tecla):
    global tiempo
    global direccion
    global speed
    global tita_dot

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

    return tecla



if __name__ == "__main__":
    # Initialize WebServer
    laser_queue = Queue()
    pose_queue = Queue()
    viewer = http_viewer.HttpViewer(port_web_server, laser_queue, pose_queue)
    print "To open the viewer go to: http:\\\\192.168.100.1:" + str(port_web_server)
    print "To see the log run in a shell the next comnnad: 'tail -f log.txt'"
    print "Press 'Q' to stop the execution."

    # Initialize Robot
    global ser
    ser = serial.Serial(port='/dev/ttyACM0', baudrate=115200, timeout=0.05)
    envia(ser, 'TestMode On')
    envia(ser, 'PlaySound 1')
    envia(ser ,'SetMotor RWheelEnable LWheelEnable')


    #Initialize Robot parameter
    L_ini, R_ini = get_motors()

    S = 121.5		# en mm
    V = [[0.01 ** 2, 0.],[0., 0.001 ** 2]]
    Pk0 = [[0.0001, 0.0001, 0.0001],[0.0001, 0.0001, 0.0001],[0.0001, 0.0001, 0.0001]]

    speed = 0 		# en mm/s
    tita_dot = 0
    tiempo = 20
    timeout = 0.5
    direccion = 0

    x_w = 0
    y_w = 0
    sum_theta = 0

    pose_est1 = transpose([[x_w, y_w, sum_theta]])
    pose_t = transpose([[x_w, y_w, sum_theta]])
    Pk1 = Pk0

    r, w = os.pipe()
    pid = os.fork()

    if pid == 0:
        # This is the child process
        os.close(r)
        w = os.fdopen(w, 'w')
        print "Child writing"
        tecla = ""
        while tecla != "q":
            tecla = getch()
            w.write(tecla)
            print('Tecla sent')
        w.close()
        print "Child closing"
        sys.exit(0)
    else:
        os.close(w)
        r = os.fdopen(r)
        print "Parent reading"
        tecla = r.read()
        try:
            while tecla != "q":
                get_next_key(tecla)
                L, R = get_motors()
                laser_queue.put([(50, 50)])
                pose_queue.put([(x_w, y_w)])
                odometry(L, R)
                print('x_w: {} y_w: {} theta: {}'.format(x_w, y_w, sum_theta))
                time.sleep(0.1)
                tecla = r.read()
            r.close()
            envia(ser, 'TestMode Off', 0.2)
            viewer.quit()

        except KeyboardInterrupt:
            envia(ser, 'SetLDSRotation Off', 0.2)
            ser.close()
            r.close()
            viewer.quit()
