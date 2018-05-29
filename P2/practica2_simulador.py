import sys
import serial
import time
from multiprocessing import Process, Queue
import http_viewer

import os
import sys, tty, termios
from math import sin, cos, fmod, atan2, sqrt, pow

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
    '''
    global speed, direccion, tita_dot, time_ini
    time_end = time.time()
    time_inc = time_end - time_ini
    L = (((speed * pow(-1, direccion) ) + (-S * tita_dot)) * time_inc) * pow(-1, direccion)
    R = (((speed * pow(-1, direccion) ) + (S * tita_dot)) * time_inc) * pow(-1, direccion)
    time_ini = time_end
    time.sleep(0.05)
    return (L, R)
    '''

def get_laser():

    msg = envia(ser, 'GetLDSScan', 0.05);

    '''
    msg = "0,0,0,0"
    #for i in range(45, 136):
    for i in range(85, 96):
        msg += "\r\n" + str(i) + "," + str(500/sin(i * 3.1415/180)) + ",0,0"
    #for i in range(225, 316):
    for i in range(265, 276):
        msg += "\r\n" + str(i) + "," + str(-500/sin(i * 3.1415/180)) + ",0,0"
    '''
    laser_values = []
    for line in msg.split('\r\n')[2:362]:
        s = line.split(',')
        row = [s[0], s[1], s[2], s[3]]
        laser_values.append(row)
    return laser_values

def get_laser_data():
    global x_w, y_w, sum_theta
    laser_values = get_laser()
    laser_points = []
    for i in range(len(laser_values)):
        angle_laser = int(laser_values[i][0]) * 3.1415 / 180
        dist_laser = int(float(laser_values[i][1]))
        if 0 < dist_laser < 16627:
            x = x_w + dist_laser * cos(angle_laser - sum_theta)
            y = y_w + dist_laser * sin(angle_laser - sum_theta)
            laser_points.append((x, -y))
    return laser_points

def iniciarVueltaBase():
    global direccion, speed, tita_dot, x_base, y_base
    # Paramos el NEATO y pedimos las coordenadas de la base de carga
    direccion = 0
    speed = 0
    tita_dot = 0
    distancia_L = 0
    distancia_R = 0

    envia(ser,'SetMotor LWheelDisable RWheelDisable', 0.05)
    envia(ser,'SetMotor RWheelEnable LWheelEnable', 0.05)

    print("Introduce las coordenadas de la base de carga")
    x_base = input("Coordenada x (mm):")
    y_base = input("Coordenada y (mm):")

def vueltaBaseCarga(estado):
    #ESTADO: 0 -> INICIO, 1 -> ORIENTACION, 2 -> ACERCAMIENTO, 3 -> BUSQUEDACONTROL, 4 -> ORIENTACION2, 5 -> ACERCAMIENTO2, 6 -> FINAL
    if estado == 0:
        iniciarVueltaBase()
        estado = 1
    elif estado == 1:
        estado = orientationToBase()
    elif estado == 2:
        estado = aproachToBase()
    elif estado == 3:
        estado = 6
    elif estado == 4:
        estado = 6
    elif estado == 5:
        estado = 6
    return estado

def orientationToBase():
    global x_base, y_base, x_w, y_w, sum_theta, speed, direccion, tita_dot, threshold_ang
    rot_ang = atan2((y_base - y_w), (x_base - x_w))
    if (rot_ang < 0):
        rot_ang = 2 * 3.1415 + rot_ang
    rot_ang = (rot_ang - sum_theta) % (2 * 3.1415)
    print "Rot_angle: " + str(rot_ang * 180/3.1415)
    if rot_ang > threshold_ang and rot_ang < (-threshold_ang % (2 * 3.1415)):
        speed = 50
        direccion = 0
        if (rot_ang < 3.1415):
            tita_dot = (3.1415/10)
        else:
            tita_dot = -(3.1415/10)
        print ('Speed: {} Tita_dot: {}'.format(speed, tita_dot))
        distancia_R = (((speed * pow(-1, direccion) ) + (S * tita_dot)) * tiempo) * pow(-1, direccion)
        distancia_L = (((speed * pow(-1, direccion) ) + (-S * tita_dot)) * tiempo) * pow(-1, direccion)
        comando = 'SetMotor LWheelDist ' + str(distancia_L) + ' RWheelDist ' + str(distancia_R) + ' Speed ' + str(speed * pow(-1, direccion))
        envia(ser,comando, 0.05)

        return 1 # Seguimos en estado ORIENTACION
    else:
        speed = 0
        direccion = 0
        tita_dot = 0

        distancia_L = 0
        distancia_R = 0
        envia(ser,'SetMotor LWheelDisable RWheelDisable', 0.05)
        envia(ser,'SetMotor RWheelEnable LWheelEnable', 0.05)

        return 2 # Pasamos a estad ACERCAMIENTO

def aproachToBase():
    global x_base, y_base, x_w, y_w, sum_theta, speed, direccion, tita_dot, threshold_dist
    dist = sqrt(pow((x_base-x_w), 2) + pow((y_base-y_w), 2))
    print('x_w: {} y_w: {} theta: {}'.format(x_w, y_w, sum_theta))
    print "Dist: " + str(dist)
    if dist > threshold_dist:
        speed = 50
        direccion = 0
        tita_dot = 0

        distancia_R = (((speed * pow(-1, direccion) ) + (S * tita_dot)) * tiempo) * pow(-1, direccion)
        distancia_L = (((speed * pow(-1, direccion) ) + (-S * tita_dot)) * tiempo) * pow(-1, direccion)
        comando = 'SetMotor LWheelDist ' + str(distancia_L) + ' RWheelDist ' + str(distancia_R) + ' Speed ' + str(speed * pow(-1, direccion))
        envia(ser,comando, 0.05)

        return 2 # Seguimos en estado ACERCAMIENTO
    else:
        speed = 0
        direccion = 0
        tita_dot = 0

        distancia_L = 0
        distancia_R = 0
        envia(ser,'SetMotor LWheelDisable RWheelDisable', 0.05)
        envia(ser,'SetMotor RWheelEnable LWheelEnable', 0.05)

        return 6 # Pasamos a estado BUSQUEDACONTROL

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

def getch():
    """
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    try:
        tty.setraw(sys.stdin.fileno())
        ch = sys.stdin.read(1)
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
    print ch
    return ch
    """
    sys.stdin = open('/dev/tty')
    ch = raw_input("Introduce next command:\n")
    return ch

def get_next_key():
    tecla = getch()
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

    elif tecla == '4' or tecla == '6':
        if tecla == '4':
            tita_dot = tita_dot + (3.1415/10)
        else:
            tita_dot = tita_dot - (3.1415/10)

    elif tecla == '5':
        direccion = 0
        speed = 0
        tita_dot = 0
    return tecla

def set_next_key(tecla):
    global tiempo
    global direccion
    global speed
    global tita_dot
    global x_w, y_w, sum_theta

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

            envia(ser,'SetMotor LWheelDisable RWheelDisable', 0.1)
            envia(ser,'SetMotor RWheelEnable LWheelEnable', 0.1)

        else:
            distancia_R = (((speed * pow(-1, direccion) ) + (S * tita_dot)) * tiempo) * pow(-1, direccion)
            distancia_L = (((speed * pow(-1, direccion) ) + (-S * tita_dot)) * tiempo) * pow(-1, direccion)

            comando = 'SetMotor LWheelDist ' + str(distancia_L) + ' RWheelDist ' + str(distancia_R) + ' Speed ' + str(speed * pow(-1, direccion))
            envia(ser,comando, 0.1)


    elif tecla == '4' or tecla == '6':
        if tecla == '4':
            tita_dot = tita_dot + (3.1415/10)
        else:
            tita_dot = tita_dot - (3.1415/10)


        distancia_R = (((speed * pow(-1, direccion) ) + (S * tita_dot)) * tiempo) * pow(-1, direccion)
        distancia_L = (((speed * pow(-1, direccion) ) + (-S * tita_dot)) * tiempo) * pow(-1, direccion)

        comando = 'SetMotor LWheelDist ' + str(distancia_L) + ' RWheelDist ' + str(distancia_R) + ' Speed ' + str(speed * pow(-1, direccion))
        envia(ser,comando, 0.1)

    elif tecla == '5':

        direccion = 0
        speed = 0
        tita_dot = 0

        envia(ser,'SetMotor LWheelDisable RWheelDisable', 0.1)
        envia(ser,'SetMotor RWheelEnable LWheelEnable', 0.1)
    elif tecla == "p":

        print('x_w: {} y_w: {} theta: {}'.format(x_w, y_w, sum_theta))
        print('speed: {} tita_dot: {} direccion: {}'.format(speed, tita_dot, direccion))


def f(q):
    while True:
        r = getch()
        q.put(r)

if __name__ == "__main__":
    # Initialize WebServer
    laser_queue = Queue()
    pose_queue = Queue()
    viewer = http_viewer.HttpViewer(port_web_server, laser_queue, pose_queue)
    print "To open the viewer go to: http:\\\\192.168.100.1:" + str(port_web_server)
    print "To see the log run in a shell the next command: 'tail -f log.txt'"
    print "Press 'Q' to stop the execution."

    # Initialize Robot

    global ser
    ser = serial.Serial(port='/dev/ttyACM0', baudrate=115200, timeout=0.05)
    envia(ser, 'TestMode On')
    envia(ser, 'PlaySound 1')
    envia(ser ,'SetMotor RWheelEnable LWheelEnable')
    envia(ser, 'SetLDSRotation On')


    #Initialize Robot parameters
    time_ini = time.time()
    speed = 0 		# en mm/s
    tita_dot = 0
    tiempo = 20
    direccion = 0


    S = 121.5 # en mm
    V = [[0.01 ** 2, 0.],[0., 0.001 ** 2]]
    Pk0 = [[0.0001, 0.0001, 0.0001],[0.0001, 0.0001, 0.0001],[0.0001, 0.0001, 0.0001]]

    x_w = 0
    y_w = 0
    sum_theta = 0

    pose_est1 = transpose([[x_w, y_w, sum_theta]])
    pose_t = transpose([[x_w, y_w, sum_theta]])
    Pk1 = Pk0

    # Because input treatment
    q = Queue()
    p = Process(target=f, args=(q,))
    p.start()
    #

    #Obtenemos distancia inicial de cada rueda
    L_ini, R_ini = get_motors()

    try:
        tecla = ""
        while tecla != "q" and tecla != "b":
            # Obtenemos distancia recorrida por cada rueda en esa iteracion (la odometria hara la resta con la anterior para saber el incremento)
            L, R = get_motors()
            # Calculamos la odometria (con incertidumbre)
            odometry(L, R)
            # Actualizamos los valores L_ini, R_ini con L y R para poder calcular el siguiente incremento

            L_ini, R_ini = L, R

            # Enviamos al servidor la nueva posicion del robot (calculada por la odometria)
            # (Modificado http_viewer -> JSON lee (x, y), antes leia (y, x)
            pose_queue.put([(x_w, -y_w)])
            # Enviamos al servidor los puntos detectados por el laser en esta iteracion
            laser_queue.put(get_laser_data())
            # Espera de 0.1s para que el servidor reciva los datos
            time.sleep(0.1)
            # Hacemos una lectura sin espera en busca de nuevos comandos (teclas)
            if not q.empty():
                tecla = q.get()
                set_next_key(tecla)
            else:
                tecla = ""

        p.terminate()
        p.join()
        # Comando vuelta a base de carga detectado
        if tecla == "b":
            # Definimos conjunto estados vuelta a base:
            #0 -> INICIO, 1 -> ORIENTACION, 2 -> ACERCAMIENTO, 3 -> BUSQUEDACONTROL, 4 -> ORIENTACION2, 5 -> ACERCAMIENTO2, 6 -> FINAL
            estado_actual = 0 # INICIO
            x_base = 0
            y_base = 0
            threshold_ang = 5 * 3.1415 / 180 # 5 grados
            threshold_dist = 500
            while estado_actual != 6: # Mientras no sea el estado final
                # Computamos accion NEATO segun estado definido
                estado_actual = vueltaBaseCarga(estado_actual)
                # Actualizamos odometria a cada iteracion
                L, R = get_motors()
                odometry(L, R)

                L_ini, R_ini = L, R

                pose_queue.put([(x_w, -y_w)])
                laser_queue.put(get_laser_data())
                time.sleep(0.1)

        envia(ser, 'TestMode Off', 0.1)
        envia(ser, 'SetLDSRotation Off', 0.1)
        ser.close()

        viewer.quit()
    except KeyboardInterrupt:

        envia(ser, 'TestMode Off', 0.1)
        envia(ser, 'SetLDSRotation Off', 0.1)
        ser.close()

        viewer.quit()
