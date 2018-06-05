#!/usr/bin/python
#coding: utf8

import time
import serial
import math
import commands

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

def getMinDist(info):
    global numConsecSensors, maxNumSensorsWithoutData
    aux = []
    for i in range(2, 362):
        if 0 < int(info[i][1]) < 16627: aux.append((int(info[i][0]), (int(info[i][1]))))
    
    wall = set([])
    for i in range(len(aux)):
        holesDetected = 0
        for j in range(numberConsecSensors):
            mod = (i + j) % len(aux)
            if mod = 0:
                if aux[mod][0] != aux[len(aux) - 1][0] + 1: holesDetected += aux[mod][0] - aux[mod - 1][0]
            else:
                if aux[mod][0] != aux[mod - 1][0] + 1: holesDetected += aux[mod][0] - aux[mod - 1][0]
        
        if holesDetected < maxNumSensorsWithoutData:
            for j in range(numberConsecSensors):
                mod = (i + j) % len(aux)
                wall.add(aux[mod])
            

    final_aux = [x for x in aux if x not in wall]
    if len(final_aux) > 0:
        minim = final_aux[0]
        for i in range(1, len(final_aux)):
            if minim[1] > final_aux[i][1]: minim = final_aux[i]

        return minim
    else: return (0, robotFrontDist)

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

    envia(ser,'SetMotor RWheelEnable LWheelEnable', 0.2)
    envia(ser, 'SetLDSRotation On', 0.3)

    maxSpeed = 200
    maxDist = 400

    robotFrontDist = 245
    numConsecSensors = 40
    maxNumSensorsWithoutData = 5

    leftMotorDist = maxDist
    rightMotorDist = maxDist

    try:
        while 1:
            data = informationToArray(envia(ser, 'GetLDSScan', 0.05))

            minDist = getMinDist(data)
            print("Min Dist: {}".format(minDist))
            """
            if (minDist[0] <= 180):
                k = minDist[0]/180.
            else:
                k = -(360 - minDist[0])/180.
            rightMotorDist = maxDist * k + (minDist[1] - robotFrontDist)
            leftMotorDist = -maxDist * k + (minDist[1] - robotFrontDist)
            print("leftMotorDist: {} rightMotorDist: {}".format(rightMotorDist, leftMotorDist))
            envia(ser, 'SetMotor LWheelDist ' + str(leftMotorDist) + ' RWheelDist ' + str(rightMotorDist) + ' Speed ' + str(maxSpeed), 0.05)
            """

    except KeyboardInterrupt:
        envia(ser, 'SetLDSRotation Off', 0.2)
