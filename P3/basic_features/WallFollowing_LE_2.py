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
    envia(ser, 'SetLDSRotation On', 0.2)

    # WallFollowing params
    maxWallSpeed = 300

    constWallDist = 300

    maxWallCentralDist = 700
    maxWallDiagDist = 200
    maxWallLateralDist = 100
    thLateralDistWall = 500
    thCentralDistWall = 900
    thDiagDistWall = 700
    thRangeWall = 20

    leftMotorDist = maxWallCentralDist
    rightMotorDist = maxWallCentralDist

    try:
        init = 1
        while init:
            print "HI"
            sensorInformation = informationToArray(envia(ser, 'GetLDSScan', 0.05))
            rightDist = getMedianDist(sensorInformation,  250, 300)
            diagDist = getMedianDist(sensorInformation,  300, 340)
            centralDist = getMedianDist(sensorInformation, 340, 20)
            if  rightDist != -1 or centralDist != -1 or diagDist != -1:
                init = 0
                envia(ser,'SetMotor RWheelEnable LWheelEnable', 0.2)
        while not ((-1 < rightDist < thLateralDistWall) or (-1 < centralDist < thCentralDistWall) or  (-1 < diagDist < thDiagDistWall)):
            sensorInformation = informationToArray(envia(ser, 'GetLDSScan', 0.05))
            rightDist = getMedianDist(sensorInformation,  250, 300)
            diagDist = getMedianDist(sensorInformation,  300, 340)
            centralDist = getMedianDist(sensorInformation, 340, 20)
            print "Distancias: " + str(centralDist) + " " + str(rightDist)
            envia(ser, 'SetMotor LWheelDist ' + str(leftMotorDist) + ' RWheelDist ' + str(rightMotorDist) + ' Speed ' + str(maxWallSpeed), 0.05)

        while 1:
            sensorInformation = informationToArray(envia(ser, 'GetLDSScan', 0.05))
            rightDist = getMedianDist(sensorInformation,  250, 300)
            diagDist = getMedianDist(sensorInformation,  300, 340)
            centralDist = getMedianDist(sensorInformation, 340, 20)
            print "Distancias-> central:" + str(centralDist) + " diagonal:" +  str(diagDist) + " lateral:" + str(rightDist)


            if -1 < centralDist < thCentralDistWall: k1 = 1 - centralDist/thCentralDistWall
            else: k1 = 0

            if thLateralDistWall - thRangeWall < rightDist < thLateralDistWall + thRangeWall: k2 = 0
            elif -1 < rightDist <= thLateralDistWall - thRangeWall: k2 = - (1 - rightDist/thLateralDistWall)
            elif -1 == rightDist: k2 = 0.5
            else: k2 = (1 - rightDist/thLateralDistWall)

            if diagDist < thDiagDistWall and diagDist != -1: k3 = 0
            else: k3 = 0.7
            print "K's -> k1: " + str(k1) + " k2: " + str(k2) + " k3: " + str(k3)

            leftMotorDist =  constWallDist - maxWallCentralDist * k1 + maxWallLateralDist * k2 + maxWallDiagDist * k3
            rightMotorDist = constWallDist  + maxWallCentralDist * k1 - maxWallLateralDist * k2 - maxWallDiagDist * k3

            envia(ser, 'SetMotor LWheelDist ' + str(leftMotorDist) + ' RWheelDist ' + str(rightMotorDist) + ' Speed ' + str(maxWallSpeed), 0.05)


    except KeyboardInterrupt:
        envia(ser, 'SetLDSRotation Off', 0.2)
