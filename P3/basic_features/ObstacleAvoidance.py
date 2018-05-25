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

    envia(ser,'SetMotor RWheelEnable LWheelEnable', 0.2)
    envia(ser, 'SetLDSRotation On', 0.4)

    maxSpeed = 250
    maxDist = 500
    thresholdDistance = 600
    midThresholdDistance = 1000


    leftMotorDist = maxDist
    rightMotorDist = maxDist
    try:
        while 1:
            sensorInformation = informationToArray(envia(ser, 'GetLDSScan', 0.05))
            leftDist = getMedianDist(sensorInformation, 20, 60)
            centralDist = getMedianDist(sensorInformation, 340, 20)
            rightDist = getMedianDist(sensorInformation, 300, 340)

            print str(leftDist) + " " + str(centralDist) + " " + str(rightDist) + "\n"

            leftPercentatge = 1
            rightPercentatge = 1

            if (0 < leftDist < thresholdDistance):
                leftPercentatge = (thresholdDistance - leftDist) / thresholdDistance

            if (0 < rightDist < thresholdDistance or 0 < centralDist < midThresholdDistance):
                rightPercentatge = max((thresholdDistance - rightDist)/ thresholdDistance, (midThresholdDistance - centralDist)/ midThresholdDistance)

            rightMotorDist = maxDist * leftPercentatge
            leftMotorDist = maxDist * rightPercentatge
            comando = 'SetMotor LWheelDist ' + str(leftMotorDist) + ' RWheelDist ' + str(rightMotorDist) + ' Speed ' + str(maxSpeed)
            envia(ser, comando, 0.05)

    except KeyboardInterrupt:
        envia(ser, 'SetLDSRotation Off', 0.2)
