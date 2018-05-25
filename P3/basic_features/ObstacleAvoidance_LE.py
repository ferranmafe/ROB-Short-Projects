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
    maxDist = 350
    latDist = 150
    thresholdDistance = 600
    midThresholdDistance = 800


    leftMotorDist = maxDist
    rightMotorDist = maxDist
    try:
        while 1:
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
