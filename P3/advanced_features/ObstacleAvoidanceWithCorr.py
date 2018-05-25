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
    
    if len(aux) > 0: return aux[int(len(aux) / 2)]
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

    maxSpeed = 150
    maxDist = 500
    thresholdDistance = 600
    midThresholdDistance = 1000
    corrThreshold = 50

    leftMotorDist = maxDist
    rightMotorDist = maxDist
    
    
    leftIni = float(envia(ser,'GetMotor LeftWheel', 0.05).split('\n')[4].split(',')[1])
    rightIni = float(envia(ser,'GetMotor RightWheel', 0.05).split('\n')[4].split(',')[1])
    leftCorr = 0
    rightCorr = 0
    try:
        while 1:
            leftPercentatge = 1
            rightPercentatge = 1
            
            # En realitat hauriem de treballar amb equacions:
            # LM = K1 * Center + K2 * Left - K3 * Right + K4 * incM 
            # LM = K1 * Center - K2 * Left + K3 * Right - K4 * incM 
            sensorInformation = informationToArray(envia(ser, 'GetLDSScan', 0.05))

            leftDist = getMedianDist(sensorInformation, 20, 60)
            centralDist = getMedianDist(sensorInformation, 340, 20)
            rightDist = getMedianDist(sensorInformation, 300, 340)
            
            leftCorr -= float(envia(ser,'GetMotor LeftWheel', 0.05).split('\n')[4].split(',')[1])
            rightCorr += float(envia(ser,'GetMotor RightWheel', 0.05).split('\n')[4].split(',')[1])
            if (0 < leftDist < thresholdDistance or 0 < rightDist < thresholdDistance or 0 < centralDist < midThresholdDistance):
                if (0 < leftDist < thresholdDistance):
                    leftPercentatge = (thresholdDistance - leftDist) / thresholdDistance
                    
                if (0 < rightDist < thresholdDistance or 0 < centralDist < midThresholdDistance):
                    rightPercentatge = max((thresholdDistance - rightDist)/ thresholdDistance, (midThresholdDistance - centralDist)/ midThresholdDistance) 
            elif abs(leftCorr - rightCorr) > corrThreshold:
                if (leftCorr > rightCorr):
                    leftPercentatge = 0.5
                else:
                    rightPercentatge = 0.5
            rightMotorDist = maxDist * leftPercentatge
            leftMotorDist = maxDist * rightPercentatge
            envia(ser, 'SetMotor LWheelDist ' + str(leftMotorDist) + ' RWheelDist ' + str(rightMotorDist) + ' Speed ' + str(maxSpeed), 0.05)
            
            
    except KeyboardInterrupt:
        envia(ser, 'SetLDSRotation Off', 0.2)
