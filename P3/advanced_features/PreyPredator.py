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

def getMinDist(info):
    global robotFrontDist, groupSize, numSimSens
    wallTh = 800
    
    aux = [None] * 360
    for i in range(2, 362):
        if 0 < int(info[i][1]) < 16602: aux[int(info[i][0])] = int(info[i][1])
    
    sensors = []
    for i in range(len(aux)/groupSize):
        values = []
        for j in range(groupSize):
            mod = (i + j) % 360
            if aux[mod] is not None:
                values.append(aux[mod])
        if (len(values) > 0): sensors.append(float(sum(values))/len(values))
        else: sensors.append(None)
    
    set = {}
    sensorsChecked = [False] * len(sensors)
    while not all(item for item in sensorsChecked):
        i = getNextNotCheckedIndex(sensorsChecked)
        if sensors[i] is not None:
            act = sensors[i]
            muro = True
            j = 1
            while j < numSimSens and muro:
                mod = (i + j) % len(sensors)
                if sensors[mod] is None:
                    muro = False
                elif abs(sensors[mod] - act) > wallTh:
                    muro = False
                else:
                    act = sensors[mod]
                    j += 1
                    
            if muro:
                for j in range(numSimSens):
                    set.add((i + j)% len(sensors))
        
        sensorsChecked[i] = True

    final_sensors = []
    for i in range(len(sensors)):
        if i not in set and sensors[i] is not None:
            final_sensors.append((i * groupSize, sensors[i]))

    if (len(final_sensors) > 0):
        minim = final_sensors[0]
        for i in range(len(final_sensors)):
            if minim[1] > final_sensors[i][1]:
                minim = final_sensors[i]
        return minim
        
    else: return (0, robotFrontDist)

def getNextNotCheckedIndex(array):
    for i in range(array):
        if not array[i]: return i
    return -1

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
    groupSize = 10
    numSimSens = 3

    leftMotorDist = maxDist
    rightMotorDist = maxDist

    playerType = raw_input('Please, type if you want to be "Prey" or "Predator":\n')

    while not (playerType == "Predator" or playerType == "Prey"):
        playerType = raw_input('Role not recognized. Please, type if you want to be "Prey" or "Predator":\n')

    try:
        while 1:
            data = informationToArray(envia(ser, 'GetLDSScan', 0.05))

            minDist = getMinDist(data)
            print("Min Dist: {}".format(minDist))

            if playerType == "Predator":
                if (minDist[0] <= 180):
                    k = minDist[0]/180.
                else:
                    k = -(360 - minDist[0])/180.
                rightMotorDist = maxDist * k + (minDist[1] - robotFrontDist)
                leftMotorDist = -maxDist * k + (minDist[1] - robotFrontDist)
                print("leftMotorDist: {} rightMotorDist: {}".format(rightMotorDist, leftMotorDist))
                envia(ser, 'SetMotor LWheelDist ' + str(leftMotorDist) + ' RWheelDist ' + str(rightMotorDist) + ' Speed ' + str(maxSpeed), 0.05)

            elif playerType == "Prey":
                if (minDist[0] > 180):
                    k = minDist[0] / 180.
                else:
                    k = -(360 - minDist[0]) / 180.
                rightMotorDist = maxDist * k + (minDist[1] - robotFrontDist)
                leftMotorDist = -maxDist * k + (minDist[1] - robotFrontDist)
                print("leftMotorDist: {} rightMotorDist: {}".format(rightMotorDist, leftMotorDist))
                envia(ser, 'SetMotor LWheelDist ' + str(leftMotorDist) + ' RWheelDist ' + str(rightMotorDist) + ' Speed ' + str(maxSpeed), 0.05)

    except KeyboardInterrupt:
        envia(ser, 'SetLDSRotation Off', 0.2)
