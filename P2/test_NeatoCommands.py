#!/usr/bin/python
#coding: utf8

import serial
import time
import commands


"""
Function to send commands to Neato robot.
Parametros: 
            missatge: Command to send
            temps: delay to recive data
"""
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

def test(prueba):

	if prueba == 1:
		"""
		Static test, try other commnads (No commands to move the robot)
		"""
		print envia(ser,'GetMotors LeftWheel RightWheel')

	elif prueba == 2: 
		"""
		Test step by step.
			1 - Get the initial conditions of the robot position.
			2 - Move the robot fordward.
			3 - Get the final conditions of the robot position.
		"""
		print envia(ser, 'GetMotors LeftWheel RightWheel')

		dist = 800  # en mm
		speed = 50	# en mm/s
		envia(ser, 'SetMotor LWheelDist '+ str(dist) +' RWheelDist ' + str(dist) + ' Speed ' + str(speed))

		print envia(ser, 'GetMotors LeftWheel RightWheel')

	elif prueba == 3:
		"""
		Geting Odometry while moving.
		For easy check move fordward 1m at 0.05m/s and ask for odometry every seconds
		"""
		print envia(ser, 'GetMotors LeftWheel RightWheel')

		dist = 800  # en mm
		speed = 100	# en mm/s
		envia(ser, 'SetMotor LWheelDist '+ str(dist) +' RWheelDist ' + str(dist) + ' Speed ' + str(speed))

		i = 0;

		while i <= 8:

			fecha_actual = commands.getoutput ('date')
			print fecha_actual
			print envia(ser, 'GetMotors LeftWheel RightWheel')
			i = i + 1

	elif prueba == 4:
		"""
		To parse the data recive from command GetMotors to create Odometry variable (incR + incL / 2; incR + incL / 2S).
		"""
		msg = envia(ser, 'GetMotors LeftWheel RightWheel')


"""
		Will run only when calling from shell. Will no run when importing it
"""
if __name__ == "__main__":
	# Open the Serial Port.
	global ser
	ser = serial.Serial(port='/dev/ttyACM0', baudrate=115200, timeout=0.05)
	envia(ser, 'TestMode On')

	envia(ser, 'PlaySound 1')

	envia(ser ,'SetMotor RWheelEnable LWheelEnable')

	test(3)
	time.sleep(32)
	envia(ser, 'TestMode Off', 0.2)

	# Close the Serial Port.
	ser.close()      

	print "Final"