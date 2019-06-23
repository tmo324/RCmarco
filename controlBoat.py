# -*- coding: utf-8 -*-
#+---------------------------------------------Imports------------------------------------------------+
#####REMINDER: GPIO MODE BCM
import os
from gps import *
from time import *
import time
import math
from math import *
import pigpio
import threading
import RPi.GPIO as GPIO
from ina219 import INA219, DeviceRangeError
#import dht11 

GPIO.setwarnings(False)
GPIO.setmode(GPIO.BCM)
gpsd = None #seting the global variable

#SHUNT_OHMS = 0.1
#MAX_EXPECTED_AMPS = 2.0
#ina = INA219(SHUNT_OHMS, MAX_EXPECTED_AMPS)
#ina.configure(ina.RANGE_16V)

##############Serial####################
import serial
port = serial.Serial("/dev/ttyAMA0", baudrate=57600, timeout=0.5)

###############Variables################
count=1
pointing=0
shouldPoint=0
whatToDo=0
distance=0
goingToLocation=0
latitude=0
longitude=0
utctime=0
speedGPS=0

###############ServoMethods################
def setServo( servoNum, pulses):
        pi.set_servo_pulsewidth(servoNum, pulses)
def setServoAngle ( servoNum, position ):
#      print ("Servo {} {} micro pulses".format(s, pw))
    pw = position * 500 / 90 + 1500    
    pi.set_servo_pulsewidth(servoNum, pw)
pi=pigpio.pi()
#+---------------------------------------------AllPins------------------------------------------------$
#BCMMODE GPIOMODE
thrusterPin=18
servoPin=23

#Thruster
setServo(thrusterPin, 1500) #pin 12 on board (GPIO18 PCM_CLK)
#14Board for Ground
#Servo
setServoAngle(servoPin, -15) #0degrees
#Antenna
#2, 6, 8(TX), and 10(RX)(All Board) 
#DHT11
#instance = dht11.DHT11(pin=24)
#result = instance.read()
#################################################################
def updateGpsInfo():
      global latitude
      latitude=gpsd.fix.latitude
      global longitude
      longitude=gpsd.fix.longitude
      global utctime
      utctime=gpsd.utc
      global speedGPS
      speedGPS=gpsd.fix.speed
      global pointing
      pointing=gpsd.fix.track

def displayTable():
      port.write("\n\r+--------------------------+")
      port.write("\n\r| Latitude |")
      port.write(str(latitude))
      port.write("\n\r| Longitude|")
      port.write(str(longitude))
      port.write("\n\r| UTC Time |")
      port.write( str(utctime))
      port.write("\n\r| Speed m/s|")
      port.write(str(speedGPS))
      port.write("\n\r| Bearing: |")
      port.write(str(pointing))
      port.write("\n\r| ShouldPt:|")
      port.write( str(shouldPoint))
      port.write("\n\r+---------------------------+")
      port.write("\n\r| WhatToDo:|")
      port.write(str(whatToDo))
      port.write("\n\r| Distance:|")
      port.write(str(distance))
      port.write("\n\r| Going To:|")
      port.write(str(goingToLocation))
      #port.write("\n\r+------Meter Data-----------+")
      #port.write("\n\r| Bus Voltage:|")
      #port.write(str('{0:0.2f}V'.format(ina.voltage())))

      #port.write("\n\r| Bus Current:|")
      #port.write(str('{0:0.2f}mA'.format(ina.current())))

      #port.write("\n\r| Power      :|")
      #port.write(str('{0:0.2f}mW'.format(ina.power())))

      #port.write("\n\r|ShuntVoltage:|")
      #port.write(str('{0:0.2f}mV'.format(ina.shunt_voltage())))

      #port.write("\n\r+----------ESC-Info------------+")
      #port.write("\n\r| ESC Temp :|")
      #port.write(str(result.temperature))
      #port.write("\n\r| ESC Humid:|")
      #port.write(str(result.humidity))
      port.write("\n\r+--------------------------+")
#+---------------------------------------------Navigation---------------------------------------------$
def forward():
        setServo(thrusterPin, 1600)

def speed(x):
        setServo(thrusterPin, x)

def stopTheEngine():
        setServo(thrusterPin, 1500)

def backward():
	setServo(thrusterPin, 1400)

def backToZero():
        setServoAngle(servoPin, -15)

def left():
        setServoAngle(servoPin, 90)

def right():
        setServoAngle(servoPin, -120)


def stopEverything():
        stopTheEngine()
        backToZero()


#@@@@@@@@@@@@@@@@@@@@@@@@MAIN@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@
#@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@
if __name__ == '__main__':
  #gpsp = GpsPoller() # create the thread
  try:
#    gpsp.start() # start it up
    while True:
 #     updateGpsInfo()
      rcv = port.read(10)
      print rcv

      if rcv==str('w'):
             forward()
             port.write("\r\nTurning W↑↑↑")
      elif rcv==str('s'):
             backward()
             port.write("\r\nTurning S↓↓↓")
      elif rcv==str('a'):
             left()
             port.write("\r\nTurning A ←←←")
      elif rcv==str('d'):
             right()
             port.write("\r\nTurning D→→→")
      elif rcv==str('o'):
             backToZero()
             port.write("\r\nTurning O")
      elif rcv==str('p'):
             port.write("\r\nStopping")
             stopEverything()
      elif rcv==str('t'):
             displayInfo()

      elif rcv==str('1'):
             speed(1550)
      elif rcv==str('2'):
             speed(1600)
      elif rcv==str('3'):
             speed(1650)
      elif rcv==str('4'):
             speed(1700)
      elif rcv==str('5'):
             speed(1750)
      elif rcv==str('6'):
             speed(1800)
      elif rcv==str('7'):
             speed(1850)
      elif rcv==str('8'):
             speed(1900)

  except KeyboardInterrupt: #when you press ctrl+c
    print "\nKilling Thread..."
    #gpsp.running = False
    #gpsp.join() # wait for the thread to finish what it's doing
  print "Done.\nExiting."

GPIO.cleanup()




