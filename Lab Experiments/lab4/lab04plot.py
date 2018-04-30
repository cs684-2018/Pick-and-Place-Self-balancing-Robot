# -*- coding: utf-8 -*-
"""
Created on Wed Jan 31 12:28:30 2018


Team members
1. Jagat Pati Singh(173074014)	2. Kamlesh Kumar Sahu(173074010)	3. Dhananjay kr. sharma(173050046) 

"""

import serial

import numpy
import matplotlib.pyplot as plt
from drawnow import *

leftrightA=[]
updownA=[]
count = 0
plt.figure()  
ser = serial.Serial('COM11', 9600, timeout=2, xonxoff=False, rtscts=False, dsrdtr=False)
ser.flushInput()
ser.flushOutput()
plt.ion()

#
#def dataPlot():
#	plt.title("X-Y Co-ordinate data from Joystick")
#	plt.grid(True)
#	plt.ylabel("ASCII Values")
#	plt.plot(leftrightA, 'ro-', label="ASCII")
#	plt.legend(loc='upper left')
#	plt2=plt.twinx()
#	plt2.plot(updownA)



while True:
    while (ser.inWaiting()==0):
        pass
 
    dataStream = ser.readline().decode("utf-8")
  
    dataArray = dataStream.split(',')
    
    leftright = float ( dataArray[0] )
   
    updown = float( dataArray[1] )
   
    print(leftright, "  ",updown)
    
    plt.gcf().clear()
    plt.plot(leftright,updown,'bo')
    plt.axis([-10, 4100, -10, 4100])
    plt.draw()
    
    plt.pause(0.05)

#    count = count +1;
