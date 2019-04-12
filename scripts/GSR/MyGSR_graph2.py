#!/usr/bin/python
import sys, struct, serial
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import datetime as dt
import numpy as np
import rospy
from std_msgs.msg import String

def wait_for_ack():
   ddata = ""
   ack = struct.pack('B', 0xff)
   while ddata != ack:
      ddata = ser.read(1)
      print "0x%02x" % ord(ddata[0])

def animate(i,xs,ys):
   xs.append(dt.datetime.now().strftime('%H:%M:%S.%f'))
   ys.append(GSR_ohm)
          

   ax.clear()
   ax.plot(xs,ys)
          
   plt.xticks(rotation=45, ha='right')
   plt.subplots_adjust(bottom=0.30)
   plt.title('GSR Data')
   plt.ylabel('GSR DATA2')

if len(sys.argv) < 2:
   print "no device specified"
   print "You need to specify the serial port of the device you wish to connect to"
   print "example:"
   print "   aAccel5Hz.py Com12"
   print "or"
   print "   aAccel5Hz.py /dev/rfcomm0"
else:
   ser = serial.Serial(sys.argv[1], 115200)
   ser.flushInput()
   print "port opening, done."

# send the set sensors command
   ser.write(struct.pack('BBBB', 0x08 , 0x04, 0x01, 0x00))  #GSR and PPG
   wait_for_ack()   
   print "sensor setting, done."

# Enable the internal expansion board power
   ser.write(struct.pack('BB', 0x5E, 0x01))
   wait_for_ack()
   print "enable internal expansion board power, done."

# send the set sampling rate command

   '''
    sampling_freq = 32768 / clock_wait = X Hz
   '''
   sampling_freq = 30
   clock_wait = (2 << 14) / sampling_freq

   ser.write(struct.pack('<BH', 0x05, clock_wait))
   wait_for_ack()

# send start streaming command
   ser.write(struct.pack('B', 0x07))
   wait_for_ack()
   print "start command sending, done."

# read incoming data
   ddata = ""
   numbytes = 0
   framesize = 8 # 1byte packet type + 3byte timestamp + 2 byte GSR + 2 byte PPG(Int A13)
   
   xs = []
   x = 0
   y = []
   y_raw = []
   file = open("Test.txt","w+")
   movingAverage = [None] * 180
   Difference = 0
   DifferenceList = []
   movingAverage2 = [None] * 90
   Difference2 = 0
   DifferenceList2 = []
   DifferencePercent = 0
   DifferencePercentList = []
   DifferencePercent2 = 0
   DifferencePercentList2 = []
   Conf = 0
   ConfList = []
   Conf = 0
   ConfList2 = []
   file.write("Timestamp, GSR_ohm, MovingAverage300, Difference300, DifferencePercent300, MovingAverage150, Difference150, DifferencePercent150 \n")

   print "Packet Type\tTimestamp\tGSR\tGSR_raw"
   try:
      while True:
         while numbytes < framesize:
            ddata += ser.read(framesize)
            numbytes = len(ddata)
          
         data = ddata[0:framesize]
         ddata = ddata[framesize:]
         numbytes = len(ddata)
         results = []
         
         # read basic packet information
         (packettype) = struct.unpack('B', data[0:1])
         (timestamp0, timestamp1, timestamp2) = struct.unpack('BBB', data[1:4])

         # read packet payload
         (PPG_raw, GSR_raw) = struct.unpack('HH', data[4:framesize])
         
         # get current GSR range resistor value
         Range = ((GSR_raw >> 14) & 0xff)  # upper two bits
         if(Range == 0):
            Rf = 40.2   # kohm
         elif(Range == 1):
            Rf = 287.0  # kohm
         elif(Range == 2):
            Rf = 1000.0 # kohm
         elif(Range == 3):
            Rf = 3300.0 # kohm

         # convert GSR to kohm value
         gsr_to_volts = (GSR_raw & 0x3fff) * (3.0/4095.0)
         GSR_ohm = Rf/( (gsr_to_volts /0.5) - 1.0)
         y.append(GSR_ohm)
         y_raw.append(GSR_raw)
         x += 1
         xs.append(x)
         timestamp = timestamp0 + timestamp1*256 + timestamp2*65536
         
         number = x % 180
         movingAverage[number] = GSR_ohm
         movingAverageSum = (sum(filter(None, movingAverage)) / 180)
         Difference = (GSR_ohm - movingAverageSum)
         DifferenceList.append(Difference)

         number2 = x % 90
         movingAverage2[number2] = GSR_ohm
         movingAverageSum2 = (sum(filter(None, movingAverage2)) / 90)
         Difference2 = (GSR_ohm - movingAverageSum2)
         DifferenceList2.append(Difference2)
 
         DifferencePercent = (1 - (GSR_ohm / movingAverageSum)) * 100
         DifferencePercentList.append(DifferencePercent)

         DifferencePercent2 = (1 - (GSR_ohm / movingAverageSum2)) * 100
         DifferencePercentList2.append(DifferencePercent2)

         Conf = -(max(0,min(1,(1-(GSR_ohm/movingAverageSum))*10))**2)+1
         ConfList.append(Conf)
         Conf2 = -(max(0,min(1,(1-(GSR_ohm/movingAverageSum2))*10))**2)+1
         ConfList2.append(Conf2)

         file.write("%d, %d, %i, %f, %f, %i, %f, %f \n" % (timestamp, GSR_ohm, movingAverageSum, Difference, DifferencePercent, movingAverageSum2, Difference2, DifferencePercent2))
        
         print "0x%02x\t\t%5d,\t%4d" % (packettype[0], timestamp, GSR_ohm)
         print "I: %4d, Avg.len: %4d, Avg: %4d, Diff: %4d, Conf: %4f, Conf2: %4f" % (x, number, movingAverageSum, Difference, Conf, Conf2)
            
   except KeyboardInterrupt:
#send stop streaming command
      ser.write(struct.pack('B', 0x20))

      f = plt.figure(1)
      plt.plot(xs, y)
      plt.ylabel('GSR_ohm')
      plt.title('GSR_ohm')
      
      g = plt.figure(2)
      plt.plot(DifferenceList[180:])
      plt.ylabel('Difference 180')
      plt.title('Difference 180')

      h = plt.figure(3)
      plt.plot(DifferenceList2[90:])
      plt.ylabel('Difference 90')
      plt.title('Difference 90')

      i = plt.figure(4)
      plt.plot(DifferencePercentList[180:])
      plt.ylabel('Diff Percent 180')
      plt.title('Diff Percent 180')

      j = plt.figure(5)
      plt.plot(DifferencePercentList2[90:])
      plt.ylabel('Diff Percent 90')
      plt.title('Diff Percent 90')

      k = plt.figure(6)
      plt.plot(ConfList[180:])
      plt.ylabel('Conf 180')
      plt.title('Conf 180')

      l = plt.figure(7)
      plt.plot(ConfList2[90:])
      plt.ylabel('Conf 90')
      plt.title('Conf 90')
      plt.show()
    
      file.close()
      print
      print "stop command sent, waiting for ACK_COMMAND"
      wait_for_ack()
      print "ACK_COMMAND received."
#close serial port
      ser.close()
      print "All done"
