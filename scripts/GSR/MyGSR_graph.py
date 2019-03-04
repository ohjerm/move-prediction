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
   sampling_freq = 50
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
      
         print "0x%02x\t\t%5d,\t%4d,\t%4d" % (packettype[0], timestamp, GSR_ohm, GSR_raw)
         print "I: %4d" % (x)
         

   
   except KeyboardInterrupt:
#send stop streaming command
      ser.write(struct.pack('B', 0x20))
      f = plt.figure(1)
      plt.plot(xs, y)
      plt.ylabel('GSR_ohm')
      plt.title('GSR_ohm')
      
      g = plt.figure(2)
      plt.plot(xs, y_raw)
      plt.ylabel('GSR_raw')
      plt.title('GSR_raw')
      plt.show()
      print
      print "stop command sent, waiting for ACK_COMMAND"
      wait_for_ack()
      print "ACK_COMMAND received."
#close serial port
      ser.close()
      print "All done"
