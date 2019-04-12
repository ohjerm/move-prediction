#!/usr/bin/python
import sys, struct, serial
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import datetime as dt
import rospy
from std_msgs.msg import String, Float64

def wait_for_ack():
   ddata = ""
   ack = struct.pack('B', 0xff)
   while ddata != ack:
      ddata = ser.read(1)
      print "0x%02x" % ord(ddata[0])
     
def talker():
    rospy.init_node('GSR', anonymous=False)
    pub = rospy.Publisher('GSR', Float64, queue_size=1)
    rate = rospy.Rate(30)
    while not rospy.is_shutdown():
        pub.publish(Conf)
        rate.sleep()
        break

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
   ys = []
   y = []
   y_raw = []
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

         Conf = -(max(0,min(1,(1-(GSR_ohm/movingAverageSum))*(100/6)))**2)+1
         ConfList.append(Conf)
         Conf2 = -(max(0,min(1,(1-(GSR_ohm/movingAverageSum2))*(100/6)))**2)+1
         ConfList2.append(Conf2)
      
         print "0x%02x\t\t%5d,\t%4d,\t%4d" % (packettype[0], timestamp, GSR_ohm, GSR_raw)
         
         if __name__ == '__main__':
            try:
                talker()
            except rospy.ROSInterruptException:
                pass


   except KeyboardInterrupt:
#send stop streaming command
      ser.write(struct.pack('B', 0x20))
      print
      print "stop command sent, waiting for ACK_COMMAND"
      wait_for_ack()
      print "ACK_COMMAND received."
#close serial port
      ser.close()
      print "All done"
