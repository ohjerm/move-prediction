#!/usr/bin/env python

import socket
import time

HOST = "169.254.178.76"     # remote host
PORT = 30002                # port used by server

s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
s.connect((HOST, PORT))

s.send("set_digital_out(1,True)" + "\n")
time.sleep(0.1)

s.send("set_digital_out(2,True)" + "\n")
time.sleep(2)

s.send("movej(p[-0.375, 0.133, 0.330, 0.46, 1.59, -3.07], a=1.0, v=0.1)" + "\n")
time.sleep(10)

s.send("set_digital_out(1,True)" + "\n")
time.sleep(0.1)

s.send("set_digital_out(2,True)" + "\n")
time.sleep(2)


data = s.recv(1024)

s.close()

print("RECEIVED:", repr(data))