#! /usr/bin/env python
""" Human control with keyboard through sockets

this example is meant to show how to use services with sockets.
After the connection with the socket, we just send a message well formed.
You can control the human with the numpad : 8 and 5 for forward and backward,
and 4 and 6 for left and right.

"""

import sys
import time
import socket
import tty, termios

id_ = 0

HOST = '127.0.0.1'
PORT = 4000
s = None

def getchar():
   #Returns a single character from standard input

   fd = sys.stdin.fileno()
   old_settings = termios.tcgetattr(fd)
   try:
      tty.setraw(sys.stdin.fileno())
      ch = sys.stdin.read(1)
   finally:
      termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
   return ch

def main():
    global s
    global id_
    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    s.connect((HOST, PORT))
    print "Socket connected"
    print "Please press q to quit and use 8456 to move"
    esc = 0

    while not esc :
        c = getchar()
        speed = 0
        rot = 0
        if (c == "8"):
            speed = 0.1
        elif (c == "5"):
            speed = -0.1
        elif (c == "4"):
            rot = 0.1
        elif (c == "6"):
            rot = -0.1
        if (speed != 0 or rot != 0) :
            msg = "id" + str(id_) + " Human move ("
            msg += str(speed) + ","
            msg += str(rot) + ")\n"
            s.send(msg)
            id_ = id_ + 1

        if c == "q" :
            esc = 1

    s.close()
    print "\nBye bye!"

main()

