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
import json
import tty, termios

id_ = 0

HOST = '127.0.0.1'
PORT = 4000
local_socket = None

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


def _connect_port(port):
    """ Establish the connection with the given MORSE port"""
    local_socket = None

    for res in socket.getaddrinfo(HOST, port, socket.AF_UNSPEC, socket.SOCK_STREAM):
        af, socktype, proto, canonname, sa = res
        try:
            local_socket = socket.socket(af, socktype, proto)
        except socket.error as msg:
            local_socket = None
            continue
        try:
            local_socket.connect(sa)
        except socket.error as msg:
            local_socket.close()
            local_socket = None
            continue
        break

    return local_socket


def main():
    global local_socket
    global id_
    local_socket = _connect_port(PORT)
    if not socket:
        sys.exit(1)

    print("Socket connected")
    print("Please press q to quit and use 8456 to move")
    esc = 0

    while not esc:
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
        if (speed != 0 or rot != 0):
            data_out = "id%s Human move [%s, %s]\n" % (str(id_), str(speed), str(rot))
            sent = local_socket.send(data_out)
            print ("SENT DATA (%d bytes): %s" % (sent, data_out))
            id_ = id_ + 1

        if c == "q":
            esc = 1

    local_socket.close()
    print("\nBye bye!")

main()
