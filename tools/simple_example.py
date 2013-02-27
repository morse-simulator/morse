#! /usr/bin/env python
""" Human control with keyboard through sockets

this example is meant to show how to use services with sockets.
After the connection with the socket, we just send a message well formed.
You can control the human with the numpad : 8 and 5 for forward and backward,
and 4 and 6 for left and right.

"""

import sys
import socket
import tty, termios

HOST = '127.0.0.1'
PORT = 4000

def getchar():
    """ Returns a single character from standard input """

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
    sock = None

    for res in socket.getaddrinfo(HOST, port, socket.AF_UNSPEC, socket.SOCK_STREAM):
        af, socktype, proto, canonname, sa = res
        try:
            sock = socket.socket(af, socktype, proto)
        except socket.error:
            sock = None
            continue
        try:
            sock.connect(sa)
        except socket.error:
            sock.close()
            sock = None
            continue
        break

    return sock

def main():
    sock = _connect_port(PORT)
    if not sock:
        sys.exit(1)

    print("sock connected")
    print("Please press q to quit and use 8456 to move")
    esc = 0
    _id = 0

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
            data_out = "id%d human move [%f, %f]\n" % (_id, speed, rot)
            sent = sock.send(data_out)
            print ("SENT DATA (%d bytes): %s" % (sent, data_out))
            _id = _id + 1

        if c == "q":
            esc = 1

    sock.close()
    print("\nBye bye!")

main()
