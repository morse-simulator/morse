#! /usr/bin/env python

""" Human control with wiimote

You can take control of the mocap_human in morse using a wiiMote through this code.
To do that, we use cwiid library to handle the wiiMote communication and
sockets and request mecanisms to comminicate with MORSE.
That being said, the manipulation of the wiiMote isn't really intuitive
neither precise, and so, the control is a bit hard.

"""

import time
import socket

import cwiid

from math import fabs


id_ = 0

HOST = '127.0.0.1'
PORT = 4000
s = None

toggled = False
grasped = False

tabOfExistentButtons = [cwiid.BTN_PLUS, cwiid.BTN_UP, cwiid.BTN_DOWN,
    cwiid.BTN_RIGHT, cwiid.BTN_LEFT, cwiid.BTN_HOME, cwiid.BTN_MINUS,
    cwiid.BTN_A, cwiid.BTN_B, cwiid.BTN_1, cwiid.BTN_2]


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
    """ Main function containing a loop for getting wiimote's inputs. """
    global s
    global toggled

    print ("Please, put the Wiimote on discoverable mode (press 1+2)")
    wiimote = cwiid.Wiimote()
    print ("Wiimote detected")

    s = _connect_port(PORT)
    if not s:
        sys.exit(1)

    print ("Socket connected")

    wiimote.led = cwiid.LED1_ON
    wiimote.enable(cwiid.FLAG_MESG_IFC)
    wm_cal = wiimote.get_acc_cal(cwiid.EXT_NONE)
    esc = 0

    tabOfExistentButtons.sort()
    tabOfExistentButtons.reverse()

    while not esc :
        wiimote.rpt_mode = cwiid.RPT_BTN
        time.sleep(0.05)
        wiimote.enable(cwiid.FLAG_NONBLOCK)
        msg = wiimote.get_mesg()
        wiimote.disable(cwiid.FLAG_NONBLOCK)

        if msg != None :
            if msg[0][0] == cwiid.MESG_BTN :
                button = msg[0][1]
                t = detect_button(button)
                for i in t:
                    buttonPress(i)
                buttonPressAllTab(t)

                if button == cwiid.BTN_1 + cwiid.BTN_2 :
                    esc = 1
        else :
            buttonPressAllTab(None)

        """
        # This seems to be the part where we treat the accelerometers
        # Don't want to use it for the moment
        wiimote.rpt_mode = cwiid.RPT_ACC
        msg1 = wiimote.get_mesg()
        if msg1 != None :
            if msg1[0][0] == cwiid.MESG_ACC :
                acceleration(msg1[0][1],wm_cal)
        """

    s.close()
    wiimote.led = 0
    wiimote.close()

    print ("Wiimote connection and socket connection closed succefully")
    print ("Bye bye!")


def acceleration(mesg,wm_cal):
    """ Handle the gyroscope on the wiimote """
    global id_

    acc_adjust = 0.5
    seuil = 0.05

    tilt =((mesg[cwiid.Y]*1.0 - wm_cal[0][cwiid.Y]*1.0) * acc_adjust  /
    wm_cal[0][cwiid.Y]*1.0)
    roll = (-(mesg[cwiid.X]*1.0 - wm_cal[0][cwiid.X]*1.0) * acc_adjust /
    wm_cal[0][cwiid.X]*1.0)

    if fabs(roll) < seuil :
        roll = 0.0
    if fabs(tilt) < seuil :
        seuil = 0.0

    msg = "id%s mocap_human move [%s, %s]\n" % (str(id_), str(tilt), str(roll))
    s.send(msg)
    id_ = id_ + 1


def buttonPress(button) :
    """ Handle the button pressed on the wiimote """
    global grasped
    global toggled

    headIncrement = 0.1
    handIncrement = 0.1

    """
    if button == cwiid.BTN_UP :
        move_man(0.1, 0)
        #head_move(0.0,-headIncrement)

    elif button == cwiid.BTN_DOWN :
        #head_move(0.0,headIncrement)
        move_man(-0.1, 0)

    elif button == cwiid.BTN_LEFT :
        #head_move(headIncrement,0.0)
        move_man(0, 0.1)

    elif button == cwiid.BTN_RIGHT :
        #head_move(-headIncrement,0.0)
        move_man(0, -0.1)

    # Toggling of grasp mode
    elif button == cwiid.BTN_B and not grasped :
        grasp("t")
        grasped = True
    elif button == cwiid.BTN_B and grasped :
        grasp("f")
        grasped = False

    # Toggle view to right hand
    elif cwiid.BTN_A and not toggled :
        toggle_manip()
        toggled = True
    elif cwiid.BTN_A and toggled :
        toggle_manip()
        toggled = False
    """

    """
    elif button == cwiid.BTN_MINUS :
        hand_move(-handIncrement)

    elif button == cwiid.BTN_PLUS :
        hand_move(handIncrement)
    """


def buttonPressAllTab(t) :
    """ Handle the button that can remain pressed and then released """
    global toggled
    global grasped

    move_speed = 0.05
    rotate_speed = 0.05

    if t == None :
        return

    ################ movement option ##############
    if cwiid.BTN_UP in t:
        move_man(move_speed, 0)
        #head_move(0.0,-headIncrement)

    if cwiid.BTN_DOWN in t:
        #head_move(0.0,headIncrement)
        move_man(-move_speed, 0)

    if cwiid.BTN_LEFT in t:
        #head_move(headIncrement,0.0)
        move_man(0, rotate_speed)

    if cwiid.BTN_RIGHT in t:
        #head_move(-headIncrement,0.0)
        move_man(0, -rotate_speed)


    ################ manipulation option ##############
    #if cwiid.BTN_A in t and not toggled :
    #    toggle_manip()
    #    toggled = True
    #elif cwiid.BTN_A not in t and toggled :
    #    toggle_manip()
    #    toggled = False

    ################ manipulation option ##############
    if cwiid.BTN_A in t:
        switch_cameras()

    ################ grasp option ##############
    if cwiid.BTN_B in t and not grasped :
        grasp("t")
        grasped = True
    elif cwiid.BTN_B not in t and grasped :
        grasp("f")
        grasped = False


def detect_button(button):
    """ Compute the different buttons pressed on the wiimote.
    return a tab with those buttons
    """
    t= []
    for b in tabOfExistentButtons :
        if button // b == 1 :
            t.append(b)
            button = button % b
    return t


def head_move(pan,tilt):
    """ Sending socket messages """
    global id_

    msg = "id%s mocap_human move_head [%s, %s]\n" % (str(id_), str(pan), str(tilt))
    s.send(msg)
    id_ = id_ + 1

def hand_move(diff):
    """ Sending socket messages """
    global id_

    msg = "id%s mocap_human move_hand [%s, 0.0]\n" % (str(id_), str(diff))
    # The second argument should be removed, however,
    # the socket have a probleme with a single argument.
    s.send(msg)
    id_ = id_ + 1

def move_man(v, w):
    """ Give forward movement or rotation instructions to the mocap_human robot """
    global id_

    msg = "id%s mocap_human move [%s, %s]\n" % (str(id_), str(v), str(w))
    s.send(msg)
    print("Socket move_man sent")
    id_ = id_ + 1

def toggle_manip():
    """ Sending socket messages """
    global id_

    msg = "id%s mocap_human toggle_manipulation []\n" % (str(id_))
    s.send(msg)
    id_ = id_ + 1

def switch_cameras():
    """ Sending socket messages """
    global id_

    msg = "id%s mocap_human switch_cameras []\n" % (str(id_))
    s.send(msg)
    print("Socket move camera sent")
    id_ = id_ + 1

def grasp(seq):
    """ Sending socket messages """
    global id_

    msg = "id%s mocap_human grasp_ [\"%s\"]\n" % (str(id_), str(seq))
    s.send(msg)
    print("Socket grasp sent")
    id_ = id_ + 1

main()
