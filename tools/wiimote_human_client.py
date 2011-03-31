#! /usr/bin/env python

""" Human control with wiimote

You can take control of the human in morse using a wiiMote through this code. 
To do that, we use cwiid library to handle the wiiMot communication and 
sockets and request mecanisms to comminicate with morse.
That being said, the manipulation of the wiiMote isn't really intuitive 
neither precise, and so, the control is a bit hard.

"""

import time
import socket

import cwiid

from math import fabs


id_ = 0

HOST = '127.0.0.1'
PORT = 70000 
s = None

toggled = False
grasped = False

tabOfExistentButtons = [cwiid.BTN_PLUS, cwiid.BTN_UP, cwiid.BTN_DOWN,
    cwiid.BTN_RIGHT, cwiid.BTN_LEFT, cwiid.BTN_HOME, cwiid.BTN_MINUS,
    cwiid.BTN_A, cwiid.BTN_B, cwiid.BTN_1, cwiid.BTN_2]


def main():
    """ Main function containing a loop for getting wiimote's inputs. """
    global s
    global toggled
    
    print "Please, put the Wiimote on discoverable mode (press 1+2)"
    wiimote = cwiid.Wiimote()
    print "Wiimote detected"
    
    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    print "Socket connected"

    wiimote.led = cwiid.LED1_ON 
    wiimote.enable(cwiid.FLAG_MESG_IFC)
    wm_cal = wiimote.get_acc_cal(cwiid.EXT_NONE)
    s.connect((HOST, PORT))
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

        wiimote.rpt_mode = cwiid.RPT_ACC
        msg1 = wiimote.get_mesg()
        if msg1 != None :
            if msg1[0][0] == cwiid.MESG_ACC :
                acceleration(msg1[0][1],wm_cal)
                
    s.close()
    wiimote.led = 0
    wiimote.close()
    
    print "Wiimote connection and socket connection closed succefully"
    print "Bye bye!"
        
        
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
        
    msg = "id" + str(id_) + " Human move (" 
    msg += str(tilt) + "," 
    msg += str(roll) + ")\n"
    s.send(msg)
    id_ = id_ + 1
   
   
def buttonPress(button) :
    """ Handle the button pressed on the wiimote """
    headIncrement = 0.1
    handIncrement = 0.1

    if button == cwiid.BTN_DOWN :
        head_move(0.0,headIncrement)
        
    elif button == cwiid.BTN_LEFT :
        head_move(headIncrement,0.0)
        
    elif button == cwiid.BTN_MINUS :
        hand_move(-handIncrement)
        
    elif button == cwiid.BTN_PLUS :
        hand_move(handIncrement)
        
    elif button == cwiid.BTN_RIGHT :
        head_move(-headIncrement,0.0)
        
    elif button == cwiid.BTN_UP :
        head_move(0.0,-headIncrement)
       
def buttonPressAllTab(t) :
    """ Handle the button that can remain pressed and than released """
    global toggled
    global grasped
    
    ################ manipulation option ##############
    if t == None :
        if toggled :
            toggle_manip()
            toggled = False
    else :
        if cwiid.BTN_B in t and not toggled :
            toggle_manip()
            toggled = True
        elif cwiid.BTN_B not in t and toggled :
            toggle_manip()
            toggled = False
            
    ################ grasp option ##############
    if t == None :
        if grasped :
            grasp("f")
            grasped = False
    else :
        if cwiid.BTN_A in t and not grasped :
            grasp("t")
            grasped = True
        elif cwiid.BTN_A not in t and grasped :
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
    
    msg = "id" + str(id_) + " Human move_head (" 
    msg += str(pan) + "," 
    msg += str(tilt) + ")\n"
    s.send(msg)
    id_ = id_ + 1
    #print msg

def hand_move(diff):
    """ Sending socket messages """
    global id_
    
    msg = "id" + str(id_) + " Human move_hand (" 
    msg += str(diff) + "," 
    msg += str(0.0) + ")\n"
    # The second argument should be removed, however,
    # the socket have a probleme with a single argument.
    
    s.send(msg)
    id_ = id_ + 1

def toggle_manip():
    """ Sending socket messages """
    global id_
    
    msg = "id" + str(id_) + " Human toggle_manipulation ()\n" 
    s.send(msg)
    id_ = id_ + 1

def grasp(seq):
    """ Sending socket messages """
    global id_
    
    msg = "id" + str(id_) + " Human grasp_ ('"+ str(seq) +"')\n" 
    s.send(msg)
    id_ = id_ + 1

main()
