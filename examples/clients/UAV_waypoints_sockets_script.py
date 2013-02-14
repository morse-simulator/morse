""" Simple script for the CAT AND MOUSE game tutorial

This will command the CAT, using two semantic cameras, to follow
after the MOUSE robot """
import sys
import socket
import collections
import json
import time
import math


HOST = "localhost"

buffers = {"mouse_pose": "", "cat_pose": ""}
sockets = {"motion": None, "mouse_pose":None, "cat_pose":None }
ports = {"motion": 60003, "mouse_pose": 60001, "cat_pose": 60000 }

# the minimal distance to maintain between the mouse and the cat
minDist = 4.0
# the height for the flying cat
# NB: this is the absolute height not the one relative to the ground...
# TODO: use sensors (laser?) to take into account the ground and the obstacle
height= 3.5 


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


def _read_socket_message(id):
    """ FROM http://stackoverflow.com/questions/5829148/message-reassembly-socket-communication """
    global buffers
    got_message = False

    # reading loop
    while not got_message:
        data = sockets[id].recv(1024)
        if not data:
            break

        # add the current data read by the socket to a temporary buffer
        buffers[id] += data.decode('utf-8')

        # search complete messages
        messages = buffers[id].split('\n')

        # we need at least 2 messages to continue
        if len(messages) == 1:
            continue

        # seperator found, iterate across complete messages
        for message in messages [:-1]:
            # Prepare to exit the loop
            got_message = True

        # set the buffer with the last cutted message
        buffers[id] = messages [-1]

    decoded_message = json.loads(message)
    return (decoded_message)


def where_is(protagonist):
    """ Read data from the (mouse) pose sensor, and determine the position of the mouse """
    socket_name = "%s_pose" % protagonist
    pose = _read_socket_message(socket_name)

    return pose


def roam_around():
    """ Travels through a serie of prefixed waypoints """

    print ("Roaming around :-) " )
    waypoint = { "x": 0, "y": 0, "z": 2, "yaw": 0 }
    data_out = (json.dumps((waypoint)) + '\n').encode()
    sent = sockets['motion'].send(data_out)    
    time.sleep(5)    

    print ("Roaming around :-) " )
    waypoint = { "x": 10, "y": 10, "z": 2, "yaw": 90 }
    data_out = (json.dumps((waypoint)) + '\n').encode()
    sent = sockets['motion'].send(data_out)
    time.sleep(5)  

    print ("I'm done !" )


def frighten_mouse():
    """ Use the mouse pose sensor to locate and "chase" it """
    while True:
        mouse_pose = where_is("mouse")
        cat_pose = where_is("cat")

        if mouse_pose and cat_pose:
            # print("Cat Height : {0}", cat_pose['z'])
            # print(mouse_pose)
            # go behind the mouse
            waypoint = {    "x": mouse_pose['x'] - minDist*math.cos(mouse_pose['yaw']), \
                            "y": mouse_pose['y'] - minDist*math.sin(mouse_pose['yaw']), \
                            "z": height, \
                            "yaw": cat_pose['yaw'], \
                            "tolerance": 0.5 \
                        }
            # look at the mouse
            if mouse_pose['x']==cat_pose['x']:
                 waypoint['yaw']= math.sign(mouse_pose['y']-cat_pose['y']) * math.pi
            else:
                waypoint['yaw']= math.atan2(mouse_pose['y']-cat_pose['y'],mouse_pose['x']-cat_pose['x'])

            # send the command through the socket
            data_out = (json.dumps((waypoint)) + '\n').encode()
            sent = sockets['motion'].send(data_out)


def _usage(program_name):
    print ("Usage: {0} [server_motion_port_number] [server_mouse_pose_port_number] [server_cat_pose_port_number]\n", program_name)


def main():
    global ports
    global sockets

    # Read the arguments
    argc = len(sys.argv)
    if argc == 2:
        ports['motion'] = int(sys.argv[1])
    if argc == 3:
        ports['motion'] = int(sys.argv[1])
        ports['mouse_pose'] = int(sys.argv[2])
    if argc == 4:
        ports['motion'] = int(sys.argv[1])
        ports['mouse_pose'] = int(sys.argv[2])
        ports['cat_pose'] = int(sys.argv[3])
    elif argc > 4:
        _usage(sys.argv[0])
        sys.exit()

    sockets['motion'] = _connect_port(ports['motion'])
    sockets['mouse_pose'] = _connect_port(ports['mouse_pose'])
    sockets['cat_pose'] = _connect_port(ports['cat_pose'])

    for socket in sockets.values():
        if socket == None:
            print ("Error connecting ports. Exiting!")
            sys.exit(-1)

    """ Main behaviour """
    #roam_around()
    frighten_mouse()


if __name__ == "__main__":
    main()
