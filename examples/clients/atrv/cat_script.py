""" Simple script for the CAT AND MOUSE game tutorial

This will command the CAT, using two semantic cameras, to follow
after the MOUSE robot """
import sys
import socket
import collections
import json


HOST = "localhost"

buffers = {"semantic_L": "", "semantic_R": ""}
sockets = {"semantic_L": None, "semantic_R": None, "motion": None}
ports = {"semantic_L": 60002, "semantic_R": 60001, "motion": 60000}


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


def is_mouse_visible(side):
    """ Read data from the semantic camera, and determine if a specific
    object is within the field of view of the robot """
    socket_name = "semantic_%s" % side
    semantic_data = _read_socket_message(socket_name)
    if semantic_data:
        for item in semantic_data:
            if item['name'] == "MOUSE":
                return True
    return False


def chase_mouse():
    """ Use the semantic cameras to locate the target and follow it """
    mouse_seen_left = False
    mouse_seen_right = False

    while True:
        mouse_seen_left = is_mouse_visible("L")
        mouse_seen_right = is_mouse_visible("R")
        #print ("SEE: L=%s R=%s" % (mouse_seen_left, mouse_seen_right))
        if mouse_seen_left and mouse_seen_right:
            v_w = {"v": 2, "w": 0}
        elif mouse_seen_left:
            v_w = {"v": 1.5, "w": 1}
        elif mouse_seen_right:
            v_w = {"v": 1.5, "w": -1}
        else:
            v_w = {"v": 0, "w": -1}

        data_out = (json.dumps((v_w)) + '\n').encode()
        sent = sockets['motion'].send(data_out)


def _usage(program_name):
    print ("Usage: {0} [server_motion_port_number] [server_semantic_left_port_number] [server_semantic_right_port_number]\n", program_name)


def main():
    global ports
    global sockets

    # Read the arguments
    argc = len(sys.argv)
    if argc == 2:
        ports['motion'] = int(sys.argv[1])
    if argc == 3:
        ports['motion'] = int(sys.argv[1])
        ports['semantic_L'] = int(sys.argv[2])
    if argc == 4:
        ports['motion'] = int(sys.argv[1])
        ports['semantic_L'] = int(sys.argv[2])
        ports['semantic_R'] = int(sys.argv[3])
    elif argc > 5:
        _usage(sys.argv[0])
        sys.exit()

    sockets['motion'] = _connect_port(ports['motion'])
    sockets['semantic_L'] = _connect_port(ports['semantic_L'])
    sockets['semantic_R'] = _connect_port(ports['semantic_R'])

    for socket in sockets.values():
        if socket == None:
            print ("Error connecting ports. Exiting!")
            sys.exit(-1)

    chase_mouse()


if __name__ == "__main__":
    main()
