import sys
import socket
import collections
import json


HOST = "localhost"
server_motion_port = 60000
server_pose_port = 60001
connected = False


def _print_data(data):
    """ Choose how to print, depending on the type """
    for item in data:
        # Recursively call this function if item is a list
        if isinstance(item, list):
            _print_data(item)
        elif isinstance(item, str):
            print ("\t%s" % item)
        elif isinstance(item, collections.OrderedDict):
            print ("\t%s" % item)
        elif isinstance(item, float):
            print ("\t%.4f" % item)


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



def _usage(program_name):
    print ("Usage: {0} [server_motion_port_number] [server_pose_port_number]\n", program_name)


def main():
    global server_motion_port
    global server_pose_port
    global connected

    # Read the arguments
    argc = len(sys.argv)
    if argc == 2:
        server_motion_port = int(sys.argv[1])
    if argc == 3:
        server_motion_port = int(sys.argv[1])
        server_pose_port = int(sys.argv[2])
    elif argc > 4:
        _usage(sys.argv[0])
        sys.exit()

    pose_socket = _connect_port(server_pose_port)
    motion_socket = _connect_port(server_motion_port)

    if pose_socket and motion_socket:
        connected = True
    else:
        print ("Error connecting ports. Exiting!")
        sys.exit(-1)

    while True:
        #--------- ASK FOR OPTIONS ----------------#
        print ("Select an option:")
        print ("a) Enter speed")
        print ("b) Read coordinates")
        print ("q) Quit client program")
        op = input("Enter option: ")

        if op == 'a':
            # Ask for the new speeds
            v = input("Enter V speed: ")
            w = input("Enter W speed: ")

            # Build the required data structure, a dictionary
            v_w = {"v": float(v), "w": float(w)}
            print ("Sending the command: {0}".format(v_w))

            # Send the data
            data_out = (json.dumps((v_w)) + '\n').encode()
            print ("SENDING: %s through port %d" % (data_out, server_motion_port))
            sent = motion_socket.send(data_out)

            print ("Just sent %d bytes to server" % sent)

        elif op == 'b':
            data_in = pose_socket.recv(1024)
            try:
                # Split the long string received and keep
                #  only the last complete item
                data_string = str(data_in)
                data_string = data_string.split('\\n')[-2]
                print ("RECEIVED: %s" % data_string)
                #received_data = json.loads(data_in.decode('utf-8'))
                #_print_data(received_data)
            except EOFError as detail:
                print ("\tNo data available for the moment")
            except TypeError as detail:
                print ("\tData is empty?? '%s'" % detail)

        elif op == 'q':
            break

        else:
            print ("Unknown option. Try again.")


if __name__ == "__main__":
    main()
