import sys
import socket
import collections
if sys.version_info >= (3,0,0):
    import pickle
else:
    import cPickle as pickle


server_ip = "localhost"
server_port = 60000
connected = False

def read_data(client_socket):
    """ Read the input socket until no more data is available """
    finished = False
    data_in = ''

    if connected:
        # Receiving
        while not finished:
            try:
                data_in, SRIP = client_socket.recvfrom(1024)
                #print ("READ: {0}".format(data_in))
            except socket.error as detail:
                finished = True
                #print ("Socket error: %s" % detail)
                continue

        return data_in


def print_data(data):
    """ Choose how to print, depending on the type """
    for item in data:
        # Recursively call this function if item is a list
        if isinstance(item, list):
            print_data(item)
        elif isinstance(item, str):
            print ("\t%s" % item)
        elif isinstance(item, float):
            print ("\t%.4f" % item)
        elif isinstance(item, collections.OrderedDict):
            print ("\t%s" % item)
        else:
            print ("Read data of type: %s" % type(item))



def usage(program_name):
    print ("Usage: {0} [server_port_number]\n", program_name)


def main():
    global server_port
    global connected

    px = 0
    py = 0
    pz = 0
    tolerance = 0.5
    speed = 1

    # Read the arguments
    argc = len(sys.argv)
    if argc == 2:
        server_port = int(sys.argv[1])
    elif argc > 3:
        usage(sys.argv[0])
        sys.exit()

    socket_client = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    host = (server_ip, server_port)
    socket_client.setblocking(0)


    while True:
        #--------- ASK FOR OPTIONS ----------------#
        print ("Select an option:")
        print ("a) Enter destination")
        print ("b) Enter speed")
        print ("c) Read coordinates")
        print ("q) Quit client program")
        if sys.version_info >= (3,0,0):
            op = input("Enter option: ")
        else:
            op = raw_input("Enter option: ")

        if op == 'a':
            if sys.version_info >= (3,0,0):
                px = input("Enter X coordinate: ")
                py = input("Enter Y coordinate: ")
                pz = input("Enter Z coordinate: ")
            else:
                px = raw_input("Enter X coordinate: ")
                py = raw_input("Enter Y coordinate: ")
                pz = raw_input("Enter Z coordinate: ")
            waypoint = [float(px), float(py), float(pz), float(tolerance), float(speed)]
            print ("Sending the command: {0}".format(waypoint))

            # Send data
            Data = pickle.dumps((waypoint))
            sent = socket_client.sendto(Data,host)

            print ("Just sent %d bytes to server" % sent)
            # Set the flag that a connection has been established
            connected = True

        elif op == 'b':
            if sys.version_info >= (3,0,0):
                speed = input("Enter speed: ")
            else:
                speed = raw_input("Enter speed: ")
            waypoint = [float(px), float(py), float(pz), float(tolerance), float(speed)]
            print ("Sending the command: {0}".format(waypoint))

            # Send data
            Data = pickle.dumps((waypoint))
            sent = socket_client.sendto(Data,host)

            print ("Just sent %d bytes to server" % sent)

        elif op == 'c':
            # Read data
            data_in = read_data(socket_client)
            try:
                pickled_data = pickle.loads(data_in)
                print_data(pickled_data)
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
