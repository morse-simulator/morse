import sys, os
import json
import re
import yarp
from collections import OrderedDict



# Global variables for the names of the input and output ports
local_in_port_name = ""
local_out_port_name = ""

local_GPS_port_name = ""
local_Status_port_name = ""
local_Dest_port_name = ""
local_Heal_port_name = ""

local_Proximity_in_port_name = ""
local_Proximity_out_port_name = ""


# Global variables for the ors names of the ports
ors_in_port_name = ""
ors_out_port_name = ""

ors_GPS_port_name = ""
ors_Status_port_name = ""
ors_Dest_port_name = ""
ors_Heal_port_name = ""

ors_Proximity_in_port_name = ""
ors_Proximity_in_port_name = ""

# Global variables for the yarp ports
local_in_port = 0
local_out_port = 0

local_GPS_port = 0
local_Status_port = 0
local_Dest_port = 0
local_Heal_port = 0




def read_waypoints():
    """ Read a list of waypoints from a file.
        Format the data as a list of dictionaries,
        in the same format as the java objects from ROSACE."""
    filename = "rosace-waypoints.txt"
    file = open(filename, "r")
    wp_list = []

    for line in file:
        # Get the individual elements, splitting by whitespace
        data_list = line.split()
        print (data_list)
        coordinate = OrderedDict( [ ('x', data_list[0]), ('y', data_list[1]), ('z', data_list[2]) ] )
        waypoint = OrderedDict( [ ('point', coordinate), ('radius', data_list[3]) ] )

        #waypoint = OrderedDict( [ ('x', data_list[0]), ('y', data_list[1]), ('z', data_list[2]), ('speed', data_list[3]) ] )

        wp_list.append (waypoint)

    return wp_list


def command_robot():
    waiting = True
    wp_list = read_waypoints ()

    while waiting:
        """
        yarp_data = local_in_port.read(False)
        if yarp_data != None:
            # Read a string from the bottle
            json_data = yarp_data.toString()
            data = decode_message(json_data)
            print ("Current robot status:")
            for key, value in data.items():
                print ("\t{0}:\t{1}".format(key, value))
        """

        #raw_coords = raw_input("Input new coordinates: ")
        #print ("The coordinates read are: {0}, of type ({1})".format(raw_coords, type(raw_coords)))

        command = input("Enter command: [ (g)ps / (n)eighbours / s(t)atus / (w)aypoint / (h)eal / e(x)it ] ")
        #command = input("Enter command: [ (m)ove / (s)top / s(t)atus / (w)aypoint / (n)eighbours / (h)eal / e(x)it ] ")

        if command == "move" or command == "m":
            command = {"command":"move"}
            send_command(command)

        elif command == "stop" or command == "s":
            command = {"command":"stop"}
            send_command(command)

        elif command == "waypoint" or command == "w":
            wp = wp_list.pop(0)
            message = json.dumps (wp)
            print ("Next waypoint: {0}".format(message))

            # Send the json string through a yarp port
            bottle = local_Dest_port.prepare()
            bottle.clear()
            bottle.addString(message)
            local_Dest_port.write()

        elif command == "heal" or command == "h":
            command = {'heal': 1}
            message = json.dumps (command)
            # Send the json string through a yarp port
            bottle = local_Heal_port.prepare()
            bottle.clear()
            bottle.addString(message)
            local_Heal_port.write()

        elif command == "neighbours" or command == "n" or \
                command == "radius" or command == "r":

            # Separate commands for the radio sensor
            #if command == "neighbours" or command == "n":
                #command = {"request": "Neighbours"}
            #elif command == "radius" or command == "r":
                #command = {"range": "10"}
            #message = json.dumps (command)

            # Send the json string through a yarp port
            #bottle = local_Proximity_out_port.prepare()
            #bottle.clear()
            #bottle.addString(message)
            #local_Proximity_out_port.write()

            # Read the response
            yarp_data = local_Proximity_in_port.read(False)
            if yarp_data != None:
                # Read a string from the bottle
                json_data = yarp_data.toString()
                data = decode_message(json_data)
                print ("Current robot neighbours:")
                for key, value in data.items():
                    print ("\t{0}:\t{1}".format(key, value))

        elif command == "gps" or command == "g":
            # Read the response
            yarp_data = local_GPS_port.read(False)
            if yarp_data != None:
                # Read a string from the bottle
                json_data = yarp_data.toString()
                data = decode_message(json_data)
                print ("Current robot location:")
                for key, value in data.items():
                    print ("\t{0}:\t{1}".format(key, value))

        elif command == "status" or command == "t":
            # Read the response
            yarp_data = local_Status_port.read(False)
            if yarp_data != None:
                # Read a string from the bottle
                json_data = yarp_data.toString()
                data = decode_message(json_data)
                print ("Current robot status:")
                for key, value in data.items():
                    print ("\t{0}:\t{1}".format(key, value))

        elif command == "exit" or command == "x":
            print ("Exiting the function")
            sys.exit()


def send_command(command):
    """ Send a message through the yarp output port."""
    message = json.dumps (command)
    bottle = local_out_port.prepare()
    bottle.clear()
    bottle.addString(message)
    local_out_port.write(False)


def decode_message(json_data):
    """ Decode a data structure using JSON.
        The data is initially a string.
        Returns a Python object,
        either an int, double, string, list or dictionary."""
    # Remove the quotations at the start and end of the string
    json_data = re.sub(r'"(.*)"', r'\1', json_data)
    # Unescape all other quotation marks
    json_data = re.sub(r'\\(.)', r'\1', json_data)
    clean_data = json.loads(json_data)

    return clean_data


def port_setup(robot_name):
    """ Open the input and output ports."""
    global local_in_port
    global local_out_port
    global local_GPS_port
    global local_Status_port
    global local_Dest_port
    global local_Heal_port

    global local_in_port_name
    global local_out_port_name
    global local_GPS_port_name
    global local_Status_port_name
    global local_Dest_port_name
    global local_Heal_port_name

    global local_Proximity_in_port
    global local_Proximity_out_port

    global ors_in_port_name
    global ors_out_port_name
    global ors_GPS_port_name
    global ors_Status_port_name
    global ors_Dest_port_name
    global ors_Heal_port_name
    global ors_Proximity_in_port_name
    global ors_Proximity_out_port_name

    # Define the names for all the ports
    port_prefix = "/ors/robots/" + robot_name + "/"
    local_port_prefix = "/atrv_client/" + robot_name + "/"

    #ors_in_port_name = port_prefix + "in"
    #ors_out_port_name = port_prefix + "out"

    ors_Dest_port_name = port_prefix + "Motion_Controller/in"
    ors_Heal_port_name = port_prefix + "Healer_Beam/in"
    ors_GPS_port_name = port_prefix + "GPS/out"
    ors_Status_port_name = port_prefix + "Status_Sensor/out"

    ors_Proximity_out_port_name = port_prefix + "Proximity_Sensor/out"
    #ors_Proximity_in_port_name = port_prefix + "Proximity_Sensor/in"

    #local_in_port_name = local_port_prefix + "in/"
    #local_out_port_name = local_port_prefix + "out/"

    local_GPS_port_name = local_port_prefix + "GPS/in/"
    local_Status_port_name = local_port_prefix + "Status_Sensor/in/"
    local_Dest_port_name = local_port_prefix + "Motion_Controller/out/"
    local_Heal_port_name = local_port_prefix + "Healer_Beam/out/"

    local_Proximity_in_port_name = local_port_prefix + "Proximity_Sensor/in"
    #local_Proximity_out_port_name = local_port_prefix + "Proximity_Sensor/out"

    # Start the yarp network connection
    yarp.Network.init()

    # Open the client ports
    #local_in_port = yarp.BufferedPortBottle()
    #local_in_port.open(local_in_port_name)
    #local_out_port = yarp.BufferedPortBottle()
    #local_out_port.open(local_out_port_name)

    local_GPS_port = yarp.BufferedPortBottle()
    local_GPS_port.open(local_GPS_port_name)

    local_Status_port = yarp.BufferedPortBottle()
    local_Status_port.open(local_Status_port_name)

    local_Dest_port = yarp.BufferedPortBottle()
    local_Dest_port.open(local_Dest_port_name)

    local_Heal_port = yarp.BufferedPortBottle()
    local_Heal_port.open(local_Heal_port_name)

    #local_Proximity_out_port = yarp.BufferedPortBottle()
    #local_Proximity_out_port.open(local_Proximity_out_port_name)
    local_Proximity_in_port = yarp.BufferedPortBottle()
    local_Proximity_in_port.open(local_Proximity_in_port_name)

    # Connect the client ports to the simulator ports
    #yarp.Network.connect (local_out_port_name, ors_in_port_name)
    #yarp.Network.connect (ors_out_port_name, local_in_port_name)

    yarp.Network.connect (ors_GPS_port_name, local_GPS_port_name)
    yarp.Network.connect (ors_Status_port_name, local_Status_port_name)
    yarp.Network.connect (local_Dest_port_name, ors_Dest_port_name)
    yarp.Network.connect (local_Heal_port_name, ors_Heal_port_name)

    #yarp.Network.connect (local_Proximity_out_port_name, ors_Proximity_in_port_name)
    yarp.Network.connect (ors_Proximity_out_port_name, local_Proximity_in_port_name)




def usage(program_name):
    print ("Usage: {0} [robot_name]\n", program_name)


def main():
    print ("********* ATRV client *********")

    robot_name = "ATRV"

    argc = len(sys.argv)

    if argc == 2:
        robot_name = sys.argv[1]
    elif argc > 3:
        usage(sys.argv[0])
        sys.exit()


    port_setup(robot_name)

    print (" * Writing commands to " + ors_in_port_name)
    print (" * Listening status on " + ors_out_port_name)

    print (" * Writing heal command to " + ors_Heal_port_name)
    print (" * Writing destination to " + ors_Dest_port_name)
    print (" * Listening to GPS on " + ors_GPS_port_name)
    print (" * Listening to robot status on " + ors_Status_port_name)

    print (" * Writing commands to " + ors_Proximity_in_port_name)
    print (" * Listening status on " + ors_Proximity_out_port_name)

    print (" * Enter command:")

    command_robot()


if __name__ == "__main__":
    main()
