# @Authors: Peter Roelants peter.roelants@gmail.com
# @Owner: KU Leuven - Dep. Mechanical Eng. - Robotics
# @File: pr2_services_tests.py
# @Description:  
# @License: 
# (C) 2010 Name, peter.roelants@gmail.com, Department of Mechanical
# Engineering, Katholieke Universiteit Leuven, Belgium.
#
# You may redistribute this software and/or modify it under either the terms of the GNU Lesser
# General Public License version 2.1
# (LGPLv2.1 <http://www.gnu.org/licenses/old-licenses/lgpl-2.1.html>)
# or (at your discretion) of the Modified BSD License:
# Redistribution and use in source and binary forms, with or without modification, are permitted
# provided that the following conditions are met:
#  1. Redistributions of source code must retain the above copyright notice, this list of 
#     conditions and the following disclaimer.
#  2. Redistributions in binary form must reproduce the above copyright notice, this list of 
#     conditions and the following disclaimer in the documentation and/or other materials
#     provided with the distribution.
#  3. The name of the author may not be used to endorse or promote products derived from
#     this software without specific prior written permission.
# THIS SOFTWARE IS PROVIDED BY THE AUTHOR ``AS IS'' AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING,
# BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE 
# ARE DISCLAIMED. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
# EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
# OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY 
# THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR 
# OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY 
# OF SUCH DAMAGE.
"""
Script to test the different MORSE PR2 Services.

The MORSE Simulation with a PR2 model 'pr2' must be running for this to work.
"""

import sys
import socket
import time

host = 'localhost'
services_port = 4000
service_socket = None


def connect_to_services():
    """
    Connect to the services via a socket.
    """
    # create the client socket to access the services
    service_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    # connect to the listening socket (host, services_port)
    service_socket.connect((host, services_port))
    print("service_socket conected to (" + str(host) + ", " + str(services_port) + ")")
    return service_socket


def generate_msg(id_, component, service, params = None):
    """
    Generate a message to be sent to the MORSE Service via a socket.
    params has to be a tuple
    """
    if params is None:
        msg = id_ + " " + component + " " + service + "\n"
    else:
        msg = id_ + " " + component + " " + service + " " + str(params) + "\n"
    return msg

def send_recv(msg):
    """
    Send a service request and receive and return the reply as a string
    """
    #Send request
    msg = msg.encode('ascii')
    bytessend = service_socket.send(msg)
    print("Msg send: " + str(msg) + ", bytes: " + str(bytessend))

    #recieve back requested information
    response = service_socket.recv(1024)    
    print("Msg response: " + str(response))
    return response.decode('ascii')

def parse_response(msg, id_):
    """
    Parse a message that is a response from a MORSE service 
    """
    msg = msg.strip() # Remove EOL character
    msg = msg.replace(id_ + " ", '') # Remove request id
    msg = msg.replace("SUCCESS ", '') # Remove SUCCESS status
    msg = msg.replace("SUCCESS", '') # Sometimes SUCCESS has a space after it, sometimes not.
    try:
        # Use an empty dictionary as the second parameter to 'eval'
        #  to restrict the environment where it executes,
        #  and avoid security problems.
        msg = eval(msg, {}) # Parse string into list/dict/..
    except (NameError, SyntaxError) as detail:
        #print("response: %s, will not be parsed. Error: %s" % (msg, detail))
        pass
    return msg

def gen_send_recv_parse(id_, component, service, params = None):
    """
    Generates a message from the given parameters.
    Sends the request and receives the response.
    Parses the result and returns it.
    """
    msg = generate_msg(id_, component, service, params)
    msg = send_recv(msg)
    msg = parse_response(msg, id_)
    return msg


def is_service_socket_connected():
    """
    Method to check if the service_socket is still connected
    """
    # If the bytestring received is empty, the service_socket is not connected.
    if service_socket.recv(1024) == b'':
        return False
    return True


def get_armatures(id_, component):
    """
    Method the test the get_armatures service provided by the PR2.
    """
    print("\nget_armatures:")

    service = 'get_armatures' # MORSE Service to call
    armature_list = gen_send_recv_parse(id_, component, service)

    print("\tarmature_list type" + str(type(armature_list)) + ", values: " + str(armature_list))
    return armature_list

def get_torso(id_, component):
    """
    Method that returns the z-translation of the torso.
    """
    print("\nget_torso:")
    
    service = 'get_torso'
    height = gen_send_recv_parse(id_, component, service)
    
    print("\theight type" + str(type(height)) + ", values: " + str(height))
    return height

def set_torso(id_, component, height):
    """
    Method that sets the z-translation of the torso to original_z + height.
    """
    print("\nset_torso:")
    
    service = 'set_torso'
    params = (height,)  
    gen_send_recv_parse(id_, component, service, params)
    
    return None

def get_torso_minmax(id_, component):
    """
    Method that returns the minimum an maximum z-translation that the torso can make from the base.
    Returns a list [min,max]
    """
    print("\nget_torso_minmax:")
    
    service = 'get_torso_minmax'
    minmax = gen_send_recv_parse(id_, component, service)
    
    print("\tminmax type" + str(type(minmax)) + ", values: " + str(minmax))
    return minmax

    
def get_channels(id_, component):
    """
    Get the channels of the armature via MORSE services.
    """
    print("\nget_channels: ")

    service = 'get_channels'
    channel_list = gen_send_recv_parse(id_, component, service)

    print("\tchannel_list type" + str(type(channel_list)) + ", values: " + str(channel_list))
    return channel_list

def get_rotations(id_, component):
    """
    Get the rotation tuples and the corresponding channels of the armature via MORSE services.
    """
    print("\nget_rotations: ")

    service = 'get_rotations'
    rotation_dict = gen_send_recv_parse(id_, component, service)
    
    print("\trotation_dict type:" + str(type(rotation_dict)) + ", values: " + str(rotation_dict))
    return rotation_dict

def get_rotation(id_, component, channel_name):
    """
    Get the rotation tuple of the given channel_name via MORSE services.
    """
    print("\nget_rotation: " + channel_name)

    service = 'get_rotation'
    params = (channel_name,)
    rotation = gen_send_recv_parse(id_, component, service, params)

    print("\trotation type:" + str(type(rotation)) + ", value: " + str(rotation))
    return rotation

def get_dofs(id_, component):
    """
    Get the dofs list corresponding to each channel as a dict via MORSE services.
    """
    print("\nget_dofs:")

    service = 'get_dofs'
    dofs_dict = gen_send_recv_parse(id_, component, service)

    print("\tdofs_dict type:" + str(type(dofs_dict)) + ", values: " + str(dofs_dict))
    return dofs_dict

def set_rotation(id_, component, channel_name, angles):
    """
    Set the axes with a dof of the channel 'channel_name' to the given angles (x,y,z).
    """
    print("\nset_rotation: channel: " + channel_name + ", angles: " + str(angles))

    service = 'set_rotation'
    params = (channel_name, angles)
    gen_send_recv_parse(id_, component, service, params)

def get_IK_minmax(id_, component):
    """
    Get the IK min/max limits as a dictionary with keys the channel names of the armature
    and values the IK min and max limits in the following form:
    [[ik_min_x,ik_max_x], [ik_min_y,ik_max_y], [ik_min_z,ik_max_z]] (list of lists of floats)
    """
    print("\nget_IK_minmax:")

    service = 'get_IK_minmax'
    minmax_dict = gen_send_recv_parse(id_, component, service)

    print("\tminmax_dict:" + str(type(minmax_dict)) + ", values: " + str(minmax_dict))
    return minmax_dict

def get_IK_limits(id_, component):
    """
    Get the IK limits a dict with keys the channel names of the armature
    and values the IK limits in the following form:
    [ik_limit_x,ik_limit_y,ik_limit_z] (list of booleans)
    """
    print("\nget_IK_limits:")

    service = 'get_IK_limits'
    limits_dict = gen_send_recv_parse(id_, component, service)

    print("\tlimits_dict:" + str(type(limits_dict)) + ", values: " + str(limits_dict))
    return limits_dict

def get_channel_lengths(id_, component):
    """
    Get the channel lenghts as a dict with keys the channel names of the armature
    and values the channel's lenght.
    """
    print("\nget_channel_lengths:")

    service = 'get_channel_lengths'
    lenghts_dict = gen_send_recv_parse(id_, component, service)

    print("\tlenghts_dict:" + str(type(lenghts_dict)) + ", values: " + str(lenghts_dict))
    return lenghts_dict

def get_robot_parent_name(id_, component):
    """
    Get the blender name of the robot that is the parent of the armature.
    """
    print("\nget_robot_parent_name:")

    service = 'get_robot_parent_name'
    name = gen_send_recv_parse(id_, component, service)

    print("\tname:" + str(type(name)) + ", values: " + str(name))
    return name


def main():
    """
    MAIN
    """
    global service_socket

    id_ = 'req1' # custom request id
    component = 'pr2' # name of the component that offers the service

    service_socket = connect_to_services()  # make connection

    # test out the pr2 services
    armatures = get_armatures(id_, component)  # try to get the armatures
    height = get_torso(id_, component)  # test get_torso
    set_torso(id_, component, 0.3)  # test set_torso
    height = get_torso(id_, component)  # test get_torso
    minmax = get_torso_minmax(id_, component)  # test get_torso_minmax

    # test out the armature actuator services
    for armature in armatures:
        component = armature
        print("\n\tNew component: " + component)
        
        channels = get_channels(id_, component)  # test get_channels
        get_rotations(id_, component)  # test get_rotations
        get_rotation(id_, component, channels[0])  # test get_rotation
        get_dofs(id_, component)  # test get_dofs
        get_IK_minmax(id_, component)  # test get_IK_minmax
        get_IK_limits(id_, component)  # test get_IK_limits
        # test set_rotation
        for channel in channels:
            set_rotation(id_, component, channel, [0.785398163, 0.785398163, 0.785398163]) # set every angle to 45 degree
        get_rotations(id_, component)  # test get_rotations to see what angles have been changed by set_rotation
        get_channel_lengths(id_, component)  # test get_channel_lengths
        get_robot_parent_name(id_, component)  # test get_robot_parent_name


if __name__ == "__main__":
    # try to catch all errors and exceptions in the main so that we can wait for the connection
    # to close before we end the script.
    try:
        main()
    except:
        e = sys.exc_info()
        print("Error catched in main: \n" + str(e))

    # close the service_socket and and the script only if the service_socket is not connected anymore
    # otherwise MORSE will have to handle exceptions
    print("Wait for connection to close.")
    is_connected = is_service_socket_connected()
    while (is_connected):
        time.sleep(1.0)
        is_connected = is_service_socket_connected()
    print("Connection is closed.")

    # try to close the socket
    try:
        service_socket.close()
    except socket.error as error_info:
        print("Error catched while closing service_socket: " + str(error_info))

