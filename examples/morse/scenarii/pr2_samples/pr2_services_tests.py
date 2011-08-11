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


HOST = 'localhost'
PORT = 4000
sock = None


def connect_to_services():
    """
    Connect to the services via a socket.
    """
    # create the client socket to access the services
    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    # connect to the listening socket (HOST, PORT)
    sock.connect((HOST, PORT))
    print("Socket conected to (" + str(HOST) + ", " + str(PORT) + ")")
    return sock


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
    bytessend = sock.send(msg)
    print("Msg send: " + str(msg) + ", bytes: " + str(bytessend))

    #recieve back requested information
    response = sock.recv(1024)    
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
    except NameError as detail:
        print("response: %s, will not be parsed. Error: %s" % (msg, detail))
    except SyntaxError as detail:
        print("response: %s, will not be parsed. Error: %s" % (msg, detail))
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
    

def test_get_armatures(id_, component):
    """
    Method the test the get_armatures service.
    """
    print("\ntest_get_armatures")

    service = 'get_armatures' # MORSE Service to call
    armature_list = gen_send_recv_parse(id_, component, service)

    #print("String: " + response)
    armature_list = armature_list[1]
    print("List: " + str(armature_list))
    return armature_list


def test_get_channels(id_, component, armature_name):
    """
    Get the channels corresponding to the given armature_name via MORSE services.
    """
    print("\nget_channels: " + armature_name)

    service = 'get_channels'
    params = (armature_name,)
    channel_list = gen_send_recv_parse(id_, component, service, params)

    print("channel_list type" + str(type(channel_list)) + ", values: " + str(channel_list))
    return channel_list


def test_get_dofs(id_, component, armature_name):
    """
    Get the dofs axes and the corresponding channels in the given armature_name via MORSE services.
    """
    print("\nget_dofs: " + armature_name)

    service = 'get_dofs'
    params = (armature_name,)
    dof_dict = gen_send_recv_parse(id_, component, service, params)

    print("dof_dict type:" + str(type(dof_dict)) + ", values: " + str(dof_dict))
    return dof_dict


def test_get_dof(id_, component, armature_name, channel_name):
    """
    Get the dof axis of the given armature_name and channel_name via MORSE services.
    """
    print("\nget_dof: " + armature_name + ", " + channel_name)

    service = 'get_dof'
    params = (armature_name, channel_name)
    dof = gen_send_recv_parse(id_, component, service, params)

    print("dof type:" + str(type(dof)) + ", value: " + str(dof))
    return dof


def test_get_rotations(id_, component, armature_name):
    """
    Get the rotation tuples and the corresponding channels int the given armature_name via MORSE services.
    """
    print("\nget_rotations: " + armature_name)

    service = 'get_rotations'
    params = (armature_name,)
    rotation_dict = gen_send_recv_parse(id_, component, service, params)

    print("rotation_dict type:" + str(type(rotation_dict)) + ", values: " + str(rotation_dict))
    return rotation_dict


def test_get_rotation(id_, component, armature_name, channel_name):
    """
    Get the rotation tuple of the given armature_name and channel_name via MORSE services.
    """
    print("\nget_rotation: " + armature_name + ", " + channel_name)

    service = 'get_rotation'
    params = (armature_name, channel_name)
    rotation = gen_send_recv_parse(id_, component, service, params)

    print("rotation type:" + str(type(rotation)) + ", value: " + str(rotation))
    return rotation
    

def test_set_dof_rotations(id_, component, armature_name, angles):
    """
    Set the rotations of the channels of the given armature to the corresponding angles in the given angles dict.
    Set the rotations of the dof axes via MORSE services.
    """
    print("\nset_dof_rotations: %s, angles: %s" % (armature_name, str(angles)))

    service = 'set_dof_rotations'
    params = (armature_name,str(angles))
    rotation_set = gen_send_recv_parse(id_, component, service, params)

    print("rotation_set: " + str(rotation_set))


def test_set_dof_rotation(id_, component, armature_name, channel_name, angle):
    """
    Set the rotation of the dof axis of the channel_name of the given armarture_name to the given angle.
    """
    print("\nset_dof_rotation: %s, %s, angle: %s" % (armature_name, channel_name, str(angle)))

    service = 'set_dof_rotation'
    params = (armature_name, channel_name, angle)
    rotation_set = gen_send_recv_parse(id_, component, service, params)

    print("rotation type:" + str(type(rotation_set)) + ", value: " + str(rotation_set))
    return rotation_set


def main():
    """
    MAIN
    """
    global sock

    id_ = 'req1' # custom request id
    component = 'pr2' # name of the component that offers the service

    # make connection
    sock = connect_to_services()

    # try to get the armatures
    armatures = test_get_armatures(id_, component)
    # test get_channels
    channels_head = test_get_channels(id_, component, armatures[0])
    # test get_dofs
    dofs_l_arm = test_get_dofs(id_, component, armatures[1])
    # test get_dof
    dof_shoulder_pan = test_get_dof(id_, component, armatures[1], 'l_shoulder_pan')
    # test get_rotations
    rotations_r_arm = test_get_rotations(id_, component, armatures[2])
    # test get_rotation
    rotation_r_forearm = test_get_rotation(id_, component, armatures[2], 'r_forearm')

    # test set_dof_rotations
    angles_l_arm = {}
    angles_l_arm['l_shoulder_pan'] = 0.78
    angles_l_arm['l_shoulder_lift'] = 0.78
    angles_l_arm['l_upper_arm'] = 1.57
    angles_l_arm['l_elbow'] = 0.78
    angles_l_arm['l_forearm'] = 3.14
    angles_l_arm['l_wrist_flex'] = 0.78
    angles_l_arm['l_wrist_roll'] = -1.57
    test_set_dof_rotations(id_, component, armatures[1], angles_l_arm)

    # test set_dof_rotation
    test_set_dof_rotation(id_, component, armatures[2], 'r_shoulder_pan', -0.78)
    

    sock.close()
    

if __name__ == "__main__":
    main()
