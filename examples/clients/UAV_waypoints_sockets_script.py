""" Simple script for the FLYING CAT AND MOUSE game tutorial

This will command the flying cat, using the pose sensor of the mouse, to follow
after the latter."""

import math
from pymorse import Morse
# Use sockets through pymorse interface


""" The minimal distance to maintain between the mouse and the cat. """
minDist = 4.0

""" The height for the flying cat. """
# NB: this is the absolute height not the one relative to the ground...
# TODO: use sensors (laser?) to take into account the ground and the obstacle
height= 3.5 


def where_is(agentPose_stream):
    """ Read data from the [mouse|cat] pose sensor, and determine the position of the agent """
    pose = agentPose_stream.get()

    return pose


def frighten_mouse():
    """ Use the mouse pose sensor to locate and "chase" it """

    with Morse() as morse:
        catPose = morse.cat.catPose
        mousePose = morse.mouse.mousePose
        motion = morse.cat.waypoint

        while True:
            catPosition = where_is(catPose)
            mousePosition = where_is(mousePose)

            # print(mousePosition)
            # print(catPosition)

            if mousePosition and catPosition:
                # go behind the mouse
                waypoint = {    "x": mousePosition['x'] - minDist*math.cos(mousePosition['yaw']), \
                                "y": mousePosition['y'] - minDist*math.sin(mousePosition['yaw']), \
                                "z": height, \
                                "yaw": catPosition['yaw'], \
                                "tolerance": 0.5 \
                            }

                # look at the mouse
                if mousePosition['x']==catPosition['x']:
                     waypoint['yaw']= math.sign(mousePosition['y']-catPosition['y']) * math.pi
                else:
                    waypoint['yaw']= math.atan2(mousePosition['y']-catPosition['y'],mousePosition['x']-catPosition['x'])
                
                # send the command through the socket
                # print(waypoint)
                motion.publish(waypoint)


def main():
    """ Main behaviour """
    frighten_mouse()

if __name__ == "__main__":
    main()
