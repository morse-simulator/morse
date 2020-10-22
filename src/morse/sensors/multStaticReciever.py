import logging; logger = logging.getLogger("morse." + __name__)
from math import pi, sqrt
import morse.core.actuator
from morse.core import status
from morse.helpers.components import add_data, add_property
from morse.core import blenderapi
from morse.builder.abstractcomponent import AbstractComponent
from morse.core.mathutils import *
import json

__author__     = "Thomas McCabe"
__copyright__  = "Copyright 2020, Mission Systems Pty Ltd"
__license__    = "BSD"
__version__    = "1.0.4"
__maintainer__ = "Thomas McCabe"
__email__      = "tom.mccabe@missionsystems.com.au"
__status__     = "Draft"


class SoccerBall(AbstractComponent):

    print("made it here")
    # define here the data fields exported by your sensor
    # format is: field name, default initial value, type, description
    add_data('node_pose', '', 'string', 'Position and orientation of node')
    add_data('node_name', '', 'string', 'Name of this node')

    # Init object
    def __init__(self,object):

        # Get every bge object in the scene
        objs = blenderapi.scene().objects

        # Get the soccer ball object
        soccer_balls = objs['soccer_ball']

        for ball in soccer_balls:

            #print to see how many we found
            print("found ball")


            # Get the name of the soccerball
            node_name = ball.name
            self.local_data['node_name'] = node_name.replace(".","_")

            # Get the pose fo the soccer ball object
            #  pose
            node_pos = self.bge_object.worldPosition
            node_mat = self.bge_object.worldOrientation

            node_pose = {
            'node_name' : self.local_data['node_name'],
            'pos'       : list(node_pos),
            'X'         : list(node_mat.col[1]),
            'Y'         : list(node_mat.col[2]),
            'Z'         : list(node_mat.col[0]),
            }

            # publish the pose to the moosDB
            self.local_data['node_pose'] = json.dumps(node_pose)


