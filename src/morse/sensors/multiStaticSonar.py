import logging; logger = logging.getLogger("morse." + __name__)

import morse.core.sensor

from morse.core.services import service, async_service
from morse.core import status
from morse.helpers.components import add_data, add_property
from morse.core import blenderapi
from morse.middleware.moos import MOOSNotifier
import json

class MultiStaticSonar(morse.core.sensor.Sensor):
    """Write here the general documentation of your sensor.
    It will appear in the generated online documentation.
    """
    _name = "MultiStaticSonar"
    _short_desc = "MultiStaticSonar for tracking communications ranges and beam orientations"

    # define here the data fields exported by your sensor
    # format is: field name, default initial value, type, description
    add_data('node_pose', '', 'string', 'Position and orientation of node')
    add_data('node_name', '', 'string', 'Name of this node')

    def __init__(self, obj, parent=None):
        
        # class publishing list to send to moos
        self.publish_list = []
        
        logger.info("%s initialization" % obj.name)

        # Call the constructor of the parent class
        morse.core.sensor.Sensor.__init__(self, obj, parent)

        self.scene = morse.core.blenderapi.scene()

        # Get the full name of the current node
        # (used by middleware driver to name the message)
        # Upper case and underscores as per MOOS convention
        node_name = self.bge_object.name
        self.local_data['node_name'] = node_name.replace(".","_")

        logger.info('Component initialized, runs at %.2f Hz', self.frequency)

    def default_action(self):

        #empty the publishing list. Might not need to do this? 
        self.publish_list = []

        # Acomms transducer pose
        node_pos = self.bge_object.worldPosition
        node_mat = self.bge_object.worldOrientation

        node_pose = {
           'node_name' : self.local_data['node_name'],
           'pos'       : list(node_pos),
           'X'         : list(node_mat.col[1]),
           'Y'         : list(node_mat.col[2]),
           'Z'         : list(node_mat.col[0]),
        }

        # Add pose of the current AUV to the list
        self.publish_list.append(json.dumps(node_pose))

        self.update_receiver()

    def update_receiver(self):
        print('added receiever to ball')
        counter = 0
        # Get every bge object in the scene
        objs = blenderapi.scene().objects

        # loop through all objs to find xnumber of soccer balls
        soccer_balls = []
        for child in objs:
            if "soccer_ball" in child.name: 
                print("found ball")
                print(child)
                soccer_balls.append(child)

        # objs gives them to us higest number first so iterating backwards
        for ball in reversed(soccer_balls):
            
            # Get the name of the soccerball
            node_name = ball.name
            #print(node_name)
            self.local_data['node_name'] = node_name.replace(".","_")

            # Get the pose fo the soccer ball object
            #  pose
            node_pos = ball.worldPosition
            node_mat = ball.worldOrientation

            node_pose = {
            'node_name' : self.local_data['node_name'],
            'pos'       : list(node_pos),
            'X'         : list(node_mat.col[1]),
            'Y'         : list(node_mat.col[2]),
            'Z'         : list(node_mat.col[0]),
            }

            # publish the pose to the moosDB
            self.publish_list.append(json.dumps(node_pose)  )
            counter += 1        
            
            # number of soccer balls to track
            if counter == 5:
                break

        self.local_data['node_pose'] = self.publish_list
