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
    add_data('transducer_pose_list', '', 'string', 'Position and orientation of transducer')
    add_data('node_name', '', 'string', 'Name of this node')

    def __init__(self, obj, parent=None):
        
        logger.info("%s initialization" % obj.name)
        morse.core.sensor.Sensor.__init__(self, obj, parent)

        self.scene = morse.core.blenderapi.scene()
        objs = blenderapi.scene().objects

        # class publishing list to send to moos
        self.transducers = {}
        
        # For the multistatic sim we have 1 transmitter and multiple receivers
        # Set the parent object to be the transmitter 
        transmitter = self.bge_object
        name = transmitter.name
        name = name.replace(".","_")                        # Convert name to moos convention
        direction = "Tx"
        self.transducers[name] = {"obj": transmitter, "direction": direction}

        # Loop through all the objects in the scene and see if they have a receiver property.
        # If they do then we need to send it through to the multistatic simulator for processing.
        for child in objs:
             if 'receiver' in child: # Make sure the variable exists
                if child['receiver'] == True: # If it exists, is it enabled?
                    direction = "Rx"
                    name = child.name
                    name = name.replace(".","_")         # Convert name to moos convention
                    self.transducers[name] = {"obj": child, "direction": direction}

        logger.info('Component initialized, runs at %.2f Hz', self.frequency)

    def add_msg_to_publish_list(self, obj, name, direction):

        if direction == "Tx" or direction == "Rx" or direction == "both":
            pass
        else:
            print("MultiStaticSonar: Send message failed, incorrect direction label. try: 'Tx', 'Rx', 'both'")
            return
        
        node_pos = obj.worldPosition
        node_mat = obj.worldOrientation
        transducer_pose_list = {
           'node_name' : name,
           'pos'       : list(node_pos),
           'X'         : list(node_mat.col[1]),
           'Y'         : list(node_mat.col[2]),
           'Z'         : list(node_mat.col[0]),
           'direction' : direction,
        }

        self.publish_list.append( json.dumps(transducer_pose_list) )
        

    def default_action(self):

        self.publish_list = []
        
        # for each of the transmitters & receivers, assume they are dynamic and update their location.
        for key in self.transducers.keys():
            self.add_msg_to_publish_list( self.transducers[key]["obj"], key, self.transducers[key]["direction"] )

        self.local_data['transducer_pose_list'] = self.publish_list
        
    