import logging; logger = logging.getLogger("morse." + __name__)

import morse.core.sensor

from morse.core.services import service, async_service
from morse.core import status
from morse.helpers.components import add_data, add_property
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

        # Ray trace with these parameters
        self.local_data['node_pose'] = json.dumps(node_pose)