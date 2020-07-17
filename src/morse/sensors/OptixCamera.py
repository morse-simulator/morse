import logging; logger = logging.getLogger("morse." + __name__)

import morse.core.sensor
import morse.sensors.camera

from morse.core.services import service, async_service
from morse.core import status
from morse.helpers.components import add_data, add_property
from morse.core.mathutils import *
from math import radians, pi
import numpy as np
import json

class Optixcamera(morse.sensors.camera.Camera):
    """Write here the general documentation of your sensor.
    It will appear in the generated online documentation.
    """
    _name = "Optixcamera"
    _short_desc = "Ray-tracing accelerated morse camera sensor."

    # define here the data fields exported by your sensor
    # format is: field name, default initial value, type, description
    add_data('optix_camera_pose',     '', 'string', 'Position and orientation of the optix camera')
    add_data('optix_camera_view',     '', 'string', 'Position and orientation of optix camera view')
    add_data('optix_camera_name',     '', 'string', 'Name of this optix camera device')
    add_data('optix_camera_status', 'ON', 'string', 'Status of this optix camera device - ON/OFF')  # maybe we wont need all of these. some might have been used for the debug lines?

    # format is: field name, default initial value, name (must match the blender name), type, description
    add_property('image_width',     256,    'cam_width in pixels')
    add_property('image_height',    256,    'cam_height in pixels')
    add_property('image_focal',     25.0,   'cam_focal length in mm')
    add_property('vertical_flip',   True,   'Vertical_Flip the final image')
    add_property('max_range',       200.0,  'Max_range','float','Camera range in m')

    def __init__(self, obj, parent=None):
        logger.info("%s initialization" % obj.name)
        # Call the constructor of the parent class
        morse.core.sensor.Sensor.__init__(self, obj, parent)

        self.scene = morse.core.blenderapi.scene()

        # Get the full name of the current optix camera
        # (used by middleware driver to name the message)
        # Upper case and underscores as per MOOS convention
        optix_camera_name = self.bge_object.name.upper()
        self.local_data['optix_camera_name'] = optix_camera_name.replace(".","_")

        # Removed the beam specific initialization that appeared in
        # the lidar code, which I think was for drawing lidar debug lines.

        logger.info('Component initialized')


    def default_action(self):
        """ Main loop of the sensor.

        Implements the component behaviour
        """
        
        # Removed the beam code that appeared in the Lidar sim that
        # was probably used to draw the debug lines in the camera 
        # viewport.
        
        # Optix camera pose
        optix_camera_pos = self.bge_object.worldPosition
        optix_camera_mat = self.bge_object.worldOrientation

        optix_camera_pose = {
           'optix_camera_name' : self.local_data['optix_camera_name'],
           'max_range'         : self.max_range,
           'pos'               : list(optix_camera_pos),
           'X'                 : list(optix_camera_mat.col[1]),
           'Y'                 : list(optix_camera_mat.col[2]),
           'Z'                 : list(optix_camera_mat.col[0]),
        }
        
        # Ray trace with the current lidar beam pose
        self.local_data['optix_camera_pose'] = json.dumps(optix_camera_pose)

        

