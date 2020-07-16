import logging; logger = logging.getLogger("morse." + __name__)

import morse.core.sensor

from morse.core.services import service, async_service
from morse.core import status
from morse.helpers.components import add_data, add_property
from morse.core.mathutils import *
from math import radians, pi
import numpy as np
import json

from morse.core import blenderapi

class Lidar(morse.core.sensor.Sensor):
    """Write here the general documentation of your sensor.
    It will appear in the generated online documentation.
    """
    _name = "Lidar"
    _short_desc = "Optix-based lidar simulator"

    # define here the data fields exported by your sensor
    # format is: field name, default initial value, type, description
    add_data('lidar_pose',     '', 'string', 'Position and orientation of Lidar beam')
    add_data('lidar_view',     '', 'string', 'Position and orientation of lidar view')
    add_data('lidar_name',     '', 'string', 'Name of this lidar device')
    add_data('lidar_status', 'ON', 'string', 'Status of this lidar device - ON/OFF')

    add_property('azimuth_width',   100.0, 'Beam_width_azimuth',     'float', 'Lidar beam width in degrees')
    add_property('elevation_width', 40.0,  'Beam_width_elevation',   'float', 'Lidar beam height in degrees')
    add_property('azimuth_beams',   100,   'Number_beams_azimuth',   'int',   'Number of lidar beams in azimuth direction')
    add_property('elevation_beams', 40,    'Number_beams_elevation', 'int',   'Number of lidar beams in elevation direction')
    add_property('lidar_type',      0,     'Lidar_type',             'int',   'Integer value specifying the lidar type')
    add_property('max_range',       100.0, 'Max_range',              'float', 'Lidar range in m')

    def __init__(self, obj, parent=None):
        logger.info("%s initialization" % obj.name)

        # Call the constructor of the parent class
        morse.core.sensor.Sensor.__init__(self, obj, parent)

        self.scene = morse.core.blenderapi.scene()

        # Get the full name of the current lidar
        # (used by middleware driver to name the message)
        # Upper case and underscores as per MOOS convention
        lidar_name = self.bge_object.name.upper()
        self.local_data['lidar_name'] = lidar_name.replace(".","_")

        # Get Blender game object corresponding to beam
        children = self.bge_object.childrenRecursive
        obj = [c for c in children if 'beam' in c.name]

        if obj:

            # Take the first child
            self.beam = obj[0]

            # Beam scalings based on Lidar properties
            x = self.max_range
            y = self.max_range
            z = radians(self.elevation_width)*x

            # Set shapes of Lidar beams
            self.beam.localScale = [x,y,z]

        logger.info('Component initialized')

    def default_action(self):
          
        if hasattr(self,'beam'):

            # Toggle beam visibility
            if self.local_data['lidar_status'] == 'OFF':
                if self.beam.visible:
                    self.beam.setVisible(False,True)
            else:
                if not self.beam.visible:
                    self.beam.setVisible(True,True)

        if self.local_data['lidar_status'] == 'OFF':
            return # No data if lidar is OFF

        # Lidar pose
        lidar_pos = self.bge_object.worldPosition
        lidar_mat = self.bge_object.worldOrientation

        lidar_pose = {
           'lidar_name' : self.local_data['lidar_name'],
           'max_range'  : self.max_range,
           'azim_width' : self.azimuth_width,
           'elev_width' : self.elevation_width,
           'azim_beams' : self.azimuth_beams,
           'elev_beams' : self.elevation_beams,
           'lidar_type' : self.lidar_type,
           'pos'        : list(lidar_pos),
           'X'          : list(lidar_mat.col[1]),
           'Y'          : list(lidar_mat.col[2]),
           'Z'          : list(lidar_mat.col[0]),
        }

        # Ray trace with the current lidar beam pose
        self.local_data['lidar_pose'] = json.dumps(lidar_pose)

        # Figure out which camera is active and
        # publish its position and view vector.
        camera = self.scene.active_camera
        pos = camera.worldPosition
        mat = camera.worldOrientation

        X = list(mat.col[0])
        Y = list(mat.col[1])
        Z = list(mat.col[2])

        # Z-axes reversed for some reason...
        Z = [-z for z in Z]

        lidar_view = {
           'camera' : camera.name, 
           'pos'    : list(pos),
           'X'      : X,
           'Y'      : Y,
           'Z'      : Z,
        }

        # Point cloud view will be active camera view
        self.local_data['lidar_view'] = json.dumps(lidar_view)

