import logging; logger = logging.getLogger("morse." + __name__)

import morse.core.sensor

from morse.core.services import service, async_service
from morse.core import status
from morse.helpers.components import add_data, add_property
from morse.core.mathutils import *
from math import radians, pi
import numpy as np
import json

class Sonar(morse.core.sensor.Sensor):
    """Write here the general documentation of your sensor.
    It will appear in the generated online documentation.
    """
    _name = "Sonar"
    _short_desc = "Optix sonar beam simulation"

    # define here the data fields exported by your sensor
    # format is: field name, default initial value, type, description
    add_data('pose',   '',   'string', 'Positions and orientations of sonar beams')
    add_data('object', '',   'string', 'What is the current object?')
    add_data('status','ON',  'string', 'Sonar status - ON/OFF')

    # Initialises some properties. They can be changed by Builder scripts
    add_property('start_alt',       8.0,   'start_altitude' ,'float','Sonar activation altitude threshold in m')
    add_property('stop_alt',       12.0,   'stop_altitude' ,'float','Sonar de-activation altitude threshold in m')
    add_property('azimuth_angle',   0.0,   'Beam_direction' ,'float','Sonar beam direction in degrees')
    add_property('elevation_angle', 0.0,   'Beam_depression_angle' ,'float','Sonar beam depression in degrees')
    add_property('azimuth_width',   30.0,  'Beam_width_azimuth' ,'float','Sonar beam width in degrees')
    add_property('elevation_width', 30.0,  'Beam_width_elevation' ,'float','Sonar beam height in degrees')
    add_property('sound_speed',   1500.0,  'sound_speed' ,'float','Water sound speed in m/s')    
    add_property('max_range',       50.0,  'Max_range','float','Sonar range in m')

    def __init__(self, obj, parent=None):
        
        logger.info("%s initialization" % obj.name)

        # Call the constructor of the parent class
        morse.core.sensor.Sensor.__init__(self, obj, parent)

        # Get the name of the current child
        # (used by middleware driver to name the message)
        child_name = self.bge_object.name.split('.')[-1]
        self.local_data['object'] = child_name

        # Get Blender game object
        # (used in default action)
        self.beam = self.bge_object
        self.parent = self.robot_parent.bge_object

        # Beam scalings based on sonar properties
        z = self.max_range
        x = radians(self.azimuth_width)*z
        y = radians(self.elevation_width)*z

        # Set shapes of sonar beams
        self.beam.localScale = [x,y,z]

        # Construct rotation matrix
        elev = radians(90+self.elevation_angle)
        azimuth = radians(self.azimuth_angle)

        rot = Euler([elev,0.0,azimuth])

        # Rotate to desired depression angle
        self.beam.localOrientation = rot

        # Start with beam turned off
        # self.beam.setVisible(False,True)

        # Calculate the ping rate
        # (Maximum without range ambiguity)
        # pri = 2.0 * self.max_range / self.sound_speed;

        logger.info('Component initialized, runs at %.2f Hz', self.frequency)

    def default_action(self):

        depth = -self.parent.worldPosition.z

        if depth > 10:
            self.local_data['status'] = 'ON'
        else:
            self.local_data['status'] = 'OFF'

        # Toggle sonar beam visibility
        if self.local_data['status'] == 'OFF':
            if self.beam.visible:
                self.beam.setVisible(False,True)
            return # No ping if sonar is OFF
        else:
            if not self.beam.visible:
                self.beam.setVisible(True,True)

        # Sonar beam dynamic data
        pos = self.beam.worldPosition
        mat = self.beam.worldOrientation
        # linVel = self.parent.getLinearVelocity(True)
        angVel = self.parent.getAngularVelocity(False)
        yaw_rate = angVel[2]

        pose = {
           'status'     : self.local_data['status'],
           'max_range'  : self.max_range,
           'elev_width' : self.elevation_width,
           'azim_width' : self.azimuth_width,
           'pos'        : list(pos),
           'X'          : list(mat.col[0]),
           'Y'          : list(mat.col[1]),
           'Z'          : list(mat.col[2]),
           'yaw_rate'   : yaw_rate
        }

        # Ping with the current sonar pose
        self.local_data['pose'] = json.dumps(pose)

