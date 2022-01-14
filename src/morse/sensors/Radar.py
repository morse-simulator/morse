import logging; logger = logging.getLogger("morse." + __name__)
from morse.middleware.moos import MOOSNotifier
import morse.core.sensor
from morse.helpers.components import add_data, add_property
from morse.core.mathutils import *
from morse.sensors.ObjectServer import create_trigger_msg
from math import radians, pi
import json

class Radar(morse.core.sensor.Sensor):
    """Write here the general documentation of your sensor.
    It will appear in the generated online documentation.
    """
    _name = "Radar"
    _short_desc = "Radar beam simulation"

    # define here the data fields exported by your sensor
    # format is: field name, default initial value, type, description
    add_data('launch_trigger',   '',   'string', 'Information for a radar beam launch')
    add_data('object', '',   'string', 'Which object are we looking at?')
    add_data('status', 'ON', 'string', 'Radar status - ON/OFF')
    add_data('ping',     0,   'float', 'Radar ping interval (s)')

    # Initialises some properties. They can be changed by Builder scripts
    add_property('azimuth_width',   30.0,  'azimuth_width',   'float', 'Radar beam width in degrees')
    add_property('elevation_width', 30.0,  'elevation_width', 'float', 'Radar beam height in degrees')
    add_property('max_range',       200.0, 'max_range',       'float', 'Radar range in m')
    add_property('azim_beams',      500,   'azim_beams',      'float', 'Number of azimuth (X) beams')
    add_property('elev_beams',      500,   'elev_beams',      'float', 'Number of elevation (Y) beams')
    add_property('send_json',       True,  'send_json',       'bool',  'Send small messages as json')

    def __init__(self, obj, parent=None):
        
        logger.info("%s initialization" % obj.name)

        # Call the constructor of the parent class
        morse.core.sensor.Sensor.__init__(self, obj, parent)

        # Get the name of the current child
        # (used by middleware driver to name the message)
        child_name = self.bge_object.name.split('.')[-1]
        self.local_data['object'] = child_name

        # Get Blender game object (used in default action)
        self.beam = self.bge_object
        self.parent = self.robot_parent.bge_object

        # Beam scalings based on Radar properties
        x = radians(self.elevation_width)*self.max_range
        y = radians(self.azimuth_width)*self.max_range
        z = self.max_range

        # Set shapes of Radar beams
        self.beam.localScale = [x,y,z]

        # Start with beam turned off
        self.beam.setVisible(False, True)

        logger.info('Component initialized')

    def default_action(self):
        # Toggle Radar beam visibility
        if self.local_data['status'] == 'OFF':
            if self.beam.visible:
                self.beam.setVisible(False, True)
            return # No data if Radar is OFF
        else:
            if not self.beam.visible:
                self.beam.setVisible(True, True)

        # Radar pose
        pos = self.bge_object.worldPosition
        rotation = self.bge_object.worldOrientation.copy()

        # Create launch trigger
        if self.send_json:
            self.local_data['launch_trigger'] = {}
            self.local_data['launch_trigger']['launch_trigger'] = create_trigger_msg(pos, rotation, self.azim_beams,
                                                                                    self.elev_beams, 1, True)
            self.local_data['launch_trigger']['max_range'] = self.max_range
            self.local_data['launch_trigger']['azimuth_fov'] = pi * self.azimuth_width / 180.0
            self.local_data['launch_trigger']['elevation_fov'] = pi * self.elevation_width / 180.0
        else:
            import sys
            sys.path.extend(["/usr/local/share", "/usr/local/share/r4"])
            import r4_capnp as r4
            self.local_data['launch_trigger'] = r4.RadarLaunchTrigger.new_message()
            self.local_data['launch_trigger'].launchTrigger = create_trigger_msg(pos, rotation, self.azim_beams,
                                                                                 self.elev_beams, 1, False)
            self.local_data['launch_trigger'].maxRange = self.max_range
            self.local_data['launch_trigger'].azimuthFov = pi * self.azimuth_width / 180.0
            self.local_data['launch_trigger'].elevationFov = pi * self.elevation_width / 180.0

        # Minimum ping interval without range ambiguity
        self.local_data['ping'] = 2.0 * self.max_range / 3.0e8;

class RadarNotifier(MOOSNotifier):
    """ Notify Radar """

    def default(self, ci = 'unused'):
        launch_trigger = self.data['launch_trigger']
        # ts = self.data['timestamp']
        msg_name = 'RADAR_TRIGGER'
        if isinstance(launch_trigger, dict):
            self.notify(msg_name, json.dumps(launch_trigger))
        else:
            self._comms.notify_binary(msg_name, launch_trigger.to_bytes())

    def update_morse_data(self):
        logger.debug('radarNotifier.update_morse_data() called.')
