import logging; logger = logging.getLogger("morse." + __name__)
from morse.middleware.moos import MOOSNotifier
import morse.core.sensor
from morse.helpers.components import add_data, add_property
from morse.core.mathutils import *
from morse.sensors.ObjectServer import create_trigger_msg
from math import radians, pi
import json

class Lidar(morse.core.sensor.Sensor):
    """Write here the general documentation of your sensor.
    It will appear in the generated online documentation.
    """
    _name = "Lidar"
    _short_desc = "Optix-based lidar simulator"

    # define here the data fields exported by your sensor
    # format is: field name, default initial value, type, description
    add_data('lidar_name',     '', 'string', 'Name of this lidar device')
    add_data('lidar_status', 'ON', 'string', 'Status of this lidar device - ON/OFF')
    add_data('launch_trigger',   '',   'string', 'Information for a radar beam launch')

    add_property('azimuth_width',   360.0, 'azimuth_width',   'float', 'Lidar beam width in degrees')
    add_property('elevation_width', 180.0, 'elevation_width', 'float', 'Lidar beam height in degrees')
    add_property('azimuth_beams',   360,   'azimuth_beams',   'int',   'Number of lidar beams in azimuth direction')
    add_property('elevation_beams', 180,   'elevation_beams', 'int',   'Number of lidar beams in elevation direction')
    add_property('distance_noise',  0.0,   'distance_noise',  'float', 'Distance noise in metres (1 std dev)')
    add_property('max_range',       100.0, 'max_range',       'float', 'Lidar range in m')
    add_property('send_json',       True,  'send_json',       'bool',  'Send small messages as json')

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
        
        pos = self.bge_object.worldPosition
        rotation = self.bge_object.worldOrientation.copy()
            
        if self.send_json:
            self.local_data['launch_trigger'] = {}
            self.local_data['launch_trigger']['launch_trigger'] = create_trigger_msg(pos, rotation, self.azimuth_beams,
                                                                                    self.elevation_beams, 1, True)
            self.local_data['launch_trigger']['max_range'] = self.max_range
            self.local_data['launch_trigger']['azimuth_fov'] = pi * self.azimuth_width / 180.0
            self.local_data['launch_trigger']['elevation_fov'] = pi * self.elevation_width / 180.0
            self.local_data['launch_trigger']['distance_noise'] = self.distance_noise
        else:
            import sys
            sys.path.extend(["/usr/local/share", "/usr/local/share/lidarsim"])
            import lidarsim_capnp as lidarsim
            self.local_data['launch_trigger'] = lidarsim.LidarsimLaunchTrigger.new_message()
            self.local_data['launch_trigger'].launchTrigger = create_trigger_msg(pos, rotation, self.azimuth_beams,
                                                                                 self.elevation_beams, 1, False)
            self.local_data['launch_trigger'].maxRange = self.max_range
            self.local_data['launch_trigger'].azimuthFov = pi * self.azimuth_width / 180.0
            self.local_data['launch_trigger'].elevationFov = pi * self.elevation_width / 180.0
            self.local_data['launch_trigger'].distanceNoise = self.distance_noise

class LidarNotifier(MOOSNotifier):
    """ Notify Lidar """

    def default(self, ci = 'unused'):
        launch_trigger = self.data['launch_trigger']
        msg_name = self.data['lidar_name'] + '_TRIGGER'
        if isinstance(launch_trigger, dict):
            self.notify(msg_name, json.dumps(launch_trigger))
        else:
            self._comms.notify_binary(msg_name, launch_trigger.to_bytes())

    def update_morse_data(self):
        logger.debug('lidarNotifier.update_morse_data() called.')
