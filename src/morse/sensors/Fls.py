import logging; logger = logging.getLogger("morse." + __name__)

import morse.core.sensor

from morse.core.services import service, async_service
from morse.core import status
from morse.helpers.components import add_data, add_property
from math import radians

class FLS(morse.core.sensor.Sensor):
    """Write here the general documentation of your sensor.
    It will appear in the generated online documentation.
    """
    _name = "FLS"
    _short_desc = "Simple forward-looking sonar obstacle detector"

    # define here the data fields exported by your sensor
    # format is: field name, default initial value, type, description
    add_data('status','OFF','string','FLS status')
    add_data('range', 0.0, "float", 'Distance to FLS contact')

    add_property('azimuth_width',    1.0, 'Beam_width_azimuth'  ,'float','FLS beam width in degrees')
    add_property('elevation_width',  1.0, 'Beam_width_elevation','float','FLS beam height in degrees')
    add_property('max_range',        20.0,'Max_range', "float",
        "Maximum distance to which ground is detected. If nothing is detected, return +infinity")

    def __init__(self, obj, parent=None):

        logger.info("%s initialization" % obj.name)
        # Call the constructor of the parent class
        morse.core.sensor.Sensor.__init__(self, obj, parent)

        # Do here sensor specific initializations        
        # Set handles to various Blender game objects
        self.FLS_beam  = parent.bge_object.children['FLS_beam']

        # Beam scalings based on beam properties
        x = self.max_range
        y = radians(self.azimuth_width)*x
        z = radians(self.elevation_width)*x

        # Set FLS beam shape
        self.FLS_beam.localScale = [x,y,z]

        # Initialise FLS visibility
        self.FLS_beam.setVisible(False)

        # Set visibility flag
        self._visible = False

        # Initial range
        self._distance = 0

        logger.info('Component initialized')

    def default_action(self):

        # Toggle FLS beam visibility
        if self.local_data['status'] == 'OFF':
            if self._visible:
                self.FLS_beam.setVisible(False)
                self._visible = False
            return # No data if FLS is OFF
        else:
            if not self._visible:
                self.FLS_beam.setVisible(True)
                self._visible = True

        # Location of FLS transmitter
        source = self.bge_object.worldPosition
        
        # Direction of FLS beam (local x-axis)
        vect = self.bge_object.worldOrientation.col[0]

        # Target to hit
        target = source + vect

        # Send a ray out infront of the vehicle
        obj, point, _ = self.bge_object.rayCast(target, source, self.max_range)
        logger.debug("FLS points to %s and hits %s" % (target, point))
    
        if point:
            self.local_data['range'] = self.bge_object.getDistanceTo(point)
            msg = 'x = ' + str(point.x) + ',y = ' + str(point.y) + ',label = ' + obj.name
            self.local_data['feature'] = msg
        else:
            self.local_data['range'] = float('inf')
