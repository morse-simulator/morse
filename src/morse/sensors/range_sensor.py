import logging; logger = logging.getLogger("morse." + __name__)
import morse.core.sensor
from morse.helpers.components import add_data, add_property

class RangeSensor(morse.core.sensor.Sensor):
    """Generic range based sensor that returns the result of a ray cast to the nearest object in the frame of the sensor.
    """
    _name = "RangeSensor"
    _short_desc = "Simple range sensor"

    # define here the data fields exported by your sensor
    # format is: field name, default initial value, type, description
    add_data('range', 0.0, "float", 'Distance to contact')
    add_data('name', "range_sensor", "string", 'Sensor name')

    # Format is data_name, default_val, argument name, type, description
    add_property('sensor_obj',  'distance_sensor', 'SensorObj','string','Name of the object in blender to attach to.')
    add_property('max_range',   1000.0,            'MaxRange', "float",
        "Maximum distance to which objects are detected. If nothing is detected, return +infinity")


    def __init__(self, obj, parent=None):

        logger.info("%s initialization" % obj.name)
        # Call the constructor of the parent class
        morse.core.sensor.Sensor.__init__(self, obj, parent)
        # Do here sensor specific initializations        
        self.sensor  = parent.bge_object.children[self.sensor_obj]
        # Set the name for the sensor so that multiple sensors can have unique topics in middleware
        lidar_name = self.sensor.name.upper()
        self.local_data['name'] = lidar_name.replace(".","_")

        logger.info('Component initialized')

    def default_action(self):

        # Location of range sensor
        source = self.sensor.worldPosition
        # Direction of range sensor (local x-axis)
        vect = self.sensor.worldOrientation.col[0]
        # Target to hit
        target = source + vect
        # Send a ray out from the sensor
        _, point, _ = self.bge_object.rayCast(target, source, self.max_range)

        logger.debug("Range sensor points to %s and hits %s" % (target, point))

        if point:
            self.local_data['range'] = self.bge_object.getDistanceTo(point)
        else:
            self.local_data['range'] = float('inf')
