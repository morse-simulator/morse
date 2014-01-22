import logging; logger = logging.getLogger("morse." + __name__)
import morse.core.sensor
from morse.helpers.components import add_data, add_property
import morse.helpers.geo_math as geo_math

from math import atan2, sqrt, degrees
from random import gauss, random

def point_from_xy(lat_degs, lon_degs, x, y):
    """
    Calculate a lat/lon starting from given lat/lon, and moving a given distance x,y.
    Cartesian coordinates x,y are converted to polar coordinates and the destination
    lat/lon is calculated using those. Positive y is north.
    """
    distance = sqrt(x**2 + y**2)
    heading_rads = atan2(x,y) # +y is north
    return geo_math.point_from_distance_and_heading(lat_degs, lon_degs, distance, heading_rads)

class GPS(morse.core.sensor.Sensor):
    """
    This sensor emulates a GPS, providing the exact coordinates in the
    Blender scene. The coordinates provided by the GPS are with respect
    to the origin of the Blender coordinate reference.
    """

    _name = "GPS"

    add_data('x', 0.0, "float",
             'x coordinate of the sensor, in world coordinate, in meter')
    add_data('y', 0.0, "float",
             'y coordinate of the sensor, in world coordinate, in meter')
    add_data('z', 0.0, "float",
             'z coordinate of the sensor, in world coordinate, in meter')

    add_data('has_fix', True, "bool", 'True if the GPS has a fix.')
    add_data('lat', 0.0, "float", 'Latitude in degrees.')
    add_data('lon', 0.0, "float", 'Longitude in degrees.')
    add_data('alt', 0.0, "float", 'Altitude in meters.')
    add_data('speed', 0.0, "float", 'Speed over ground (speed in the x-y plane), in meters-per-second.')
    add_data('heading', 0.0, "float", 'Heading (in the x-y plane) in degrees.')

    add_property('origin_lat', 0, 'origin_lat', "float", 'Latitude of the GPS Lat/Lon that corresponds to x=0,y=0,z=0.')
    add_property('origin_lon', 0, 'origin_lon', "float", 'Longitude of the GPS Lat/Lon that corresponds to x=0,y=0,z=0.')

    def __init__(self, obj, parent=None):
        """ Constructor method.
            Receives the reference to the Blender object.
            The second parameter should be the name of the object's parent. """
        logger.info('%s initialization' % obj.name)
        # Call the constructor of the parent class
        morse.core.sensor.Sensor.__init__(self, obj, parent)

        self.prev_x, self.prev_y = 0,0

        logger.info('Component initialized, runs at %.2f Hz', self.frequency)

    def default_action(self):
        """ Main function of this component. """
        x = self.position_3d.x
        y = self.position_3d.y
        z = self.position_3d.z

        self.local_data['x'] = x
        self.local_data['y'] = y
        self.local_data['z'] = z

        dx = x - self.prev_x
        dy = y - self.prev_y

        lat,lon = point_from_xy(self.origin_lat, self.origin_lon, x, y)

        self.local_data['has_fix'] = True
        self.local_data['lat'] = lat
        self.local_data['lon'] = lon
        self.local_data['alt'] = z
        self.local_data['speed'] = sqrt(dx**2 + dy**2)*self.frequency
        self.local_data['heading'] = -degrees(self.position_3d.yaw)

        self.prev_x,self.prev_y = x,y

