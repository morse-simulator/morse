import logging; logger = logging.getLogger("morse." + __name__)
import morse.core.sensor
from morse.helpers.components import add_data, add_property

from math import sin, cos, asin, atan2, sqrt, degrees, radians

earth_radius = 6371000.0

# from http://www.movable-type.co.uk/scripts/latlong.html
def distance_and_direction(lat_start_degs, lon_start_degs, lat_finish_degs, lon_finish_degs):
    if (lat_start_degs == lat_finish_degs and lon_start_degs == lon_finish_degs):
        return 0, 0

    lat_s = radians(lat_start_degs)
    lon_s = radians(lon_start_degs)
    lat_f = radians(lat_finish_degs)
    lon_f = radians(lon_finish_degs)
    lat_d = lat_f - lat_s
    lon_d = lon_f - lon_s

    R = earth_radius

    t1 = sin(lat_d/2)
    t2 = sin(lon_d/2)
    distance = R*2*asin(sqrt(t1*t1 + cos(lat_s)*cos(lat_f)*t2*t2))

    y = sin(lon_d)*cos(lat_f)
    x = cos(lat_s)*sin(lat_f) - sin(lat_s)*cos(lat_f)*cos(lon_d)
    heading = degrees(atan2(y, x))

    return distance, heading

def point_from_distance_and_heading(lat_degs, lon_degs, distance, heading_rads):
    lat_s = radians(lat_degs)
    lon_s = radians(lon_degs)
    t = heading_rads
    d = distance
    R = earth_radius

    lat_f = asin(sin(lat_s)*cos(d/R) + cos(lat_s)*sin(d/R)*cos(t))
    lon_f = lon_s + atan2(sin(t)*sin(d/R)*cos(lat_s), cos(d/R) - sin(lat_s)*sin(lat_f))

    return degrees(lat_f), degrees(lon_f)

def point_from_xy(lat_degs, lon_degs, x, y):
    distance = sqrt(x**2 + y**2)
    heading_rads = atan2(x,y) # +y is north
    return point_from_distance_and_heading(lat_degs, lon_degs, distance, heading_rads)

class GPS(morse.core.sensor.Sensor):
    """
    This sensor emulates a GPS, providing the exact coordinates in the
    Blender scene. The coordinates provided by the GPS are with respect
    to the origin of the Blender coordinate reference.
    """

    _name = "GPS"

    add_data('x', 0.0, "float", \
             'x coordinate of the sensor, in world coordinate, in meter')
    add_data('y', 0.0, "float", \
             'y coordinate of the sensor, in world coordinate, in meter')
    add_data('z', 0.0, "float", \
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
        super(self.__class__, self).__init__(obj, parent)

        self.prev_x = 0
        self.prev_y = 0
        self.prev_lat = None
        self.prev_lon = None

        logger.info('Component initialized, runs at %.2f Hz', self.frequency)


    def default_action(self):
        """ Main function of this component. """
        x = self.position_3d.x
        y = self.position_3d.y
        z = self.position_3d.z

        # Store the data acquired by this sensor that could be sent
        #  via a middleware.
        self.local_data['x'] = x
        self.local_data['y'] = y
        self.local_data['z'] = z

        if self.prev_lat is None and self.prev_lon is None:
            self.prev_lat = self.origin_lat
            self.prev_lon = self.origin_lon

        dx = x - self.prev_x
        dy = y - self.prev_y

        # dfo means distance from origin.
        dfo_xy = sqrt(x**2 + y**2)

        # when far away from the origin, calculate the lat/lon relative to the previous lat/lon
        if dfo_xy < 100:
            lat,lon = point_from_xy(self.origin_lat, self.origin_lon, x, y)
        else:
            lat,lon = point_from_xy(self.prev_lat, self.prev_lon, dx, dy)

        # check that the x,y and lat/lon from origin agree.
        dfo_latlon, hfo_latlon = distance_and_direction(self.origin_lat, self.origin_lon, lat, lon)
        dfo_error = abs(dfo_latlon - dfo_xy)
        assert(dfo_error < 0.1)
        #logger.info("GPS dfo error: %f" % dfo_error)

        speed = sqrt(dx**2 + dy**2)*self.frequency

        self.local_data['has_fix'] = True
        self.local_data['lat'] = lat
        self.local_data['lon'] = lon
        self.local_data['alt'] = z
        self.local_data['speed'] = speed
        self.local_data['heading'] = -degrees(self.position_3d.yaw)

        self.prev_x = x
        self.prev_y = y
        self.prev_lat = lat
        self.prev_lon = lon
