import logging; logger = logging.getLogger("morse." + __name__)
import morse.core.sensor
from morse.helpers.components import add_property, add_data, add_level
import math, time
from morse.core import mathutils
from morse.core import blenderapi
from morse.helpers.coordinates import CoordinateConverter
import numpy

class GPS(morse.core.sensor.Sensor):
    """
    A GPS sensor which returns the position either in Blender or Geodetic coordinates.

    This sensor always provides perfect data on the levels "raw" and "extended".
    To obtain more realistic readings, it is recommended to add modifiers.

    - **Noise modifier**: Adds random Gaussian noise to the data

    coordinates in Blender: :math:`x` -> east and :math:`y` -> north

    The "heading" is Clockwise (mathematically negative).

    .. warning::

        To work properly in "raw" and "extented" mode, you need to
        configure the following variables at the environment level:
            - **longitude** in degrees (double) of Blender origin
            - **latitude** in degrees (double) of Blender origin
            - **altitude** in m  of the Blender origin
            - optionnaly **angle_against_north** in degrees is the angle
              between geographic north and the blender X axis.
              **angle_against_north** is positive when the blender X-axis is
              east of true north, and negative when it is to the west.

    Conversion of Geodetic coordinates into ECEF-r, LTP into ECEF-r and vice versa
    ------------------------------------------------------------------------------

    Conversion of Geodetic coordinates into ECEF-r
    ++++++++++++++++++++++++++++++++++++++++++++++

    To be able to simulate a GPS-sensor :math:`P` (the Blender origin) must
    be defined in the properties in Geodetic coordinates (longitude,
    latitude, altitude).  For the transformation [Psas_] the
    coordinates must be in decimal degrees (no North, minutes,
    etc.). The result is a point :math:`x_0` in the ``ECEF-r`` coordinates.


    Conversion of ECEF-r into LTP[Psas_]
    ++++++++++++++++++++++++++++++++++++

    For this conversion :math:`x_0` is the base. A point :math:`x_e` is given
    in the ``ECEF-r`` coordinates and the goal is to get :math:`x_t` (:math:`=
    x_e` in the ``LTP``-coordinates).

    .. image:: ../../../media/conversion_coordinates.png

    1. Transform :math:`P` (Blender origin, geodetic coordinates
    (stored in the properties)) into :math:`x0` (geocentric (``ECEF-r``)
    coordinates)

    2. Calculate :math:`R_{te}[1]` with longitude, latitude and altitude;
    matrix is the rotation part of the transformation

    3. Transform :math:`x_e` into :math:`x_t` with :math:`x_t = R_{te} * (x_e-x_0)`


    Conversion of LTP into ECEF-r
    +++++++++++++++++++++++++++++

    Known: :math:`P` in Geodetic coordinates (→ :math:`x_0` in ``ECEF-r``) and
    :math:`x_t` in ``LTP``-coordinates

    Goal: :math:`x_e` (:math:`= x_t` in ``ECEF-r`` coordinates)

    Based on the transformation described above the transformation is
    calculated with the transposed matrix :math:`R_{te}`: :math:`x_e = x_0 +
    (R_{te})' * x_t` [Psas_]

    Conversion of ECEF-r into Geodetic coordinates
    ++++++++++++++++++++++++++++++++++++++++++++++

    The last transformation is from ``ECEF-r`` coordinates into Geodetic
    coordinates.  This transformation is calculated with the Vermeille's method
    [FoIz_].  The result is the point :math:`x_e` in "GPS-coordinates" in
    radians.

    Sources
    +++++++

    .. _FoIz: 

     "3.4 Vermeille's Method(2002)" in
     "Comparative Analysis of the Performance of Iterative and
     Non-iterative Solutions to the Cartesian to Geodetic Coordinate
     Transformation", Hok Sum Fok and H. Bâki Iz,
     http://www.lsgi.polyu.edu.hk/staff/zl.li/Vol_5_2/09-baki-3.pdf

    .. _Psas:

     "Conversion of Geodetic coordinates to the Local Tangent
     Plane", Version 2.01,
     http://psas.pdx.edu/CoordinateSystem/Latitude_to_LocalTangent.pdf
    """

    _name = "GPS"
    
    _short_desc = "A GPS sensor that returns coordinates ."

    add_level("simple", None,
              doc = "simple GPS: only current position in Blender is exported",
              default = True)
    add_level("raw", "morse.sensors.gps.RawGPS",
              doc = "raw GPS: position in Geodetic coordinates and velocity \
                      are exported")
    add_level("extended", "morse.sensors.gps.ExtendedGPS",
              doc = "extended GPS: adding information to fit a standard \
                      GPS-sentence")

    add_data('x', 0.0, "float",
             'x coordinate of the sensor, in world coordinate, in meter',
             level = "simple")
    add_data('y', 0.0, "float",
             'y coordinate of the sensor, in world coordinate, in meter',
             level = "simple")
    add_data('z', 0.0, "float",
             'z coordinate of the sensor, in world coordinate, in meter',
             level = "simple")
    add_data('longitude', 0.0, "double",
             'longitude in degree [-180°,180] or [0°,360°]', level = ["raw", "extended"])
    add_data('latitude', 0.0, "double",
             'latitude in degree [-90°,90°]', level = ["raw", "extended"])
    add_data('altitude', 0.0, "double",
             'altitude in m a.s.l.', level = ["raw", "extended"])
    add_data('velocity', [0.0, 0.0, 0.0], "vec3<float>",
             'Instantaneous speed along East, North, Up in meter sec^-1', level = ["raw", "extended"])
    add_data('date', 0000000, "DDMMYY",
             'current date in DDMMYY-format', level = "extended")
    add_data('time', 000000, "HHMMSS",
             'current time in HHMMSS-format', level = "extended")
    add_data('heading', 0, "float",
             'heading in degrees [0°,360°] to geographic north',
             level = "extended")


    def __init__(self, obj, parent=None):
        """ Constructor method.
            Receives the reference to the Blender object.
            The second parameter should be the name of the object's parent. """
        logger.info('%s initialization' % obj.name)
        # Call the constructor of the parent class
        morse.core.sensor.Sensor.__init__(self, obj, parent)

        logger.info('Component initialized, runs at %.2f Hz', self.frequency)


    def default_action(self):
        """
        Main function of this component.
        """
        x = self.position_3d.x
        y = self.position_3d.y
        z = self.position_3d.z

        # Store the data acquired by this sensor that could be sent
        #  via a middleware.
        self.local_data['x'] = float(x)
        self.local_data['y'] = float(y)
        self.local_data['z'] = float(z)

class RawGPS(GPS):
    """
    This sensor emulates a GPS, providing the exact coordinates in the
    Blender scene. The coordinates provided by the GPS are with respect
    to the origin of the Blender coordinate reference.
    """

    def __init__(self, obj, parent=None):
        """ Constructor method.
            Receives the reference to the Blender object.
            The second parameter should be the name of the object's parent. """
        # Call the constructor of the parent class
        GPS.__init__(self, obj, parent)
        
        # Variables to store the previous LTP position
        self.pltp = None
        self.v = [0.0, 0.0, 0.0]

        self.coord_converter = CoordinateConverter.instance()
    
    def default_action(self):
        """
        Calculates speed and GPS-position

        Configurations are the GPS-values for the Blenderorigin
        Transforms point from LTP to Geodetic coordinates

        Refer to:
        - Conversion of Geodetic coordinates to the Local Tangent Plane,
          Version 2.01,
          http://psas.pdx.edu/CoordinateSystem/Latitude_to_LocalTangent.pdf
        - Comparative Analysis of the Performance of Iterative and Non-iterative
          Solutions to the Cartesian to Geodetic Coordinate Transformation, 
          Hok Sum Fok and H.   Bâki Iz,
          http://www.lsgi.polyu.edu.hk/staff/zl.li/Vol_5_2/09-baki-3.pdf
        """
        # Call the default_action of the parent class
        GPS.default_action(self)

        #current position
        xt = numpy.matrix(self.position_3d.translation)
        ltp = self.coord_converter.blender_to_ltp(xt)
        if self.pltp is not None:
            v = (ltp - self.pltp) * self.frequency
            self.v = [v[0, 0], v[0, 1], v[0, 2]]
        self.pltp = ltp
        gps_coords = self.coord_converter.ltp_to_geodetic(ltp)

        #compose message as close as possible to a GPS-standardprotocol
        self.local_data['longitude'] = math.degrees(gps_coords[0, 0])
        self.local_data['latitude'] = math.degrees(gps_coords[0, 1])
        self.local_data['altitude'] = gps_coords[0, 2]
        self.local_data['velocity'] = self.v

class ExtendedGPS(RawGPS):
    """
    Additional information to fit a standard GPS-sentence
    """
    def __init__(self, obj, parent=None):
        """
        Constructor method.

        Receives the reference to the Blender object.
        The second parameter should be the name of the object's parent. 
        """
        # Call the constructor of the parent class
        RawGPS.__init__(self, obj, parent)

    def default_action(self):
        """
        Adds additional information (date, time and heading) to the
        message of the RawGPS
        """
        # Call the default_action of the parent class
        RawGPS.default_action(self)
        current_time = time.gmtime(blenderapi.persistantstorage().time.time)
        date = time.strftime("%d%m%y", current_time)
        time_h_m_s = time.strftime("%H%M%S", current_time)
        if abs(self.v[0]) < 1e-6 and abs(self.v[1]) < 1e-6:
            # Not observable if we do not move
            self.local_data['heading'] = float("inf")
        else:
            heading = self.coord_converter.angle_against_geographic_north(self.position_3d.euler)
            self.local_data['heading'] = math.degrees(heading)
        self.local_data['date'] = date
        self.local_data['time'] = time_h_m_s   

