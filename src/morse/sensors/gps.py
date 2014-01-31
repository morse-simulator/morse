import logging; logger = logging.getLogger("morse." + __name__)
import morse.core.sensor
from morse.helpers.components import add_property, add_data, add_level
import math, time
from morse.core import mathutils
from morse.core import blenderapi

class GPS(morse.core.sensor.Sensor):
    """
    A GPS sensor which returns the position either in Blender or Geodetic coordinates.

    This sensor always provides perfect data on the levels "raw" and "extended".
    To obtain more realistic readings, it is recommended to add modifiers.

    - **Noise modifier**: Adds random Gaussian noise to the data

    coordinates in Blender: :math:`x` -> east and :math:`y` -> north

    The "heading" is Clockwise (mathematically negative).


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
             'Instantaneous speed in X, Y, Z, in meter sec^-1', level = ["raw", "extended"])
    add_data('date', 0000000, "DDMMYY",
             'current date in DDMMYY-format', level = "extended")
    add_data('time', 000000, "HHMMSS",
             'current time in HHMMSS-format', level = "extended")
    add_data('heading', 0, "float",
             'heading in degrees [0°,360°] to geographic north',
             level = "extended")

    add_property('longitude', 0.0, 'longitude', 'double',
                 'longitude in degree [-180°,180°] or [0°,360°] of the \
                  Blender origin')
    add_property('latitude', 0.0, 'latitude', 'double',
             'latitude in degree [-90°,90°] of the Blender origin')
    add_property('altitude', 0.0, 'altitude', 'double',
             'altitude in m a.s.l. of the Blender origin')

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
        
        ##copied from accelerometer
        # Variables to store the previous position
        self.ppx = 0.0
        self.ppy = 0.0
        self.ppz = 0.0
        # Variables to store the previous velocity
        self.pvx = 0.0
        self.pvy = 0.0
        self.pvz = 0.0
        # Make a new reference to the sensor position
        self.p = self.bge_object.position
        self.v = [0.0, 0.0, 0.0] # Velocity
        self.pv = [0.0, 0.0, 0.0] # Previous Velocity

    
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

        ####
        #Speed
        ####
        ##copied from accelerometer
        # Compute the difference in positions with the previous loop
        self.dx = self.p[0] - self.ppx
        self.dy = self.p[1] - self.ppy
        self.dz = self.p[2] - self.ppz

        # Store the position in this instant
        self.ppx = self.p[0]
        self.ppy = self.p[1]
        self.ppz = self.p[2]

        # Scale the speeds to the time used by Blender
        self.v[0] = self.dx * self.frequency
        self.v[1] = self.dy * self.frequency
        self.v[2] = self.dz * self.frequency

        # Update the data for the velocity
        self.pvx = self.v[0]
        self.pvy = self.v[1]
        self.pvz = self.v[2]


        ####
        #GPS
        ####

        ####
        #constants in calculations
        #a: WGS-84 Earth semimajor axis
        #ecc: first eccentricity
        ####
        a  = float(6378137)
        ecc = 8.181919191e-2

        def convert_GPS_to_ECEF(P):
            """
            converts gps-data(radians) to ECEF-r coordinates
            """
            N = a/math.sqrt(1-(ecc**2*(math.sin(P[1])**2)))
            h = P[2]
            x0 = [ (h + N)*math.cos(P[1])*math.cos(P[0]),
                   (h + N)*math.cos(P[1])*math.sin(P[0]),
                   (h + (1 - ecc**2) * N)*math.sin(P[1])]
            return x0

        def convert_LTP_to_ECEF(P):
            """
            converts point in LTP(Blender) to ECEF-r coordinates
            """
            x0 = convert_GPS_to_ECEF(P) #P->x0
            x0 = mathutils.Vector(x0)
            transform_matrix = [[-math.sin(P[0]), math.cos(P[0]), 0],
               [-math.cos(P[0]) * math.sin(P[1]), 
                -math.sin(P[1])*math.sin(P[0]), math.cos(P[1])],
               [math.cos(P[1])*math.cos(P[0]), math.cos(P[1])*math.sin(P[0]), math.sin(P[1])]]
            transform_matrix = mathutils.Matrix(transform_matrix)
            transform_matrix.invert()
            xe = x0 + transform_matrix*xt  #transformed xt -> xe
            return xe

        def vermeille_method(xe):
            """
            converts point in ECEF-r coordinates into Geodetic (GPS) via
            Vermeille's method
            """
            #"just intermediary parameters" see FoIz
            p = (xe[0]**2+xe[1]**2)/a**2
            q = (1-ecc**2)/a**2*xe[2]**2
            r = (p+q-ecc**4)/6
            s = ecc**4 * (p*q)/(4*r**3)
            t = (1+s+math.sqrt(s*(2+s)))**(1/3.0)
            u = r*(1+t+1/t)
            v = math.sqrt(u**2+(ecc**4*q))
            w = ecc**2*((u+v-q)/(2*v))
            k = math.sqrt(u+v+w**2)-w
            D = (k*(math.sqrt(xe[0]**2+xe[1]**2)))/(k+ecc**2)
            gps_coords = [2*math.atan(xe[1]/(xe[0]+(math.sqrt(xe[0]**2+xe[1]**2)))),
                          2*math.atan(xe[2]/(D+math.sqrt(D**2+xe[2]**2))),
                         ((k+ecc**2-1)/k)*math.sqrt(D**2+xe[2]**2)]
            return gps_coords

        #P -> Blender origin in Geodetic coordinates
        P = [self.longitude, self.latitude, self.altitude]

        #current position
        xt = self.position_3d.translation
        xt = mathutils.Vector(xt)

        #P (in degrees) to radians
        for i in range(len(P)-1):
            P[i] = math.radians(P[i])

        ####
        #GPS -> ECEF-r
        ####
        xe = convert_LTP_to_ECEF(P)

        ####
        #ECEF-r -> GPS
        ####
        gps_coords = vermeille_method(xe)

        #gps_coords (in radians) to degrees
        for i in range(len(gps_coords)-1):
            gps_coords[i] = math.degrees(gps_coords[i])


        #compose message as close as possible to a GPS-standardprotocol
        self.local_data['longitude'] = gps_coords[0]
        self.local_data['latitude'] = gps_coords[1]
        self.local_data['altitude'] = gps_coords[2]
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
        current_time = time.gmtime(blenderapi.persistantstorage().current_time)
        date = time.strftime("%d%m%y", current_time)
        time_h_m_s = time.strftime("%H%M%S", current_time)
        heading = (2*math.pi - math.atan2(self.dy, self.dx) + math.pi/2)%(2*math.pi)
        self.local_data['date'] = date
        self.local_data['time'] = time_h_m_s   
        self.local_data['heading'] = math.degrees(heading)

