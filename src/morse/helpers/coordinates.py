import logging; logger = logging.getLogger("morse." + __name__)
from math import sqrt, cos, sin, tan, atan, atan2, radians, degrees, pi
from morse.core import blenderapi
from morse.helpers.morse_math import normalise_angle
import numpy

class CoordinateConverter:
    """ Allow to convert coordinates from Geodetic to LTP to ECEF-r ... """
    A  = 6378137.0 # WGS-84 Earth semi-major axis
    F = 1 / 298.257223563 # WGS-84 flattening
    ECC = 8.181919191e-2 # first excentricity
    R = 6378137.0 # Radius of Earth at equator
    A2 = A**2
    ECC2 = ECC**2
    ECC4 = ECC**4

    _instance = None
    
    def __init__(self, latitude, longitude, altitude, angle_east_blender_x):
        P = [radians(longitude), radians(latitude), altitude]
        self.origin_ecef = self.geodetic_to_ecef(numpy.matrix(P))
        _rot = \
          [[-sin(P[0]), cos(P[0]), 0],
           [-cos(P[0]) * sin(P[1]), -sin(P[1]) * sin(P[0]), cos(P[1])],
           [cos(P[1]) * cos(P[0]), cos(P[1]) * sin(P[0]), sin(P[1])]]
        self._rot_ltp_ecef = numpy.matrix(_rot)
        self._rot_ecef_ltp = self._rot_ltp_ecef.T
        _rot_east_x = \
         [[cos(angle_east_blender_x), -sin(angle_east_blender_x), 0],
          [sin(angle_east_blender_x), cos(angle_east_blender_x),  0],
          [0, 0, 1]]
        self._angle_east = angle_east_blender_x
        self._rot_blender_ltp = numpy.matrix(_rot_east_x )
        self._rot_ltp_blender = self._rot_blender_ltp.T

    @staticmethod
    def instance():
        if not CoordinateConverter._instance:
            try:
                ssr = blenderapi.getssr()
                latitude = ssr["latitude"]
                longitude = ssr["longitude"]
                altitude = ssr["altitude"]
                try:
                    angle_against_east = radians(ssr["angle_against_north"]) - pi / 2
                except KeyError as e:
                    angle_against_east = 0.0
                CoordinateConverter._instance = \
                    CoordinateConverter(latitude, longitude, altitude, 
                                        angle_against_east)
            except KeyError as e:
                logger.error("Missing environment parameter %s\n", e)

        return CoordinateConverter._instance

    def geodetic_to_ecef(self, P):

        """
        converts gps-data(radians) to ECEF-r coordinates
        """
        lg = P[0, 0]
        la = P[0, 1]
        h =  P[0, 2]
        N = self.A/sqrt(1-(self.ECC2*(sin(la)**2)))
        return numpy.matrix([
               (h + N)*cos(la)*cos(lg),
               (h + N)*cos(la)*sin(lg),
               (h + (1 - self.ECC2) * N)*sin(la)])

    def ltp_to_ecef(self, xt):
        """
        converts point in LTP(Blender) to ECEF-r coordinates
        """
        return  self.origin_ecef + xt * self._rot_ltp_ecef   #transformed xt -> xe
    
    def ecef_to_ltp(self, xt):
        """
        converts point in ECEF-r coordinates to LTP(Blender)
        """
        return (xt - self.origin_ecef) * self._rot_ecef_ltp


    def ecef_to_geodetic(self, xe):
        """
        converts point in ECEF-r coordinates into Geodetic (GPS) via
        Vermeille's method
        """
        x = xe[0, 0]
        y = xe[0, 1]
        z = xe[0, 2]
        #"just intermediary parameters" see FoIz
        p = (x**2+y**2)/self.A2
        q = (1-self.ECC2)/self.A2*z**2
        r = (p+q-self.ECC4)/6
        s = self.ECC4 * (p*q)/(4*r**3)
        t = (1+s+sqrt(s*(2+s)))**(1/3.0)
        u = r*(1+t+1/t)
        v = sqrt(u**2+(self.ECC4*q))
        w = self.ECC2*((u+v-q)/(2*v))
        k = sqrt(u+v+w**2)-w
        D = (k*(sqrt(x**2+y**2)))/(k+self.ECC2)
        gps_coords = numpy.matrix([
                      2*atan(y/(x+(sqrt(x**2+y**2)))),
                      2*atan(z/(D+sqrt(D**2+z**2))),
                     ((k+self.ECC2-1)/k)*sqrt(D**2+z**2)])
        return gps_coords

    def ltp_to_geodetic(self, xe):
        return self.ecef_to_geodetic(self.ltp_to_ecef(xe))

    def geodetic_to_ltp(self, xe):
        return self.ecef_to_ltp(self.geodetic_to_ecef(xe))

    def geodetic_to_geocentric(self, lat, h):
        """ Convert geodetic latitude to geocentric latitude

        :param: latitude geodetic latitude in degree
        :param: h height against sea level in meter
        :return: geocentric latitude in degree
        """
        lat_rad = radians(lat)
        lat_surface = atan((1 - self.F)**2 *tan(lat_rad))
        sin_lat = sin(lat_rad)
        cos_lat = cos(lat_rad)
        s1 = h * sin_lat + self.R * sin(lat_surface)
        cc = h * cos_lat + self.R * cos(lat_surface)
        lat_geoc = atan(s1 / cc)
        return degrees(lat_geoc)

    def geocentric_to_ecef(self, xt):
        longitude = xt[0, 0]
        latitude = xt[0, 1]
        radius =  xt[0, 2]
        clon = cos(longitude)
        slon = sin(longitude)
        clat = cos(latitude)
        slat = sin(latitude)
        return radius * numpy.matrix([
            clat * clon,
            clat * slon,
            slat])

    def ecef_to_geocentric(self, xt):
        x = xt[0, 0]
        y = xt[0, 1]
        z = xt[0, 2]
        longitude = atan2(y, x)
        nxy = sqrt(x * x + y * y)
        latitude = atan2(z, nxy);
        radius = sqrt(x * x + y * y + z * z)
        return numpy.matrix([longitude, latitude, radius])

    def blender_to_ltp(self, xt):
        return xt * self._rot_blender_ltp

    def ltp_to_blender(self,  xt):
        return xt * self._rot_ltp_blender

    def angle_against_geographic_north(self, orientation):
        """
        Return the angle against geographic_north, as returned by a compass, i.e.
        between [0, 2 pi], clockwise
        """
        res =  pi / 2 - (orientation[2] - self._angle_east)
        while res < 0:
            res += 2 * pi
        return res

