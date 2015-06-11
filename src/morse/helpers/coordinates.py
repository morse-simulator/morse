import logging; logger = logging.getLogger("morse." + __name__)
from math import sqrt, cos, sin, tan, atan, radians, degrees
from morse.core import blenderapi
import numpy

class CoordinateConverter:
    """ Allow to convert coordinates from Geodetic to LTP to ECEF-r ... """
    A  = 6378137.0 # WGS-84 Earth semi-major axis
    B = 6356752.3142 # Second semi-major axis
    ECC = 8.181919191e-2 # first excentricity
    A2 = A**2
    ECC2 = ECC**2
    ECC4 = ECC**4

    _instance = None
    
    def __init__(self, latitude, longitude, altitude):
        P = [radians(longitude), radians(latitude), altitude]
        self.origin_ecef = self.geodetic_to_ecef(numpy.matrix(P))
        _rot = \
          [[-sin(P[0]), cos(P[0]), 0],
           [-cos(P[0]) * sin(P[1]), -sin(P[1]) * sin(P[0]), cos(P[1])],
           [cos(P[1]) * cos(P[0]), cos(P[1]) * sin(P[0]), sin(P[1])]]
        self._rot_ltp_ecef = numpy.matrix(_rot)
        self._rot_ecef_ltp = self._rot_ltp_ecef.T

    @staticmethod
    def instance():
        if not CoordinateConverter._instance:
            try:
                ssr = blenderapi.getssr()
                latitude = ssr["latitude"]
                longitude = ssr["longitude"]
                altitude = ssr["altitude"]
                CoordinateConverter._instance = \
                    CoordinateConverter(latitude, longitude, altitude)
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
