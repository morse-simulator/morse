import logging; logger = logging.getLogger("morse." + __name__)
from morse.core import mathutils
from math import sqrt, cos, sin, atan, radians
from morse.core import blenderapi

class CoordinateConverter:
    """ Allow to convert coordinates from Geodetic to LTP to ECEF-r ... """
    A  = float(6378137) # WGS-84 Earth semi-major axi
    ECC = 8.181919191e-2 # first excentricity
    A2 = A**2
    ECC2 = ECC**2
    ECC4 = ECC**4

    _instance = None
    
    def __init__(self, latitude, longitude, altitude):
        self.origin_geodetic = [radians(longitude),
                                radians(latitude),
                                altitude ]
        P = self.origin_geodetic # alias to ease formula
        self.origin_ecef = self.geodetic_to_ecef(P)
        _rot = \
          [[-sin(P[0]), cos(P[0]), 0],
           [-cos(P[0]) * sin(P[1]), -sin(P[1]) * sin(P[0]), cos(P[1])],
           [cos(P[1]) * cos(P[0]), cos(P[1]) * sin(P[0]), sin(P[1])]]
        self._rot_ecef_ltp = mathutils.Matrix(_rot)
        self._rot_ltp_ecef = self._rot_ecef_ltp.transposed()

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
        N = self.A/sqrt(1-(self.ECC2*(sin(P[1])**2)))
        h = P[2]
        return mathutils.Vector((
               (h + N)*cos(P[1])*cos(P[0]),
               (h + N)*cos(P[1])*sin(P[0]),
               (h + (1 - self.ECC2) * N)*sin(P[1])))

    def ltp_to_ecef(self, xt):
        """
        converts point in LTP(Blender) to ECEF-r coordinates
        """
        return  self.origin_ecef + self._rot_ltp_ecef * xt  #transformed xt -> xe
    
    def ecef_to_ltp(self, xt):
        """
        converts point in ECEF-r coordinates to LTP(Blender)
        """
        return self._rot_ecef_ltp * (xt - self.origin_ecef)


    def ecef_to_geodetic(self, xe):
        """
        converts point in ECEF-r coordinates into Geodetic (GPS) via
        Vermeille's method
        """
        #"just intermediary parameters" see FoIz
        p = (xe[0]**2+xe[1]**2)/self.A2
        q = (1-self.ECC2)/self.A2*xe[2]**2
        r = (p+q-self.ECC4)/6
        s = self.ECC4 * (p*q)/(4*r**3)
        t = (1+s+sqrt(s*(2+s)))**(1/3.0)
        u = r*(1+t+1/t)
        v = sqrt(u**2+(self.ECC4*q))
        w = self.ECC2*((u+v-q)/(2*v))
        k = sqrt(u+v+w**2)-w
        D = (k*(sqrt(xe[0]**2+xe[1]**2)))/(k+self.ECC2)
        gps_coords = [2*atan(xe[1]/(xe[0]+(sqrt(xe[0]**2+xe[1]**2)))),
                      2*atan(xe[2]/(D+sqrt(D**2+xe[2]**2))),
                     ((k+self.ECC2-1)/k)*sqrt(D**2+xe[2]**2)]
        return gps_coords

    def ltp_to_geodetic(self, xe):
        return self.ecef_to_geodetic(self.ltp_to_ecef(xe))
