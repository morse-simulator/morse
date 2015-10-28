import logging; logger = logging.getLogger("morse." + __name__)
from morse.builder.data import MORSE_COMPONENTS
from morse.core import mathutils
import morse.core.sensor
from morse.helpers.components import add_data, add_property
from morse.helpers.coordinates import CoordinateConverter

from math import degrees
import datetime
import os

import numpy

def _decimal_date(date):
    bisextile = (date.year % 4 == 0 and date.year % 100 != 0) or (date.year % 400 == 0)
    days_month = [0, 31, 28, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31]
    if bisextile:
        days_month[2] += 1
    day_nb = date.day
    for i in range(0, date.month):
        day_nb += days_month[i]
    return date.year +  (day_nb - 1.0) / (365.0 + bisextile)

class MagnetoDriver(object):
    def __init__(self, date = None):
        from morse.sensors._magnetometer import Magnetometer as Mag
        self._mag = Mag(os.path.join(MORSE_COMPONENTS, 'WMM.COF'))
        self._coord_conv = CoordinateConverter.instance()
        if date:
            self._date = date
        else:
            self._date = _decimal_date(datetime.date.today())

    def compute(self, pose):
        pos = numpy.matrix(pose.translation)
        pos_ltp = self._coord_conv.blender_to_ltp(pos)
        pos_lla = self._coord_conv.ltp_to_geodetic(pos_ltp)
        (decl, incl, f, h, x, y, z) = self._mag.compute(
                                 degrees(pos_lla[0, 0]),
                                 degrees(pos_lla[0, 1]),
                                 pos_lla[0, 2] / 1000.0, self._date)
        mag_field = mathutils.Vector((x, y, z))
        return mag_field * pose.rotation_matrix

class Magnetometer(morse.core.sensor.Sensor):
    """ 
    This sensor computes the magnetic field vector, at the sensor
    position. It relies on the WMM2015 model, available 
    `at NOAA <http://www.ngdc.noaa.gov/geomag/WMM/DoDWMM.shtml>`_.

    .. warning::

        For proper computation, the sensor needs a real position on
        Earth, and so the following properties should be added at the
        environment level:
            - **longitude** in degrees (double) of Blender origin
            - **latitude** in degrees (double) of Blender origin
            - **altitude** in m  of the Blender origin
            - optionnaly **angle_against_north** in degrees is the angle
              between geographic north and the blender X axis.
              **angle_against_north** is positive when the blender X-axis is
              east of true north, and negative when it is to the west.
    """

    _name = "Magnetometer"

    add_data('x', 0.0, "float",
             'Northern component of the magnetic field vector, in nT')
    add_data('y', 0.0, "float",
             'Eastern component of the magnetic field vector, in nT')
    add_data('z', 0.0, "float",
             'Downward component of the magnetic field vector, in nT')

    add_property('date', None, 'date', 'float', 'the date used to adjust \
            for magnetic field. If not precised, consider the today \
            date')

    def __init__(self, obj, parent=None):
        """ Constructor method.

        Receives the reference to the Blender object.
        The second parameter should be the name of the object's parent.
        """
        logger.info('%s initialization' % obj.name)
        # Call the constructor of the parent class
        morse.core.sensor.Sensor.__init__(self, obj, parent)
        self._mag = MagnetoDriver(self.date)

        logger.info('Component initialized, runs at %.2f Hz', self.frequency)


    def default_action(self):
        mag_field = self._mag.compute(self.position_3d)
        self.local_data['x'] = mag_field[0]
        self.local_data['y'] = mag_field[1]
        self.local_data['z'] = mag_field[2]
