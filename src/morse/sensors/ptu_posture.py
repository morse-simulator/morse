import logging; logger = logging.getLogger("morse." + __name__)
import re

from morse.core import blenderapi
import morse.core.sensor
from morse.helpers.components import add_data

class PTUPosture(morse.core.sensor.Sensor):
    """
    Simple sensor that provides the current rotation angles of the *pan* and *tilt*
    segments of the :doc:`PTU actuator <../actuators/ptu>`.
    The angles returned are in radians in the range (-pi, pi).

    .. note::

        This sensor **must** be added as a child of the PTU
        you want to sense, like in the example below:

        .. code-block:: python

            robot = ATRV()

            ptu = PTU()
            robot.append(ptu)
            ptu.translate(z=0.9)

            ptu = PTUPosture('ptu_pose')
            ptu.append(ptu_pose)

    .. note:: The angles are given with respect to the orientation of the robot
    
    :sees: :doc:`PTU actuator <../actuators/ptu>`.
    """
    _name = "PTU Pose Sensor"
    _short_desc = "Returns the pan/tilt values of a pan-tilt unit"

    add_data('pan', 0.0, "float","pan value, in radians")
    add_data('tilt', 0.0, "float","tilt value, in radians")
 
    def __init__(self, obj, parent=None):
        """ Constructor method.

        Receives the reference to the Blender object.
        The second parameter should be the name of the object's parent.
        """
        logger.info('%s initialization' % obj.name)
        # Call the constructor of the parent class
        morse.core.sensor.Sensor.__init__(self, obj, parent)
        
        ptu = self._get_ptu(self.bge_object)
        if not ptu:
            logger.error("The PTU pose sensor has not been parented to a PTU! " + \
                    "This sensor must be a child of a PTU. Check you scene.")
            return

        self._ptu_obj = blenderapi.persistantstorage().componentDict[ptu.name]

        self.local_data['pan'] = 0.0
        self.local_data['tilt'] = 0.0
        logger.info('Component <%s> initialized, runs at %.2f Hz' % (self.bge_object.name, self.frequency))

    def _get_ptu(self, obj):
        """
        Retrieve the associated PTU actuator

        Need to carefully deal with possible renaming scheme from Blender,
        in the case of multiples PTU in the scene.
        """
        regexp_ = "^PanBase(\.[0-9]{3})?$"
        regexp = re.compile(regexp_)
        if len([c for c in obj.children if re.match(regexp, c.name)]) > 0:
            return obj
        elif not obj.parent:
            return None
        else:
            return self._get_ptu(obj.parent)


    def default_action(self):
        """ Read the rotation of the platine unit """
        # Find the actual PTU unit as my child
        if not self._ptu_obj:
            return

        # Update the postition of the base platforms
        current_pan, current_tilt = self._ptu_obj.get_pan_tilt()
        logger.debug("Platine: pan=%.4f, tilt=%.4f" % (current_pan, current_tilt))
        
        # Store the data acquired by this sensor that could be sent
        #  via a middleware.
        self.local_data['pan'] = float(current_pan)
        self.local_data['tilt'] = float(current_tilt)
