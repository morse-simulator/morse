import logging; logger = logging.getLogger("morse." + __name__)
import random
from math import radians, degrees, cos
from morse.core.mathutils import Vector, Quaternion

from morse.helpers.components import add_property
from morse.modifiers.abstract_modifier import AbstractModifier

class NoiseModifier(AbstractModifier):
    """ 
    This modifier allows to simulate Gaussian noise for pose measurements.
    If the variable ``orientation`` exists, it is taken to be a unit quaternion
    and noise added to it. Otherwise rotational noise will be added to the roll,
    pitch and yaw variables.

    This modifier attempts to alter data ``x``, ``y`` and ``z`` for position, 
    and either ``orientation`` or ``yaw``, ``pitch`` and ``roll`` for orientation. 

    The PoseNoise modifier provides as modifiers:
    
    * :py:class:`morse.modifiers.pose_noise.PositionNoiseModifier`
    * :py:class:`morse.modifiers.pose_noise.OrientationNoiseModifier`
    * :py:class:`morse.modifiers.pose_noise.PoseNoiseModifier`

    """

    _name = "PoseNoise"

    add_property('_pos_std_dev', {'x': 0.05, 'y': 0.05, 'z': 0.05}, "pos_std", type="dict",
                 doc="Standard deviation for position noise as dictionary with x,y,z as floats")
    add_property('_rot_std_dev', {'roll': radians(5), 'pitch': radians(5), 'yaw': radians(5)},
                "rot_std", type="dict",
                 doc="Standard deviation for rotation noise of roll,pitch,yaw axes as floats in radians")
    add_property('_2D', False, "_2D", type="bool",
                 doc="If True, noise is only applied to 2D pose attributes (i.e., x, y and yaw)")

    def initialize(self):
        pos_std = self.parameter("pos_std", default=0.05)
        if isinstance(pos_std, dict):
            self._pos_std_dev = pos_std
        else:
            self._pos_std_dev = {'x': float(pos_std), 'y': float(pos_std), 'z': float(pos_std)}
        rot_std = self.parameter("rot_std", default=radians(5))
        if isinstance(rot_std, dict):
            self._rot_std_dev = rot_std
        else:
            self._rot_std_dev = {'roll': float(rot_std), 'pitch': float(rot_std), 'yaw': float(rot_std)}
        self._2D = bool(self.parameter("_2D", default=False))
        if self._2D:
            logger.info("Noise modifier standard deviations: x:%.4f, y:%.4f, yaw:%.3f deg",
                        self._pos_std_dev.get('x', 0),
                        self._pos_std_dev.get('y', 0),
                        degrees(self._rot_std_dev.get('yaw', 0)))
        else:
            logger.info("Noise modifier standard deviations: x:%.4f, y:%.4f, z:%.4f, "
                        "roll:%.3f deg, pitch:%.3f deg, yaw:%.3f deg",
                        self._pos_std_dev.get('x', 0),
                        self._pos_std_dev.get('y', 0),
                        self._pos_std_dev.get('z', 0),
                        degrees(self._rot_std_dev.get('roll', 0)),
                        degrees(self._rot_std_dev.get('pitch', 0)),
                        degrees(self._rot_std_dev.get('yaw', 0)))

class PositionNoiseModifier(NoiseModifier):
    """ Add a gaussian noise to a position 
    """
    def modify(self):
        data_vars = ['x', 'y']
        if not self._2D:
            data_vars.append('z')
        for variable in data_vars:
            if variable in self.data and variable in self._pos_std_dev:
                self.data[variable] = random.gauss(self.data[variable], self._pos_std_dev[variable])

class OrientationNoiseModifier(NoiseModifier):
    """ Add a gaussian noise to an orientation 
    """
    def modify(self):
        data_vars = []
        if not self._2D:
            data_vars.append('roll')
            data_vars.append('pitch')
        data_vars.append('yaw')
        # generate a gaussian noise rotation vector
        rot_vec = Vector((0.0, 0.0, 0.0))
        for i in range(0, 3):
            if data_vars[i] in self._rot_std_dev:
                rot_vec[i] = random.gauss(rot_vec[i], self._rot_std_dev[data_vars[i]])
        # convert rotation vector to a quaternion representing the random rotation
        angle = rot_vec.length
        if angle > 0:
            axis = rot_vec / angle
            noise_quat = Quaternion(axis, angle)
        else:
            noise_quat = Quaternion()
            noise_quat.identity()
        try:
            self.data['orientation'] = (noise_quat * self.data['orientation']).normalized()
        except KeyError:
            # for eulers this is a bit crude, maybe should use the noise_quat here as well...
            for var in data_vars:
                if var in self.data and var in self._rot_std_dev:
                    self.data[var] = random.gauss(self.data[var], self._rot_std_dev[var])

class PoseNoiseModifier(PositionNoiseModifier, OrientationNoiseModifier):
    """ Add a gaussian noise to both position and orientation 
    """
    def modify(self):
        PositionNoiseModifier.modify(self)
        OrientationNoiseModifier.modify(self)
