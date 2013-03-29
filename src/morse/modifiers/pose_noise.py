import logging; logger = logging.getLogger("morse." + __name__)
import random
from math import radians, degrees, cos
import mathutils

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
    
    add_property('_pos_std_dev', 0.05, "pos_std", type = "float", 
                 doc = "Standard deviation for position noise")
    add_property('_rot_std_dev', radians(5), "rot_std", type = "float", 
                 doc = "Standard deviation for rotation noise")
    add_property('_2D', False, "_2D", type = "bool", 
                 doc = "If True, noise is only applied to 2D pose attributes (i.e., x, y and yaw)")
    
    def initialize(self):
        self._pos_std_dev = float(self.parameter("pos_std", default=0.05))
        self._rot_std_dev = float(self.parameter("rot_std", default=radians(5)))
        self._2D = bool(self.parameter("_2D", default=False))
        logger.info("Noise modifier standard deviations: position %.4f, rotation %.4f deg"
                    % (self._pos_std_dev, degrees(self._rot_std_dev)))

class PositionNoiseModifier(NoiseModifier):
    """ Add a gaussian noise to a position 
    """
    def modify(self):
        try:
            data_vars = ['x', 'y']
            if not self._2D:
                data_vars.append('z')
            for variable in data_vars:
                self.data[variable] = random.gauss(self.data[variable], self._pos_std_dev)
        except KeyError as detail:
            self.key_error(detail)

class OrientationNoiseModifier(NoiseModifier):
    """ Add a gaussian noise to an orientation 
    """
    def modify(self):
        # generate a gaussian noise rotation vector
        rot_vec = mathutils.Vector((0.0, 0.0, 0.0))
        for i in range(0, 3):
            rot_vec[i] = random.gauss(rot_vec[i], self._rot_std_dev)
        # convert rotation vector to a quaternion representing the random rotation
        angle = rot_vec.length
        if angle > 0:
            axis = rot_vec / angle
            noise_quat = mathutils.Quaternion(axis, angle)
        else:
            noise_quat = mathutils.Quaternion()
            noise_quat.identity()
        try:
            self.data['orientation'] = (noise_quat * self.data['orientation']).normalized()
        except KeyError:
            # for eulers this is a bit crude, maybe should use the noise_quat here as well...
            try: 
                data_vars = ['yaw']
                if not self._2D:
                    data_vars.append('roll')
                    data_vars.append('pitch')
                for variable in data_vars:
                    self.data[variable] = random.gauss(self.data[variable], self._rot_std_dev)
            except KeyError as detail:
                self.key_error(detail)

class PoseNoiseModifier(PositionNoiseModifier, OrientationNoiseModifier):
    """ Add a gaussian noise to both position and orientation 
    """
    def modify(self):
        PositionNoiseModifier.modify(self)
        OrientationNoiseModifier.modify(self)
