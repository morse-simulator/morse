import logging
logger = logging.getLogger("morse." + __name__)

from morse.core import blenderapi

def _param_id(_id, axis):
    """
    An helper function to compute the right index to pass to setParam

    :param axis: should be one of ['X', 'Y', 'Z']
    """
    return _id + ord(axis) - ord('X')

class Joint6DoF(object):
    def __init__(self, obj1, obj2, pos_pivot =[0.0, 0.0, 0.0],
                                   rot_pivot =[0.0, 0.0, 0.0],
                                   may_collide =False):
        """ 
        Construct a 6DoF joint between obj1 and obj2. By default, all
        axis are locked and should be explicitly unlocked

        :param obj1: the first physical object to link
        :param obj2: the second physical object to link
        :param pos_pivot: the position of the pivot, relative to obj1
        frame. Default to (0.0, 0.0, 0.0), i.e. obj1's center.
        :param rot_pivot: the rotation of the pivot frame, related to
        obj1 frame. Defaults to (0.0,0.0,0.0), ie aligned with obj1's
        orientation.
        :param bool may_collide: indicates if collisions should be
        enabled or not between the two linked rigid-bodies
        """
        self._joint = blenderapi.constraints().createConstraint(
                obj1.getPhysicsId(),
                obj2.getPhysicsId(),
                12, # 6DoF
                pos_pivot[0], pos_pivot[1], pos_pivot[2],
                rot_pivot[0], rot_pivot[1], rot_pivot[2],
                0 if may_collide else 128)
        for i in range(0, 5):
            self._lock_axis(i)

    def _lock_axis(self, i):
        self._joint.setParam(i, 0.0, 0.0)

    def _free_axis(self, i):
        self._joint.setParam(i, 1.0, 0.0)

    def _limit_axis(self, i, min_value, max_value):
        self._joint.setParam(i, min_value, max_value)

    def lock_translation_dof(self, axis):
        self._lock_axis(_param_id(0, axis))

    def free_translation_dof(self, axis):
        self._free_axis(_param_id(0, axis))

    def limit_translation_dof(self, axis, min_value, max_value):
        self._limit_axis(_param_id(0, axis), min_value, max_value)

    def lock_rotation_dof(self, axis):
        self._lock_axis(_param_id(3, axis))

    def free_rotation_dof(self, axis):
        self._free_axis(_param_id(3, axis))

    def limit_rotation_dof(self, axis, min_value, max_value):
        self._limit_axis(_param_id(3, axis), min_value, max_value)

    def linear_velocity(self, axis, velocity):
        self._joint.setParam(_param_id(6, axis), velocity, 300.0)

    def angular_velocity(self, axis, velocity):
        self._joint.setParam(_param_id(9, axis), velocity, 300.0)

    def linear_spring(self, axis, spring, damping):
        self._joint.setParam(_param_id(12, axis), spring, damping)

    def angular_spring(self, axis, spring, damping):
        self._joint.setParam(_param_id(15, axis), spring, damping)

