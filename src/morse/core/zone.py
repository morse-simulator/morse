import logging
logger = logging.getLogger("morse." + __name__)
from morse.core import mathutils
import morse.core.object
from sys import float_info

class Zone:
    """ 
    Creates a named zone in the 3D environment. This can be used by
    components via the :py:class:morse.core.ZoneManagerto trigger
    specific behaviours when the component is inside the zone.

    The main method is the contains one, which allow to decide if a
    point is part of a zone (and so potentially to trigger some
    behaviour).

    Note that for simplicity, we currently assume the zone axis follows
    the world axis.
    """
    def __init__(self, obj):
        self.name = obj.name
        self.type = obj['Type']
        self.obj = obj

        mesh = obj.meshes[0]

        # Get list of unique vertexes
        vertexes_ = set()
        for v_index in range(24):
            vertex = mesh.getVertex(0, v_index)
            vertexes_.add(vertex.getXYZ()[:])

        vertexes = [mathutils.Vector(p) for p in vertexes_]

        # Compute range
        self._min_values = [float_info.max, float_info.max, float_info.max]
        self._max_values = [-float_info.max, -float_info.max, -float_info.max]

        for i in range(len(vertexes)):
            for j in range(3):
                # XXX Here, we assume there is no rotation
                vertexes[i][j] = vertexes[i][j] * obj.worldScale[j] + obj.worldPosition[j]
                if self._min_values[j] > vertexes[i][j]:
                    self._min_values[j] = vertexes[i][j]
                if self._max_values[j] < vertexes[i][j]:
                    self._max_values[j] = vertexes[i][j]

    def contains(self, pos):
        """
        Verify if a pos (represented by a vector) is contained in the zone

        The implementation assumes that the zone is a rectangle, with
        axis following world axis.
        """
        res = True
        for i in range(3):
            res = res and pos[i] >= self._min_values[i]
            res = res and pos[i] <= self._max_values[i]
        return res


class ZoneManager:
    """
    Handle the different zones, allowing to search for them
    'efficiently' (at least in an abstract way)
    """
    def __init__(self):
        self.all_zones = {}
        self.zones_by_type = {}

    def add(self, obj):
        new_zone = Zone(obj)
        logger.info("Adding zone %s of type %s" % (new_zone.name, new_zone.type))

        self.all_zones[new_zone.name] = new_zone

        zone_type = self.zones_by_type.get(new_zone.type, None)
        if not zone_type:
            self.zones_by_type[new_zone.type] = {}
            zone_type = self.zones_by_type[new_zone.type]
        zone_type[new_zone.name] = new_zone

    def _get_subset(self, name = None, type = None):
        search_list = self.all_zones

        if name:
            if name in self.all_zones:
                search_list = {name: [self.all_zones[name]]}
            else:
                search_list = {}
        if type:
            if type in self.zones_by_type:
                search_list = self.zones_by_type[type]
            else:
                search_list = {}
        return search_list


    def is_in(self, obj_or_pos, name = None, type = None):
        """
        Determine if obj_or_pos is  in a zone

        If :param obj_or_pos: is a :py:class:morse.core.object.Object,
        consider the position of the object. Otherwise, assume it is a
        position.
        If a :param name: is precised, check only if this specific zone
        contains the position
        If a :param type: is precised, only search in the zone of this
        type.
        """
        pos = obj_or_pos
        if (isinstance(obj_or_pos, morse.core.object.Object)):
            pos = obj_or_pos.position_3d.translation

        search_list = self._get_subset(name, type)

        for zone in search_list.values():
            if zone.contains(pos):
                return True

        return False

    def contains(self, obj_or_pos, name = None, type = None):
        """
        Determine which zone(s) contain(s) the position pos

        If :param obj_or_pos: is a :py:class:morse.core.object.Object,
        consider the position of the object. Otherwise, assume it is a
        position.
        If a :param name: is precised, check only if this specific zone
        contains the position
        If a :param type: is precised, only search in the zone of this
        type.

        The method returns the list of zones containing the position,
        considering the previous limitation
        """
        pos = obj_or_pos
        if (isinstance(obj_or_pos, morse.core.object.Object)):
            pos = obj_or_pos.position_3d.translation

        search_list = self._get_subset(name, type)

        res = []

        for zone in search_list.values():
            if zone.contains(pos):
                res.append(zone)

        return res
