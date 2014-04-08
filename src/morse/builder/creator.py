import logging; logger = logging.getLogger("morsebuilder." + __name__)
from morse.builder import AbstractComponent, bpymorse
from morse.core.exceptions import *

class ComponentCreator(AbstractComponent):
    APPEND_EMPTY = 0
    USE_BLEND = 1
    LINK_EXISTING_BLEND = 2

    def __init__(self, cname, category, filename, action, make_morseable):
        """ ComponentCreator constructor

        This class allow to create Python component for MORSE. It consists of an
        Empty object, to which you can then add meshs of your choice. It adds
        automaticaly the logic (Always sensor link to a Python controller). And
        set the default physics_type to 'NO_COLLISION'.

        :param cname: (string) component name (Empty object in Blender scene)
        :param category: (string) in ['actuators', 'sensors', 'robots']
        :param filename: (string, optional) used for the datastream configuration
            name of the Blender file in MORSE_COMPONENTS/category/filename.blend
            see morse.builder.data.MORSE_DATASTREAM_DICT (default: None)
        :param action: Indicate what to do with the filename. Must be one of
            [APPEND_EMPTY, USE_BLEND, LINK_EXISTING_BLEND]
        :param make_morseable: (boolean) Add Morse logic. Make it false
            if you add some blend file which already contains the
            necessary logic.
        :return: a new AbstractComponent instance.
        """
        AbstractComponent.__init__(self, filename=filename, category=category)
        bpymorse.deselect_all()
        if action == ComponentCreator.APPEND_EMPTY:
            bpymorse.add_morse_empty()
        elif action == ComponentCreator.USE_BLEND:
            self.append_meshes()
        elif action == ComponentCreator.LINK_EXISTING_BLEND:
            bpymorse.select_only(bpymorse.get_object(filename))

        obj = bpymorse.get_first_selected_object()
        if cname:
            obj.name = cname
            self.basename = cname
        # no collision by default for components
        obj.game.physics_type = 'NO_COLLISION'
        self.set_blender_object(obj)
        # Add MORSE logic
        if make_morseable:
            self.morseable()

    def parent_root(self, objects):
        # Parent the root objects with this Component
        for child in objects:
            if not child.parent:
                child.matrix_parent_inverse.identity()
                child.parent = self._bpy_object

    def append_meshes(self, objects=None, component=None, prefix=None):
        """ Append the objects to this component

        Overloads :py:meth:`morse.builder.abstractcomponent.AbstractComponent.append_meshes`

        :param objects: list of the objects names to append
        :param component: component in which the objects are located
        :return: list of the imported Blender objects
        """
        imported_objects = AbstractComponent.append_meshes(self, objects,
                                                           component, prefix)
        self.parent_root(imported_objects)

        return imported_objects

    def append_collada(self, component=None):
        """ Append Collada objects to this component

        Overloads :py:meth:`morse.builder.abstractcomponent.AbstractComponent.append_collada`

        :param component: component in which the objects are located
        :return: list of the imported Blender objects
        """
        imported_objects = AbstractComponent.append_collada(self, component)
        self.parent_root(imported_objects)

        return imported_objects

class SensorCreator(ComponentCreator):
    _classpath = None
    _blendname = None

    def __init__(self, name="SensorCreator", action = ComponentCreator.APPEND_EMPTY,
                                            make_morseable = True):
        ComponentCreator.__init__(self, name, 'sensors',
                self.__class__._blendname, action, make_morseable)
        self.properties(Component_Tag = True, classpath = self.__class__._classpath)

class ActuatorCreator(ComponentCreator):
    _classpath = None
    _blendname = None

    def __init__(self, name="ActuatorCreator", action = ComponentCreator.APPEND_EMPTY,
                                               make_morseable = True):
        ComponentCreator.__init__(self, name, 'actuators', 
                self.__class__._blendname, action, make_morseable)
        self.properties(Component_Tag = True, classpath = self.__class__._classpath)

class ArmatureCreator(ComponentCreator):
    _classpath = "morse.actuators.armature.Armature"

    def __init__(self, name = None, armature_name = None, model_name = None):
        """ Initialize an Armature

        :param armature_name: Armature object name
        :param model_name: Armature model name, if any
        """
        
        if not armature_name and not model_name:
            raise MorseBuilderError("You need to specify either the name of " \
                    "an armature or a Blender model in order to create an " \
                    "armature actuator.")

        blendname = None
        action = None
        if model_name:
            blendname = model_name
            action = ComponentCreator.USE_BLEND
        else:
            blendname = armature_name
            action = ComponentCreator.LINK_EXISTING_BLEND

        ComponentCreator.__init__(self, name, 'actuators',  blendname, action, False)
        self.properties(Component_Tag = True, classpath = self.__class__._classpath)

        self.ik_targets = []

        # the user may have created IK constraints on the armature, without
        # setting an IK target. In that case, we add such a target
        for bone in self._bpy_object.pose.bones:
            for c in bone.constraints:
                if c.type == 'IK' and c.ik_type == 'DISTANCE':
                    if not c.target:
                        self.create_ik_targets([bone.name])

    def _get_posebone(self, bone_name):
        """ Returns a given PoseBone in the armature.

        If the joint does not exist, throw an exception.
        """
        armature = self._bpy_object

        if bone_name not in [c.name for c in armature.pose.bones]:
            msg = "Joint <%s> does not exist in model %s." % (bone_name, armature.name)
            msg += " Did you add a skeleton to your model in MakeHuman?"
            raise MorseBuilderError(msg)

        return armature.pose.bones[bone_name]

    def create_ik_targets(self, bones):

        # Bug with iTaSC! cf http://developer.blender.org/T37894
        if bpymorse.version() < (2, 70, 0):
            if self._bpy_object.pose.ik_solver == 'ITASC':
                logger.warn("Due to a bug in Blender (T37894), only the standard " \
                            "IK solver can be used with IK targets. Switching " \
                            "from iTaSC to standard IK solver.")
                self._bpy_object.pose.ik_solver = 'LEGACY'

        for target in bones:
            posebone = self._get_posebone(target)
            bpymorse.add_morse_empty("ARROWS")
            empty = bpymorse.get_first_selected_object()
            empty.scale = [0.01, 0.01, 0.01]

            empty.matrix_local = posebone.bone.matrix_local
            empty.location = posebone.bone.tail_local

            existing_ik = [c for c in posebone.constraints if c.type == 'IK']
            if len(existing_ik) == 1:
                ik_constraint = existing_ik[0]
            elif existing_ik:
                raise MorseBuilderError("Bone %s has several IK constraints." \
                        "MORSE supports only one IK constraint per bone. Please " \
                        "remove other ones.")
            else:
                ik_constraint = posebone.constraints.new("IK")

            ik_constraint.ik_type = "DISTANCE"
            ik_constraint.use_rotation = True
            ik_constraint.use_tail = True
            ik_constraint.target = empty

            self.ik_targets.append((empty, target))


    def after_renaming(self):
        for empty, target in self.ik_targets:
            empty.name = "ik_target." + self.name + "." + target




# helpers

def get_properties_str(name):
    """ Returns the Game properties of the Blender object represented by the name

    get_properties_str('Sick') gives
    laser_range = 30.0, Component_Tag = True, scan_window = 180.0, 
    Visible_arc = True, resolution = 0.25,
    classpath = 'morse.sensors.sick.LaserScannerClass'
    """
    obj = bpymorse.get_object(name)
    properties_dictionary = get_properties(obj)
    return ", ".join(["%s = %s"%(pname, properties_dictionary[pname]) \
                      for pname in properties_dictionary])

def get_properties(obj):
    import json
    properties_dictionary = {}
    properties = obj.game.properties
    for name in properties.keys():
        properties_dictionary[name] = properties[name].value
        if properties[name].type == 'TIMER':
            properties_dictionary[name] = "timer(%f)"%properties_dictionary[name]
        elif type(properties_dictionary[name]) is str:
            properties_dictionary[name] = json.dumps(properties_dictionary[name])
    return properties_dictionary
