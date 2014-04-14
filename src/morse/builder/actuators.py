import logging; logger = logging.getLogger("morsebuilder." + __name__)
from morse.builder.creator import ComponentCreator, ActuatorCreator
from morse.builder.blenderobjects import *

class Destination(ActuatorCreator):
    _classpath = "morse.actuators.destination.Destination"

    def __init__(self, name=None):
        ActuatorCreator.__init__(self, name)

class ForceTorque(ActuatorCreator):
    _classpath = "morse.actuators.force_torque.ForceTorque"
    def __init__(self, name=None):
        ActuatorCreator.__init__(self, name)

class MocapControl(ActuatorCreator):
    _classpath = "morse.actuators.mocap_control.MocapControl"

    def __init__(self):
        ActuatorCreator.__init__(self)

# Gripper uses Actuator from morse.builder
class Gripper(ActuatorCreator):
    _classpath = "morse.actuators.gripper.Gripper"
    _blendname = "gripper"

    def __init__(self, name=None):
        ActuatorCreator.__init__(self, name,
                    action = ActuatorCreator.USE_BLEND,
                    make_morseable = False)
        self.properties(Angle = 60.0, Distance = 0.5)
    def properties(self, **kwargs):
        radar = self._bpy_object.game.sensors["Radar"]
        if 'Angle' in kwargs:
            radar.angle = kwargs['Angle']
        if 'Distance' in kwargs:
            radar.distance = kwargs['Distance']
        ActuatorCreator.properties(self, **kwargs)


class Keyboard(ActuatorCreator):
    _classpath = "morse.actuators.keyboard.Keyboard"

    def __init__(self, name=None):
        ActuatorCreator.__init__(self, name)
        self.properties(Speed = 1.0)

class Joystick(ActuatorCreator):
    _classpath = "morse.actuators.joystick.Joystick"

    def __init__(self, name=None, index=0):
        """ Create a new Joystick controller

        :param index: Which joystick to use
        :type index:  int in [0, 7], default 0
        """
        ActuatorCreator.__init__(self, name)
        self.properties(Speed = 1.0)
        obj = bpymorse.get_context_object()
        # replace Always sensor by Joystick sensor
        sensor = obj.game.sensors[-1]
        sensor.type = 'JOYSTICK'
        # need to get the new Joystick Sensor object
        sensor = obj.game.sensors[-1]
        sensor.use_pulse_true_level = True
        sensor.joystick_index = index
        sensor.event_type = 'AXIS'
        sensor.use_all_events = True

class Orientation(ActuatorCreator):
    _classpath = "morse.actuators.orientation.Orientation"

    def __init__(self, name=None):
        ActuatorCreator.__init__(self, name)

class PA10(ActuatorCreator):
    _classpath = "morse.actuators.pa_10.PA10"
    _blendname = "pa_10"

    def __init__(self, name=None):
        ActuatorCreator.__init__(self, name,
                action = ComponentCreator.USE_BLEND,
                make_morseable = False)
        self.properties(Speed = 1.0)

class PTU(ActuatorCreator):
    _classpath = "morse.actuators.ptu.PTU"
    _blendname = "ptu"

    def __init__(self, name=None):
        ActuatorCreator.__init__(self, name)
        self.properties(Speed = 1.0, Manual = False, Tolerance = 0.01)
        # append PanBase with its logic
        self.append_meshes(['PanBase', 'TiltBase'])

class RotorcraftAttitude(ActuatorCreator):
    _classpath = "morse.actuators.rotorcraft_attitude.RotorcraftAttitude"

    def __init__(self, name=None):
        ActuatorCreator.__init__(self, name)

class RotorcraftWaypoint(ActuatorCreator):
    _classpath = "morse.actuators.rotorcraft_waypoint.RotorcraftWaypoint"

    def __init__(self, name=None):
        ActuatorCreator.__init__(self, name)

class StabilizedQuadrotor(ActuatorCreator):
    _classpath = "morse.actuators.stabilized_quadrotor.StabilizedQuadrotor"

    def __init__(self, name=None):
        ActuatorCreator.__init__(self, name)

class SteerForce(ActuatorCreator):
    _classpath = "morse.actuators.steer_force.SteerForce"

    def __init__(self, name=None):
        ActuatorCreator.__init__(self, name)

class Teleport(ActuatorCreator):
    _classpath = "morse.actuators.teleport.Teleport"

    def __init__(self, name=None):
        ActuatorCreator.__init__(self, name)

class MotionVW(ActuatorCreator):
    _classpath = "morse.actuators.v_omega.MotionVW"

    def __init__(self, name=None):
        ActuatorCreator.__init__(self, name)

class MotionVWDiff(ActuatorCreator):
    _classpath = "morse.actuators.v_omega_diff_drive.MotionVWDiff"

    def __init__(self, name=None):
        ActuatorCreator.__init__(self, name)

class Waypoint(ActuatorCreator):
    _classpath = "morse.actuators.waypoint.Waypoint"

    def __init__(self, name=None):
        ActuatorCreator.__init__(self, name)
        self.properties(Target = "")
        # append 2 Radar with logic
        self.add_lr_radars()

    def add_lr_radars(self):
        self.add_radar('Radar.L', 'Lcollision', +1)
        self.add_radar('Radar.R', 'Rcollision', -1)
    def add_radar(self, name, collision, fact):
        bpymorse.deselect_all()
        bpymorse.add_object(type='EMPTY')
        # bpymorse.add_empty(type='ARROWS')
        obj = bpymorse.get_context_object()
        obj.name = name
        obj.location = (0.7, fact*0.4, 0.8)
        obj.parent = self._bpy_object
        bpymorse.new_game_property(type='BOOL', name=collision)
        prop = obj.game.properties[-1]
        prop.value = False
        bpymorse.add_sensor(type="RADAR")
        sensor = obj.game.sensors[-1]
        sensor.angle = 5.0
        sensor.distance = 3.0
        sensor.axis = 'XAXIS'
        sensor.use_pulse_true_level = True
        sensor.frequency = 20
        self.radar_set_collision(obj, sensor, 'LOGIC_AND',  collision, True)
        self.radar_set_collision(obj, sensor, 'LOGIC_NAND', collision, False)
    def radar_set_collision(self, obj, sensor, controller_type, collision, value):
        bpymorse.add_controller(type=controller_type)
        controller = obj.game.controllers[-1]
        bpymorse.add_actuator(type='PROPERTY')
        actuator = obj.game.actuators[-1]
        actuator.mode = 'TOGGLE'
        actuator.property = collision
        actuator.mode = 'ASSIGN'
        actuator.value = str(value)
        controller.link(sensor = sensor, actuator = actuator)

class MotionXYW(ActuatorCreator):
    _classpath = "morse.actuators.xy_omega.MotionXYW"

    def __init__(self, name=None):
        ActuatorCreator.__init__(self, name)

class Light(ActuatorCreator):
    _classpath = "morse.actuators.light.Light"
    def __init__(self, name=None):
        self.light = None
        ActuatorCreator.__init__(self, name)
        self.light = Spot("LightSpot")
        self.append(self.light)
        self.properties(Emit=True)
        
    def properties(self, **kwargs):
        ActuatorCreator.properties(self, **kwargs)
        if self.light:
            spot = self.light._bpy_object.data
            if spot.type == 'SPOT':
                if "size" in kwargs.keys():
                    spot.spot_size = kwargs['size']
                if "distance" in kwargs.keys():
                    spot.distance = kwargs['distance']
                if "color" in kwargs.keys():
                    import re
                    spot.color = tuple(int(v) for v in re.findall("[0-9]+", kwargs['color']))

class Sound(ActuatorCreator):
    _classpath = "morse.actuators.sound.Sound"

    def __init__(self, name=None):
        ActuatorCreator.__init__(self, name)
        self.properties(mode="play")
        #self.select()
        bpymorse.add_actuator(type="SOUND", name="MORSE_SOUND")
        actuator = self._bpy_object.game.actuators[-1]
        controller =  self._bpy_object.game.controllers[-1]
        controller.link(actuator=actuator)
    def open(self, filepath):
        # just to raise a FileNotFoundError
        open(filepath).close()
        actuator = self._bpy_object.game.actuators[-1]
        #if bpy.ops.sound.open.poll():
        bpymorse.open_sound(filepath=filepath)
        actuator.sound = bpymorse.get_last_sound()
        actuator.use_sound_3d = True
        actuator.distance_3d_max = 10000.0

class Armature(ActuatorCreator):
    _classpath = "morse.actuators.armature.Armature"

    def __init__(self, name = None, armature_name = None, model_name = None):
        """ Initialize an armature

        Either `armature_name` or `model_name` or both must be specified.
        

        :param armature_name: Armature object name
        :param model_name: Armature model name, if any
        """
        
        if not armature_name and not model_name:
            raise MorseBuilderError("You need to specify either the name of " \
                    "an armature or a Blender model in order to create an " \
                    "armature actuator.")

        if model_name:
            ActuatorCreator.__init__(self, 
                                    name, 
                                    action = ComponentCreator.USE_BLEND,
                                    blendfile = model_name,
                                    blendobject = armature_name,
                                    make_morseable = False)

        else:
            ActuatorCreator.__init__(self, 
                                name, 
                                action = ComponentCreator.LINK_EXISTING_OBJECT,
                                blendobject = armature_name,
                                make_morseable = False)


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

class KukaLWR(Armature):
    """
    This actuator provides a KUKA LWR mesh with the associated kinematic chain.

    An IK target is available on the last join, allowing for cartesian control of
    the arm.

    See :doc:`the general documentation on armatures <./armature>` for details.
    """
    _name = "KUKA LWR"
    _short_desc="7DoF KUKA Lightweight Robotic Arm (LWR)"

    def __init__(self, name=None):
        Armature.__init__(self, name, model_name = "kuka_lwr")
        self.create_ik_targets(["kuka_7"])

# end morse.builder.actuators
