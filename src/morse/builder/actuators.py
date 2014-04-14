from morse.builder.creator import ActuatorCreator, ArmatureCreator
from morse.builder.blenderobjects import *

class Destination(ActuatorCreator):
    _classpath = "morse.actuators.destination.Destination"

    def __init__(self, name=None):
        ActuatorCreator.__init__(self, name)

class ForceTorque(ActuatorCreator):
    _classpath = "morse.actuators.force_torque.ForceTorque",
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

class KukaLWR(ArmatureCreator):
    def __init__(self, name=None):
        ArmatureCreator.__init__(self, name, model_name = "kuka_lwr")
        self.create_ik_targets(["kuka_7"])

class Mocap(ActuatorCreator):
    _classpath = "morse.actuators.mocap_control.MocapControl",

    def __init__(self, name=None):
        ActuatorCreator.__init__(self, name)

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

# end morse.builder.actuators
