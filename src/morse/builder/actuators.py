from morse.builder.creator import ActuatorCreator
from morse.builder.blenderobjects import *
from morse.builder import Actuator

class Destination(ActuatorCreator):
    def __init__(self, name=None):
        ActuatorCreator.__init__(self, name, \
                                 "morse.actuators.destination.Destination",\
                                 "destination")

class ForceTorque(ActuatorCreator):
    def __init__(self, name=None):
        ActuatorCreator.__init__(self, name, \
                                 "morse.actuators.force_torque.ForceTorque",\
                                 "force_torque")

class MocapControl(ActuatorCreator):
    def __init__(self):
        ActuatorCreator.__init__(self)
        self.properties(classpath="morse.actuators.mocap_control.MocapControl")

# Gripper uses Actuator from morse.builder
class Gripper(Actuator):
    def __init__(self, name=None):
        Actuator.__init__(self, "gripper")
        self.name = name
        self.properties(classpath = "morse.actuators.gripper.Gripper")
        self.properties(Angle = 60.0, Distance = 0.5)
    def properties(self, **kwargs):
        radar = self._bpy_object.game.sensors["Radar"]
        if 'Angle' in kwargs:
            radar.angle = kwargs['Angle']
        if 'Distance' in kwargs:
            radar.distance = kwargs['Distance']
        Actuator.properties(self, **kwargs)


class Keyboard(ActuatorCreator):
    def __init__(self, name=None):
        ActuatorCreator.__init__(self, name, \
                                 "morse.actuators.keyboard.Keyboard",\
                                 "keyboard")
        self.properties(Speed = 1.0)
        obj = bpymorse.get_context_object()
        # replace Always sensor by Keyboard sensor
        sensor = obj.game.sensors[-1]
        sensor.type = 'KEYBOARD'
        # need to get the new Keyboard Sensor object
        sensor = obj.game.sensors[-1]
        sensor.use_pulse_true_level = True
        sensor.use_all_keys = True

# kuka_lwr uses Actuator from morse.builder
class KukaLWR(Actuator):
    def __init__(self, name=None):
        Actuator.__init__(self, "kuka_lwr")
        self.name = name
        self.properties(classpath = "morse.actuators.armature.Armature")

class Mocap(ActuatorCreator):
    def __init__(self, name=None):
        ActuatorCreator.__init__(self, name, \
                                 "morse.actuators.mocap_control.MocapControl",\
                                 "mocap_control")

class Orientation(ActuatorCreator):
    def __init__(self, name=None):
        ActuatorCreator.__init__(self, name, \
                                 "morse.actuators.orientation.Orientation",\
                                 "orientation")

# pa_10 uses Actuator from morse.builder
class PA10(Actuator):
    def __init__(self, name=None):
        Actuator.__init__(self, "pa_10")
        self.name = name
        self.properties(classpath = "morse.actuators.pa_10.PA10", Speed = 1.0)
        # Sound Game Logic Actuator servo_1.mp3

class PTU(ActuatorCreator):
    def __init__(self, name=None):
        ActuatorCreator.__init__(self, name, \
                                 "morse.actuators.ptu.PTU", "ptu")
        self.properties(Speed = 1.0, Manual = False, Tolerance = 0.01)
        # append PanBase with its logic
        self.append_meshes(['PanBase', 'TiltBase'])

class RotorcraftAttitude(ActuatorCreator):
    def __init__(self, name=None):
        ActuatorCreator.__init__(self, name,
            "morse.actuators.rotorcraft_attitude.RotorcraftAttitude",\
            "rotorcraft_attitude")

class RotorcraftWaypoint(ActuatorCreator):
    def __init__(self, name=None):
        ActuatorCreator.__init__(self, name,
            "morse.actuators.rotorcraft_waypoint.RotorcraftWaypoint",\
            "rotorcraft_waypoint")

class StabilizedQuadrotor(ActuatorCreator):
    def __init__(self, name=None):
        ActuatorCreator.__init__(self, name,
            "morse.actuators.stabilized_quadrotor.StabilizedQuadrotor",\
            "stabilized_quadrotor")

class SteerForce(ActuatorCreator):
    def __init__(self, name=None):
        ActuatorCreator.__init__(self, name, \
                                 "morse.actuators.steer_force.SteerForce",\
                                 "steer_force")

class Teleport(ActuatorCreator):
    def __init__(self, name=None):
        ActuatorCreator.__init__(self, name, \
                                 "morse.actuators.teleport.Teleport",\
                                 "teleport")

class MotionVW(ActuatorCreator):
    def __init__(self, name=None):
        ActuatorCreator.__init__(self, name, \
                                 "morse.actuators.v_omega.MotionVW",\
                                 "v_omega")

class MotionVWDiff(ActuatorCreator):
    def __init__(self, name=None):
        ActuatorCreator.__init__(self, name,
            "morse.actuators.v_omega_diff_drive.MotionVWDiff",
            "v_omega_diff_drive")

class Waypoint(ActuatorCreator):
    def __init__(self, name=None):
        ActuatorCreator.__init__(self, name, \
            "morse.actuators.waypoint.Waypoint", "waypoint")
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
    def __init__(self, name=None):
        ActuatorCreator.__init__(self, name, \
                                 "morse.actuators.xy_omega.MotionXYW",\
                                 "xy_omega")

class Light(ActuatorCreator):
    def __init__(self, name=None):
        ActuatorCreator.__init__(self, name, \
                                 "morse.actuators.light.Light",\
                                 "light")
        light = Spot("LightSpot")
        self.append(light)
        self.properties(emit = True)
