from morse.builder.creator import ActuatorCreator
from morse.builder.blenderobjects import *
from morse.builder import Actuator

class Armature(ActuatorCreator):
    def __init__(self, name="Armature_Controller"):
        ActuatorCreator.__init__(self, name,
            "morse/actuators/armature_actuator", "ArmatureActuatorClass",
            "armature_actuator")

class Destination(ActuatorCreator):
    def __init__(self, name="Motion_Controller"):
        ActuatorCreator.__init__(self, name, "morse/actuators/destination", \
                                 "DestinationActuatorClass", "destination")

class ForceTorque(ActuatorCreator):
    def __init__(self, name="Motion_Controller"):
        ActuatorCreator.__init__(self, name, "morse/actuators/force_torque", \
                                 "ForceTorqueActuatorClass", "force_torque")

# Gripper uses Actuator from morse.builder
class Gripper(Actuator):
    def __init__(self, name="Gripper"):
        Actuator.__init__(self, "gripper")
        self.name = name
        self.properties(Class = "GripperActuatorClass", \
                        Path = "morse/actuators/gripper")

class Keyboard(ActuatorCreator):
    def __init__(self, name="Gripper"):
        ActuatorCreator.__init__(self, name, "morse/actuators/keyboard", \
                                 "KeyboardActuatorClass", "armature_actuator")
        self.properties(Speed = 1.0)
        # replace Always sensor by Keyboard sensor
        sensor = bpy.context.object.game.sensors[-1]
        sensor.type = 'KEYBOARD'
        # need to get the new Keyboard Sensor object
        sensor = bpy.context.object.game.sensors[-1]
        sensor.use_pulse_true_level = True
        sensor.use_all_keys = True

# kuka_lwr uses Actuator from morse.builder
class KukaLWR(Actuator):
    def __init__(self, name="kuka_armature"):
        Actuator.__init__(self, "kuka_lwr")
        self.name = name
        self.properties(Class = "KukaActuatorClass", \
                        Path = "morse/actuators/kuka_lwr")

class Mocap(ActuatorCreator):
    def __init__(self, name="Mocap_Controller"):
        ActuatorCreator.__init__(self, name, "morse/actuators/mocap_control", \
                                 "MocapControlClass", "mocap_control")

class Orientation(ActuatorCreator):
    def __init__(self, name="Motion_Controller"):
        ActuatorCreator.__init__(self, name, "morse/actuators/orientation", \
                                 "OrientationActuatorClass", "orientation")

# pa_10 uses Actuator from morse.builder
class PA10(Actuator):
    def __init__(self, name="PA10"):
        Actuator.__init__(self, "pa_10")
        self.name = name
        self.properties(Class = "PA10ActuatorClass", \
                        Path = "morse/actuators/pa_10", \
                        Speed = 1.0)
        # Sound Game Logic Actuator servo_1.mp3

class PTU(ActuatorCreator):
    def __init__(self, name="Motion_Controller"):
        ActuatorCreator.__init__(self, name, "morse/actuators/ptu", \
                                 "PTUActuatorClass", "ptu")
        self.properties(Speed = 1.0, Manual = False, Tolerance = 0.01)
        # append PanBase with its logic
        self.append_meshes(['PanBase', 'TiltBase'])

class RotorcraftAttitude(ActuatorCreator):
    def __init__(self, name="attitude_controller"):
        ActuatorCreator.__init__(self, name,
            "morse/actuators/rotorcraft_attitude",
            "RotorcraftAttitudeActuatorClass", "rotorcraft_attitude")

class RotorcraftWaypoint(ActuatorCreator):
    def __init__(self, name="Motion_Controller"):
        ActuatorCreator.__init__(self, name,
            "morse/actuators/rotorcraft_waypoint",
            "RotorcraftWaypointActuatorClass", "rotorcraft_waypoint")

class StabilizedQuadrotor(ActuatorCreator):
    def __init__(self, name="quadrotor_UIMU"):
        ActuatorCreator.__init__(self, name,
            "morse/actuators/stabilized_quadrotor",
            "StabilizedQuadrotorActuatorClass", "stabilized_quadrotor")

class SteerForce(ActuatorCreator):
    def __init__(self, name="Motion_Controller"):
        ActuatorCreator.__init__(self, name, "morse/actuators/steer_force", \
                                 "SteerForceActuatorClass", "steer_force")

class Teleport(ActuatorCreator):
    def __init__(self, name="Motion_Controller"):
        ActuatorCreator.__init__(self, name, "morse/actuators/teleport", \
                                 "TeleportActuatorClass", "teleport")

class MotionVW(ActuatorCreator):
    def __init__(self, name="Motion_Controller"):
        ActuatorCreator.__init__(self, name, "morse/actuators/v_omega", \
                                 "VWActuatorClass", "v_omega")

class MotionVWDiff(ActuatorCreator):
    def __init__(self, name="Motion_Controller"):
        ActuatorCreator.__init__(self, name,
            "morse/actuators/v_omega_diff_drive", "VWDiffDriveActuatorClass",
            "v_omega_diff_drive")

class Waypoint(ActuatorCreator):
    def __init__(self, name="Motion_Controller"):
        ActuatorCreator.__init__(self, name,
            "morse/actuators/waypoint",
            "WaypointActuatorClass", "waypoint")
        self.properties(Target = "")
        # append 2 Radar with logic
        self.add_lr_radars()

    def add_lr_radars(self):
        self.add_radar('Radar.L', 'Lcollision', +1)
        self.add_radar('Radar.R', 'Rcollision', -1)
    def add_radar(self, name, collision, fact):
        bpy.ops.object.empty_add(type='ARROWS')
        bpy.context.object.name = name
        bpy.context.object.location = (0.7, fact*0.4, 0.8)
        bpy.context.object.parent = self._blendobj
        bpy.ops.object.game_property_new(type='BOOL', name=collision)
        prop = bpy.context.object.game.properties[-1]
        prop.value = False
        bpy.ops.logic.sensor_add(type="RADAR")
        sensor = bpy.context.object.game.sensors[-1]
        sensor.angle = 5.0
        sensor.distance = 3.0
        sensor.axis = 'XAXIS'
        sensor.use_pulse_true_level = True
        sensor.frequency = 20
        self.radar_set_collision(sensor, 'LOGIC_AND',  collision, True)
        self.radar_set_collision(sensor, 'LOGIC_NAND', collision, False)
    def radar_set_collision(self, sensor, controller_type, collision, value):
        bpy.ops.logic.controller_add(type=controller_type)
        controller = bpy.context.object.game.controllers[-1]
        bpy.ops.logic.actuator_add(type='PROPERTY')
        actuator = bpy.context.object.game.actuators[-1]
        actuator.mode = 'TOGGLE'
        actuator.property = collision
        actuator.mode = 'ASSIGN'
        actuator.value = str(value)
        controller.link(sensor = sensor, actuator = actuator)

class MotionXYW(ActuatorCreator):
    def __init__(self, name="Motion_Controller"):
        ActuatorCreator.__init__(self, name, "morse/actuators/xy_omega", \
                                 "XYWActuatorClass", "xy_omega")

class Light(ActuatorCreator):
    def __init__(self, name="LightAct"):
        ActuatorCreator.__init__(self, name, "morse/actuators/light", \
                                 "LightActuatorClass", "light")
        light = Spot("LightSpot")
        self.append(light)
        self.properties(emit = True)
