import logging; logger = logging.getLogger("morserobots." + __name__)

from morse.builder.creator import RobotCreator
from morse.builder import Robot, WheeledRobot

class Morsy(Robot):
    def __init__(self, name=None):
        Robot.__init__(self, "morsy")
        self.name = name
    def set_color(self, color = (0.0, 0.0, 0.8)):
        """
        Allows to change Morsy's body color.
        """
        self.get_child('morsy_mesh').material_slots['body'].material.diffuse_color = color


class ATRV(Robot):
    def __init__(self, name=None):
        Robot.__init__(self, "atrv")
        self.name = name
        self.properties(classpath = "morse.robots.atrv.ATRVClass")

class B21(Robot):
    def __init__(self, name=None):
        Robot.__init__(self, "b21")
        self.name = name
        self.properties(classpath = "morse.robots.b21.B21Class")

# see data/robots/environment.blend and src/morse/robots/environment.py
class FakeRobot(RobotCreator):
    def __init__(self, name=None):
        RobotCreator.__init__(self, name,
            "morse.robots.environment.EnvironmentClass", "environment")

class Hummer(Robot):
    def __init__(self, name=None):
        Robot.__init__(self, "hummer")
        self.name = name
        self.properties(classpath = "morse.robots.hummer.HummerClass", \
                        brakes = 0.0, friction = 200.0, force = 0.0, \
                        steer = 0.0, init = 0, cid = 0)

class Jido(Robot):
    def __init__(self, name=None):
        Robot.__init__(self, "jido")
        self.name = name
        self.properties(classpath = "morse.robots.jido.JidoClass")

# see human.py
#class MocapHuman(Robot):
#    def __init__(self, name="Human"):
#        Robot.__init__(self, "mocap_human")
#        self.name = name
#        self.properties(classpath = "morse.robots.mocap_human.MocapHumanClass",\
#                        Sensitivity = 0.001, Speed = 0.01, \
#                        DraggedObject = "", move_cameraFP = True)
#        # Mouse Game Logic

class Pioneer3DX(WheeledRobot):
    def __init__(self, name=None):
        WheeledRobot.__init__(self, "pioneer3dx")
        self.name = name
        self.properties(classpath = "morse.robots.pioneer3dx.Pioneer3DXClass", \
                        HasSuspension = False, HasSteering = False, \
                        Influence = 0.1, Friction = 0.8, \
                        WheelFLName = "Wheel_L", WheelFRName = "Wheel_R", \
                        WheelRLName = "None", WheelRRName = "None", \
                        CasterWheelName = "CasterWheel")

class QUAD2012(Robot):
    def __init__(self, name=None):
        Robot.__init__(self, "quadrotor")
        self.name = name
        self.properties(classpath = "morse.robots.quadrotor.QuadrotorClass")
        # Collision - Motion Game Logic

class Quadrotor(Robot):
    def __init__(self, name=None):
        Robot.__init__(self, "quadrotor_dynamic")
        self.name = name
        self.properties(classpath = "morse.robots.quadrotor.QuadrotorClass")

class RMax(Robot):
    def __init__(self, name=None):
        Robot.__init__(self, "rmax")
        self.name = name
        self.properties(classpath = "morse.robots.rmax.RmaxClass")

class SegwayRMP400(WheeledRobot):
    def __init__(self, name=None):
        WheeledRobot.__init__(self, "segwayrmp400")
        self.name = name
        self.properties(classpath = "morse.robots.segwayrmp400.SegwayRMP400PhysicsClass", \
                        HasSuspension = False, HasSteering = False, \
                        Influence = 0.1, Friction = 0.8, FixTurningSpeed = 0.0, \
                        WheelFLName = "wheel1", WheelFRName = "wheel2", \
                        WheelRLName = "wheel3", WheelRRName = "wheel4")

class Submarine(Robot):
    def __init__(self, name=None):
        Robot.__init__(self, "submarine")
        self.name = name
        self.properties(classpath = "morse.robots.submarine.SubmarineClass")
        # Keys Game Logic

class Victim(Robot):
    def __init__(self, name=None):
        Robot.__init__(self, "victim")
        self.name = name
        self.properties(classpath = "morse.robots.victim.VictimClass", \
                        Victim_Tag = True, Requirements = "1,2,3", \
                        Injured = True, Severity = 10)
