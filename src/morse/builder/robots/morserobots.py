import logging; logger = logging.getLogger("morserobots." + __name__)

from morse.builder import Robot, GroundRobot, WheeledRobot

class Morsy(GroundRobot):
    def __init__(self, name=None):
        GroundRobot.__init__(self, "morsy", name)
        self.properties(classpath = "morse.robots.morsy.Morsy")

        self.set_rigid_body()
        mesh = self.get_child('morsy_mesh')
        mesh.game.physics_type = 'NO_COLLISION'

        self._bpy_object.game.radius = 0.08

        self.set_collision_bounds()

    def set_color(self, color = (0.0, 0.0, 0.8)):
        """
        Allows to change Morsy's body color.
        """
        mats = self.get_child('morsy_mesh').material_slots.keys()
        [body_mat] = [mat for mat in mats if mat.startswith('body')] # account for body.001, body.002...
        self.get_child('morsy_mesh').material_slots[body_mat].material.diffuse_color = color

class ATRV(GroundRobot):
    def __init__(self, name=None):
        GroundRobot.__init__(self, "atrv", name)
        self.properties(classpath = "morse.robots.atrv.ATRV")

class B21(GroundRobot):
    def __init__(self, name=None):
        GroundRobot.__init__(self, "b21", name)
        self.properties(classpath = "morse.robots.b21.B21")

        self.set_rigid_body()
        self.set_collision_bounds()

        collision = self.get_child('b21_collision')
        collision.game.physics_type = 'STATIC'

# see src/morse/robots/environment.py
class FakeRobot(Robot):
    def __init__(self, name=None):
        Robot.__init__(self, name = name) # no Blender model -> a simple Empty will be created
        self.properties(classpath = "morse.robots.fakerobot.FakeRobot")
        self.set_no_collision()

class Hummer(GroundRobot):
    def __init__(self, name=None):
        GroundRobot.__init__(self, "hummer", name)
        self.properties(classpath = "morse.robots.hummer.Hummer",
                        brakes = 0.0, friction = 200.0, force = 0.0,
                        steer = 0.0, init = 0, cid = 0)

class Jido(GroundRobot):
    def __init__(self, name=None):
        GroundRobot.__init__(self, "jido", name)
        self.properties(classpath = "morse.robots.jido.Jido")

        self.set_dynamic()
        self.set_collision_bounds()
        self._bpy_object.game.radius = 0.01

        mesh = self.get_child('JidoBase')
        mesh.game.physics_type = 'STATIC'

# see human.py
#class MocapHuman(Robot):
#    def __init__(self, name="Human"):
#        Robot.__init__(self, "mocap_human")
#        self.name = name
#        self.properties(classpath = "morse.robots.mocap_human.MocapHuman",\
#                        Sensitivity = 0.001, Speed = 0.01, \
#                        DraggedObject = "", move_cameraFP = True)
#        # Mouse Game Logic

class Pioneer3DX(WheeledRobot):
    def __init__(self, name=None):
        WheeledRobot.__init__(self, "pioneer3dx", name)
        self.properties(classpath = "morse.robots.pioneer3dx.Pioneer3DX",
                        HasSuspension = False, HasSteering = False,
                        Influence = 0.1, Friction = 0.8,
                        WheelFLName = "Wheel_L", WheelFRName = "Wheel_R",
                        WheelRLName = "None", WheelRRName = "None",
                        CasterWheelName = "CasterWheel", 
                        FixTurningSpeed = 0.52)

class QUAD2012(Robot):
    def __init__(self, name=None):
        Robot.__init__(self, "quadrotor", name)
        self.properties(classpath = "morse.robots.quadrotor.Quadrotor")
        # Collision - Motion Game Logic
        self.set_no_collision()

class Quadrotor(Robot):
    def __init__(self, name=None):
        Robot.__init__(self, "quadrotor_dynamic", name)
        self.properties(classpath = "morse.robots.quadrotor_dynamic.Quadrotor")

class RMax(Robot):
    def __init__(self, name=None):
        Robot.__init__(self, "rmax", name)
        self.properties(classpath = "morse.robots.rmax.RMax",
                        NoGravity = True)

        self.set_rigid_body()
        rotor = self.get_child('Rotor')
        rotor.game.physics_type = 'NO_COLLISION'

class SegwayRMP400(WheeledRobot):
    def __init__(self, name=None):
        WheeledRobot.__init__(self, "segwayrmp400", name)
        self.properties(classpath = "morse.robots.segwayrmp400.SegwayRMP400",
                        HasSuspension = False, HasSteering = False,
                        Influence = 0.1, Friction = 0.8, FixTurningSpeed = 1.16,
                        WheelFLName = "wheel1", WheelFRName = "wheel2",
                        WheelRLName = "wheel3", WheelRRName = "wheel4")

class Submarine(Robot):
    def __init__(self, name=None):
        Robot.__init__(self, "submarine", name)
        self.properties(classpath = "morse.robots.submarine.Submarine",
                        NoGravity = True)
        self.set_rigid_body()
        self.set_collision_bounds()

class Victim(GroundRobot):
    def __init__(self, name=None):
        GroundRobot.__init__(self, "victim", name)
        self.properties(classpath = "morse.robots.victim.Victim",
                        Victim_Tag = True, Requirements = "1,2,3",
                        Injured = True, Severity = 10)


class PatrolBot(WheeledRobot):
    def __init__(self, name=None):
        WheeledRobot.__init__(self, "patrolbot", name)
        self.properties(classpath = "morse.robots.patrolbot.PatrolBot",
                        HasSuspension = False, HasSteering = False,
                        Influence = 0.1, Friction = 0.8,
                        WheelFLName = "Wheel_L", WheelFRName = "Wheel_R",
                        WheelRLName = "None", WheelRRName = "None",
                        CasterWheelName = "CasterWheel")



