# ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ Mission Systems Pty Ltd ~ ~ ~ ~ ~ ~ ~ ~ ~ ~
# This file is subject to the terms and conditions defined in
# file 'LICENSE', which is part of this source code package.
# - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
# Project: WamV-Morse-Sim
# - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
# Primary Author:
# david battle <david.battle@missionsystems.com.au>
# Other Author(s):
# none
# - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
# Date Created:
# 29/01/2019
# - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

from morse.builder import *
# from wamv_sim.builder.actuators import Buoyancy
#from wamv_sim.builder.actuators import Dynamics
#from wamv_sim.builder.actuators import Fixedthrusters
from morse.builder.actuators import WamVHydrodynamics
from morse.builder.actuators import Fixedthrusters
from morse.builder.sensors import *

class Wamv(GroundRobot):
    """
    A template robot model for wamv, with thruster control and a pose sensor.
    """
    def __init__(self, name = None, debug = True):

        # wamv.blend is located in the data/robots directory
        # GroundRobot.__init__(self, 'wamv_sim/robots/wamv.blend',name,blender_object_name='platform_boundary')
        GroundRobot.__init__(self, 'wamv',name,blender_object_name='platform_boundary')
        self.properties(classpath = "morse.robots.wamv.Wamv")

        ###################################
        # Actuators
        ###################################

        # Buoyancy forces
        # self.buoyancy = Buoyancy()
        # self.append(self.buoyancy)

        # Hydrodynamic forces
        self.dynamics = WamVHydrodynamics()
        self.append(self.dynamics)

        # Fixed differential thrusters
        self.thrusters = Fixedthrusters()
        self.append(self.thrusters)
        self.thrusters.add_stream('moos','morse.middleware.moos.thruster.CtrlReader')

        ###################################
        # Sensors
        ###################################

        self.pose = Pose()
        self.append(self.pose)
        self.pose.alter('','morse.modifiers.PoseMod.WamVPoseModifier')
        self.pose.add_stream('moos')

        self.imu = IMU()
        self.append(self.imu)
        self.imu.alter('','morse.modifiers.IMUMod.WamVIMUModifier')
        self.imu.add_stream('moos')

        self.DVL = DVL()
        self.append(self.DVL)
        self.DVL.add_stream('moos','morse.middleware.moos.DVLCtrl.DVLNotifier')

        self.GPS = GPS()
        self.GPS.level('raw')
        self.append(self.GPS)
        self.GPS.add_stream('moos')

        self.bat = Battery()
        self.append(self.bat)
        self.bat.add_stream('moos')
        self.bat.frequency(10)

        # NOTE: Adding cameras makes the GLSL water renderer flash annoyingly!

        # self.rear_camera = VideoCamera()
        # self.append(self.rear_camera)
        # self.rear_camera.properties(cam_far = 10000.0, cam_width = 800, cam_height = 600)
        # self.rear_camera.translate(-8,0,1)
        # self.rear_camera.rotate(0,-.2,0)

        # self.side_camera = VideoCamera()
        # self.append(self.side_camera)
        # self.side_camera.properties(cam_far = 10000.0, cam_width = 800, cam_height = 600)
        # self.side_camera.translate(0.6,10,1.5)
        # self.side_camera.rotate(-1.6,-.2,0)
