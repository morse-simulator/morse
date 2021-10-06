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

#from morse.builder import *
from morse.builder import GroundRobot
from morse.builder.actuators import WamVHydrodynamics
from morse.builder.actuators import Fixedthrusters
from morse.builder.sensors import IMU
from morse.builder.sensors import DVL
from morse.builder.sensors import GPS
from morse.builder.sensors import Pose
from morse.builder.sensors import Battery
from morse.builder.sensors import Odometry

class Wamv(GroundRobot):
    """
    A template robot model for wamv, with thruster control and a pose sensor.
    """
    def __init__(self, name = None, debug = True):

        # wamv.blend is located in the data/robots directory
        GroundRobot.__init__(self, 'robots/wamv.blend',name,blender_object_name='platform_boundary')
        self.properties(classpath = "morse.robots.wamv.Wamv")

        ###################################
        # Actuators
        ###################################

        # Hydrodynamic forces
        self.dynamics = WamVHydrodynamics()
        self.append(self.dynamics)

        # Fixed differential thrusters
        self.control = Fixedthrusters()
        self.append(self.control)

        ###################################
        # Sensors
        ###################################

        self.pose = Pose()
        self.append(self.pose)
        self.pose.alter('','morse.modifiers.PoseMod.WamVPoseModifier')

        self.imu = IMU()
        self.append(self.imu)
        self.imu.alter('','morse.modifiers.IMUMod.WamVIMUModifier')

        self.dvl = DVL()
        self.append(self.dvl)

        self.gps = GPS()
        self.gps.level('raw')
        self.append(self.gps)

        self.bat = Battery()
        self.append(self.bat)
        self.bat.frequency(1)

        self.odom = Odometry()
        self.append(self.odom)


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

    # This function sets the communications streams for various devices
    def set_moos(self, moos_host='127.0.0.1', moos_port=9000, moos_name='iMorse'):

        self.control.add_stream('moos','morse.middleware.moos.thruster.FixedCtrlReader',
            moos_host=moos_host, moos_port=moos_port, moos_name=moos_name)

        self.dvl.add_stream('moos','morse.middleware.moos.DVLCtrl.DVLNotifier',
            moos_host=moos_host, moos_port=moos_port, moos_name=moos_name)
        
        self.gps.add_stream('moos',
            moos_host=moos_host, moos_port=moos_port, moos_name=moos_name)

        self.pose.add_stream('moos',
            moos_host=moos_host, moos_port=moos_port, moos_name=moos_name)

        self.imu.add_stream('moos',
            moos_host=moos_host, moos_port=moos_port, moos_name=moos_name)

        self.bat.add_stream('moos',
            moos_host=moos_host, moos_port=moos_port, moos_name=moos_name)

        self.odom.add_stream('moos',
            moos_host=moos_host, moos_port=moos_port, moos_name=moos_name)

    def set_ros(self, namespace=""):

        self.gps.add_stream('ros', 
            frame_id=namespace+self.name+"gps_frame" )
        self.imu.add_stream('ros', 
            frame_id=namespace+self.name+"imu_frame" )
        self.odom.add_stream('ros', 
            frame_id=namespace+self.name+"odom_frame" )
        self.control.add_stream('ros')

    # This function sets the frequencies of some devices
    def frequency(self, frequency=None):
        GroundRobot.frequency(frequency)
        self.control.frequency(frequency)
        self.dynamics.frequency(frequency)
        self.imu.frequency(frequency)
        self.pose.frequency(frequency)
        self.dvl.frequency(frequency)
        self.pose.frequency(frequency)
        self.gps.frequency(frequency)
