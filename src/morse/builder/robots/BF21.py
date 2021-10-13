from morse.builder import *
from morse.builder.actuators import Hydrodynamics
from morse.builder.actuators import MotionVW
from morse.modifiers.abstract_modifier import AbstractModifier
from morse.builder.actuators import Hydrodynamics
from morse.builder.sensors import Pose
from morse.builder.sensors import IMU
from morse.builder.sensors import Battery
from morse.builder.sensors import DVL
from morse.builder.sensors import GPS

class Bf21(GroundRobot):
    """
    A template robot model for BF21, with an articulated tailcone.
    """
    def __init__(self, name = None):

        # BF21.blend is located in the data/robots directory
        GroundRobot.__init__(self, 'BF21', name)
        self.properties(classpath = "morse.robots.BF21.Bf21")

        ###################################
        # Actuator components
        ###################################

        # AUV hydrodynamics
        self.control = Hydrodynamics()
        self.append(self.control) 

        ###################################
        # Sensor components
        ###################################

        self.pose = Pose() 
        self.append(self.pose)
        self.pose.alter('','morse.modifiers.PoseMod.Bluefin21PoseModifier')
            
        self.imu = IMU()
        self.append(self.imu)
        self.imu.alter('','morse.modifiers.IMUMod.Bluefin21IMUModifier')

        self.bat = Battery()
        self.append(self.bat)

        self.dvl = DVL()
        self.append(self.dvl)

        self.gps = GPS()
        self.gps.level('raw')
        self.append(self.gps)

    ###################################
    # Comms functions
    ###################################
    # This function sets the communications streams for various devices
    def set_moos(self, moos_host='127.0.0.1', moos_port=9000, moos_name='iMorse'):

        self.control.add_stream('moos','morse.middleware.moos.thruster.TailconeCtrlReader',
            moos_host=moos_host, moos_port=moos_port, moos_name=moos_name)

        self.pose.add_stream('moos',
            moos_host=moos_host, moos_port=moos_port, moos_name=moos_name)

        self.imu.add_stream('moos',
            moos_host=moos_host, moos_port=moos_port, moos_name=moos_name)

        self.bat.add_stream('moos',
            moos_host=moos_host, moos_port=moos_port, moos_name=moos_name)

        self.dvl.add_stream('moos','morse.middleware.moos.DVLCtrl.DVLNotifier',
            moos_host=moos_host, moos_port=moos_port, moos_name=moos_name)

        self.gps.add_stream('moos',
            moos_host=moos_host, moos_port=moos_port, moos_name=moos_name)
            # GPSRawNotifier

    def set_ros(self, namespace=""):
        
        self.gps.add_stream('ros', 
            frame_id=namespace+self.name+"gps_frame" )
        self.imu.add_stream('ros', 
            frame_id=namespace+self.name+"imu_frame" )
        self.control.add_stream('ros')



    # This function sets the frequencies of some devices
    def frequency(self, frequency=None):
        GroundRobot.frequency(frequency)
        self.control.frequency(frequency)
        self.pose.frequency(frequency)
        self.imu.frequency(frequency)
        self.dvl.frequency(frequency)
        self.gps.frequency(1)
        self.bat.frequency(1)
