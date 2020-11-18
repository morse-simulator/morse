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
<<<<<<< HEAD
from morse.builder.sensors import Odometry
=======
>>>>>>> 36b6dd57d9e96fc06c19ee7e66d8ff2e5bb4832f

class Bf21(GroundRobot):
    """
    A template robot model for BF21, with an articulated tailcone.
    """
    def __init__(self, name = None, debug = True, use_sonar = False, use_multiStaticSonar = False, timewarp = 1):

        # BF21.blend is located in the data/robots directory
        GroundRobot.__init__(self, 'BF21', name)
        self.properties(classpath = "morse.robots.BF21.Bf21")

        ###################################
        # Actuators
        ###################################

        # AUV hydrodynamics
        self.control = Hydrodynamics()
        self.append(self.control) 

        ###################################
        # Sensors
        ###################################
        if( use_sonar ):
            port_beam = Sonar()                             
            port_beam.translate(y = 0.3, z = -0.1)
            port_beam.properties(Beam_direction = 180)
            port_beam.properties(Max_range = 25)
            port_beam.properties(Beam_width_azimuth = 0.15)
            port_beam.properties(Beam_width_elevation = 60)
            port_beam.properties(Beam_depression_angle = 20)
            port_beam.add_stream('moos','morse.middleware.moos.SonarCtrl.sonarReader')
            port_beam.add_stream('moos','morse.middleware.moos.SonarCtrl.sonarNotifier')
            port_beam.frequency(30 * timewarp)
            self.append(port_beam)

            stbd_beam = Sonar()
            stbd_beam.translate(y = -0.3, z = -0.1)
            stbd_beam.properties(Beam_direction = 0)
            stbd_beam.properties(Max_range = 25)
            stbd_beam.properties(Beam_width_azimuth = 0.15)
            stbd_beam.properties(Beam_width_elevation = 60)
            stbd_beam.properties(Beam_depression_angle = 20)
            stbd_beam.add_stream('moos','morse.middleware.moos.SonarCtrl.sonarReader')
            stbd_beam.add_stream('moos','morse.middleware.moos.SonarCtrl.sonarNotifier')
            stbd_beam.frequency(30 * timewarp)
            self.append(stbd_beam)

        if ( use_multiStaticSonar ):
            self.multiStaticSonar = MultiStaticSonar()
            self.multiStaticSonar.translate(-0.15,0,0.3)
            self.multiStaticSonar.add_stream('moos','morse.sensors.multiStaticSonarCtrl.multiStaticSimNotifier')
            self.multiStaticSonar.frequency(10)
            self.append(self.multiStaticSonar)

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
        self.append(self.gps)
        
        self.odom = Odometry()
        self.append(self.odom)

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
        self.pose.frequency(frequency)
        self.imu.frequency(frequency)
        self.dvl.frequency(frequency)
        self.gps.frequency(1)
        self.bat.frequency(1)
