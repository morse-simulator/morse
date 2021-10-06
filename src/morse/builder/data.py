""" This module holds MORSE Builder API settings

MORSE_COMPONENTS:
Default path to Morse components.

MORSE_RESOURCE_PATH:
Path list to Morse components (like the shell PATH variable).

The search path for components. It is a colon-separated list of directories in
which MORSE looks for components. A zero-length (null) directory name in the
value of PATH indicates the current directory. A null directory name may appear
as two adjacent colons, or as an initial or trailing colon. The default path is
``MORSE_COMPONENTS``, wich means this list always contains morse components path.
You can export it before to start morse (commonly in ``~/.bashrc``) as::

    export MORSE_RESOURCE_PATH="/home/user/my_own_components:/home/user/my_own_environments"

MORSE_MODIFIERS:
Path to the modifiers modules.

MORSE_DATASTREAM_MODULE:
Path to the middleware modules.

MORSE_MODIFIER_DICT:
Associate a modifier function to a component.
"""
import os

MORSE_COMPONENTS = os.path.join(os.getenv('MORSE_ROOT', '/usr/local'),
                                'share', 'morse', 'data')

MORSE_RESOURCE_PATH = ':'.join([MORSE_COMPONENTS,
                                os.getenv('MORSE_RESOURCE_PATH', '')])

MORSE_DATASTREAM_MODULE = {
    'ros': 'morse.middleware.ros_datastream.ROSDatastreamManager',
    'socket': 'morse.middleware.socket_datastream.SocketDatastreamManager',
    'yarp': 'morse.middleware.yarp_datastream.YarpDatastreamManager',
    'pocolibs': 'morse.middleware.pocolibs_datastream.PocolibsDatastreamManager',
    'text': 'morse.middleware.text_datastream.TextDatastreamManager',
    'moos': 'morse.middleware.moos_datastream.MOOSDatastreamManager',
    'hla': 'morse.middleware.hla_datastream.HLADatastreamManager',
    'mavlink': 'morse.middleware.mavlink_datastream.MavlinkDatastreamManager',
    'pprzlink': 'morse.middleware.pprzlink_datastream.PprzlinkDatastreamManager',
}

MORSE_MODIFIER_DICT = {
    'NED': {
        'morse.sensors.pose.Pose': "morse.modifiers.ned.CoordinatesToNED",
        'morse.sensors.gps.GPS': "morse.modifiers.ned.CoordinatesToNED",
        'morse.sensors.odometry.Odometry': "morse.modifiers.ned.PoseToNED",
        'morse.sensors.gyroscope.Gyroscope': "morse.modifiers.ned.AnglesToNED",
        'morse.actuators.destination.Destination': "morse.modifiers.ned.CoordinatesFromNED",
        'morse.actuators.waypoint.Waypoint': "morse.modifiers.ned.CoordinatesFromNED",
        'morse.actuators.orientation.Orientation': "morse.modifiers.ned.AnglesFromNED",
        'morse.actuators.teleport.Teleport': "morse.modifiers.ned.CoordinatesFromNED",
    },
    'ECEF': {
        'morse.sensors.pose.Pose': "morse.modifiers.ecef.CoordinatesToECEF",
        'morse.sensors.gps.GPS': "morse.modifiers.ecef.CoordinatesToECEF",
        'morse.actuators.destination.Destination': "morse.modifiers.ecef.CoordinatesFromECEF",
        'morse.actuators.waypoint.Waypoint': "morse.modifiers.ecef.CoordinatesFromECEF",
        'morse.actuators.teleport.Teleport': "morse.modifiers.ecef.CoordinatesFromECEF",
    },
    'feet': {
        'morse.sensors.pose.Pose': "morse.modifiers.feet.MeterToFeet",
        'morse.sensors.gps.GPS': "morse.modifiers.feet.MeterToFeet",
        'morse.actuators.destination.Destination': "morse.modifiers.feet.FeetToMeter",
        'morse.actuators.waypoint.Waypoint': "morse.modifiers.feet.FeetToMeter",
        'morse.actuators.teleport.Teleport': "morse.modifiers.feet.FeetToMeter"
    },
    'geodetic': {
        'morse.sensors.pose.Pose': "morse.modifiers.geodetic.CoordinatesToGeodetic",
        'morse.sensors.gps.GPS': "morse.modifiers.geodetic.CoordinatesToGeodetic",
        'morse.actuators.destination.Destination': "morse.modifiers.geodetic.CoordinatesFromGeodetic",
        'morse.actuators.waypoint.Waypoint': "morse.modifiers.geodetic.CoordinatesFromGeodetic",
        'morse.actuators.teleport.Teleport': "morse.modifiers.geodetic.CoordinatesFromGeodetic",
    },
    'geocentric': {
        'morse.sensors.pose.Pose': "morse.modifiers.geocentric.CoordinatesToGeocentric",
        'morse.sensors.gps.GPS': "morse.modifiers.geocentric.CoordinatesToGeocentric",
        'morse.actuators.destination.Destination': "morse.modifiers.geocentric.CoordinatesFromGeocentric",
        'morse.actuators.waypoint.Waypoint': "morse.modifiers.geocentric.CoordinatesFromGeocentric",
        'morse.actuators.teleport.Teleport': "morse.modifiers.geocentric.CoordinatesFromGeocentric",
    },
    'UTM' : {
        'morse.sensors.pose.Pose': "morse.modifiers.utm.CoordinatesToUTM",
        'morse.sensors.gps.GPS': "morse.modifiers.utm.CoordinatesToUTM",
        'morse.actuators.destination.Destination': "morse.modifiers.utm.CoordinatesFromUTM",
        'morse.actuators.waypoint.Waypoint': "morse.modifiers.utm.CoordinatesFromUTM",
    },
    'PoseNoise' : {
        'morse.sensors.odometry.Odometry': "morse.modifiers.pose_noise.PoseNoiseModifier",
        'morse.sensors.pose.Pose': "morse.modifiers.pose_noise.PoseNoiseModifier",
        'morse.sensors.gps.GPS': "morse.modifiers.pose_noise.PositionNoiseModifier",
        'morse.sensors.gyroscope.Gyroscope': "morse.modifiers.pose_noise.OrientationNoiseModifier",
    },
    'IMUNoise' : {
        'morse.sensors.imu.IMU': "morse.modifiers.imu_noise.IMUNoiseModifier",
    },
    'Noise' : {
        'morse.sensors.imu.IMU': "morse.modifiers.imu_noise.IMUNoiseModifier",
        'morse.sensors.odometry.Odometry': "morse.modifiers.pose_noise.PoseNoiseModifier",
        'morse.sensors.pose.Pose': "morse.modifiers.pose_noise.PoseNoiseModifier",
        'morse.sensors.gps.GPS': "morse.modifiers.pose_noise.PositionNoiseModifier",
        'morse.sensors.gyroscope.Gyroscope': "morse.modifiers.pose_noise.OrientationNoiseModifier",
    }
}

INTERFACE_DEFAULT_OUT = {
        "socket": "morse.middleware.socket_datastream.SocketPublisher",
        "yarp": "morse.middleware.yarp_datastream.YarpPublisher",
        "yarp_json": "morse.middleware.yarp.yarp_json.YarpJsonPublisher",
        "text": "morse.middleware.text_datastream.Publisher",
        "moos": "morse.middleware.moos.abstract_moos.StringPublisher"
}

INTERFACE_DEFAULT_IN = {
        "socket": "morse.middleware.socket_datastream.SocketReader",
        "yarp": "morse.middleware.yarp_datastream.YarpReader",
        "yarp_json": "morse.middleware.yarp.yarp_json.YarpJsonReader",
}

MORSE_DATASTREAM_DICT = {
    "morse.sensors.accelerometer.Accelerometer": {
        "default": {
            "ros": 'morse.middleware.ros.accelerometer.TwistPublisher',
            "socket": INTERFACE_DEFAULT_OUT,
            "yarp": INTERFACE_DEFAULT_OUT,
            "moos": INTERFACE_DEFAULT_OUT,
            "text": INTERFACE_DEFAULT_OUT,
            }
        },
    "morse.sensors.armature_pose.ArmaturePose": {
        "default": {
            "ros": 'morse.middleware.ros.jointstate.JointStatePublisher',
            "socket": INTERFACE_DEFAULT_OUT,
            "yarp": INTERFACE_DEFAULT_OUT,
            "moos": INTERFACE_DEFAULT_OUT,
            "text": INTERFACE_DEFAULT_OUT
            }
        },
    "morse.sensors.attitude.Attitude": {
        "default": {
            "socket": INTERFACE_DEFAULT_OUT,
            "yarp": INTERFACE_DEFAULT_OUT,
            "moos": INTERFACE_DEFAULT_OUT,
            "text": INTERFACE_DEFAULT_OUT,
            "mavlink": 'morse.middleware.mavlink.attitude.AttitudeSensor',
            }
        },
    "morse.sensors.barometer.Barometer": {
        "default": {
            "socket": INTERFACE_DEFAULT_OUT,
            "yarp": INTERFACE_DEFAULT_OUT,
            "moos": INTERFACE_DEFAULT_OUT,
            "text": INTERFACE_DEFAULT_OUT,
            }
        },
    "morse.sensors.battery.Battery": {
        "default": {
            "ros": 'morse.middleware.ros.battery.Float32Publisher',
            "socket": INTERFACE_DEFAULT_OUT,
            "yarp": INTERFACE_DEFAULT_OUT,
            "text": INTERFACE_DEFAULT_OUT,
            "moos": INTERFACE_DEFAULT_OUT,
            }
        },
    "morse.sensors.clock.Clock": {
        "default": {
            "socket": INTERFACE_DEFAULT_OUT,
            "yarp": INTERFACE_DEFAULT_OUT,
            "text": INTERFACE_DEFAULT_OUT,
            "moos": INTERFACE_DEFAULT_OUT,
            "ros": 'morse.middleware.ros.clock.ClockPublisher',
            }
        },
    "morse.sensors.depth_camera.DepthCamera": {
        "default": {
            "socket": 'morse.middleware.sockets.depth_camera.DepthCameraPublisher',
            "ros": 'morse.middleware.ros.depth_camera.DepthCameraPublisher',
            'pocolibs': 'morse.middleware.pocolibs.sensors.stereopixel.Spix3DImagePoster'
            }
        },
    "morse.sensors.depth_camera.DepthCameraRotationZ": {
        "default": {
            "socket": 'morse.middleware.sockets.depth_camera.DepthCameraPublisher',
            "ros": 'morse.middleware.ros.depth_camera.DepthCameraPublisher',
            'pocolibs': 'morse.middleware.pocolibs.sensors.velodyne.Velodyne3DImage'
            }
        },
    "morse.sensors.depth_camera_aggregator.DepthCameraAggregator": {
        "default": {
            "socket": 'morse.middleware.sockets.depth_camera.DepthCameraPublisher',
            "ros": 'morse.middleware.ros.depth_camera.DepthCameraPublisher',
            }
        },
    "morse.sensors.gps.GPS": {
        "simple": {
            "ros": 'morse.middleware.ros.gps.NavSatFixPublisher',
            "socket": INTERFACE_DEFAULT_OUT,
            "yarp": INTERFACE_DEFAULT_OUT,
            "yarp_json": INTERFACE_DEFAULT_OUT,
            "text": INTERFACE_DEFAULT_OUT,
            "pocolibs": ['morse.middleware.pocolibs.sensors.pom.PomSensorPoster',
                         'morse.middleware.pocolibs.sensors.pom.PomPoster'],
            "moos": 'morse.middleware.moos.gps.GPSNotifier'
            },
        "raw": {
            "ros": 'morse.middleware.ros.gps.NavSatFixRawPublisher',
            "socket": INTERFACE_DEFAULT_OUT,
            "yarp": INTERFACE_DEFAULT_OUT,
            "yarp_json": INTERFACE_DEFAULT_OUT,
            "text": INTERFACE_DEFAULT_OUT,
            "moos": 'morse.middleware.moos.gps.GPSRawNotifier'
            },
        "extended": {
            "socket": INTERFACE_DEFAULT_OUT,
            "yarp": INTERFACE_DEFAULT_OUT,
            "yarp_json": INTERFACE_DEFAULT_OUT,
            "text": INTERFACE_DEFAULT_OUT,
            }
        },
    "morse.sensors.gyroscope.Gyroscope": {
        "default": {
            "ros": 'morse.middleware.ros.StringPublisher',
            "socket": INTERFACE_DEFAULT_OUT,
            "yarp": INTERFACE_DEFAULT_OUT,
            "text": INTERFACE_DEFAULT_OUT,
            "pocolibs": ['morse.middleware.pocolibs.sensors.pom.PomSensorPoster',
                         'morse.middleware.pocolibs.sensors.pom.PomPoster'],
            "moos": 'morse.middleware.moos.gyroscope.GyroscopeNotifier'
            }
        },
    "morse.sensors.human_posture.HumanPosture": {
        "default": {
            "socket": INTERFACE_DEFAULT_OUT,
            "yarp": INTERFACE_DEFAULT_OUT,
            "moos": INTERFACE_DEFAULT_OUT,
            "text": INTERFACE_DEFAULT_OUT,
            "pocolibs": 'morse.middleware.pocolibs.sensors.human_posture.HumanPoster'
            }
        },
    "morse.sensors.imu.IMU": {
        "default": {
            "ros": 'morse.middleware.ros.imu.ImuPublisher',
            "socket": INTERFACE_DEFAULT_OUT,
            "yarp": INTERFACE_DEFAULT_OUT,
            "text": INTERFACE_DEFAULT_OUT,
            "moos": 'morse.middleware.moos.imu.IMUNotifier'
            }
        },
    "morse.sensors.laserscanner.LaserScanner": {
        "raw": {
            "ros": ['morse.middleware.ros.laserscanner.LaserScanPublisher',
                    'morse.middleware.ros.laserscanner.PointCloud2Publisher'],
            "socket": INTERFACE_DEFAULT_OUT,
            "yarp": 'morse.middleware.yarp.laserscanner.YarpLaserScannerPublisher',
            "moos": 'morse.middleware.moos.sick.LIDARNotifier'
            },
        "rssi": {
            "socket": INTERFACE_DEFAULT_OUT
            },
        "range": {
            "ros": 'morse.middleware.ros.infrared.RangePublisher',
            "socket": INTERFACE_DEFAULT_OUT,
            }
        },
    "morse.sensors.odometry.Odometry": {
        "differential": {
            "socket": INTERFACE_DEFAULT_OUT,
            "yarp": INTERFACE_DEFAULT_OUT,
            "moos": INTERFACE_DEFAULT_OUT,
            "text": INTERFACE_DEFAULT_OUT
            },
        "raw": {
            "socket": INTERFACE_DEFAULT_OUT,
            "moos": INTERFACE_DEFAULT_OUT,
            "yarp": INTERFACE_DEFAULT_OUT,
            "text": INTERFACE_DEFAULT_OUT
            },
        "integrated": {
            "ros": 'morse.middleware.ros.odometry.OdometryPublisher',
            "socket": INTERFACE_DEFAULT_OUT,
            "moos": INTERFACE_DEFAULT_OUT,
            "yarp": INTERFACE_DEFAULT_OUT,
            "text": INTERFACE_DEFAULT_OUT,
            "pocolibs": ['morse.middleware.pocolibs.sensors.pom.PomSensorPoster',
                         'morse.middleware.pocolibs.sensors.pom.PomPoster'],
            "mavlink": 'morse.middleware.mavlink.odometry_to_local_ned.OdometrySensor'
            }
        },
    "morse.sensors.magnetometer.Magnetometer": {
        "default": {
            "socket": INTERFACE_DEFAULT_OUT,
            "yarp": INTERFACE_DEFAULT_OUT,
            "moos": INTERFACE_DEFAULT_OUT,
            "text": INTERFACE_DEFAULT_OUT,
        }
        },
    "morse.sensors.pose.Pose": {
        "default": {
            "ros": ['morse.middleware.ros.pose.PoseStampedPublisher',
                    'morse.middleware.ros.pose.TFPublisher'],
            "socket": INTERFACE_DEFAULT_OUT,
            "yarp": INTERFACE_DEFAULT_OUT,
            "text": INTERFACE_DEFAULT_OUT,
            "pocolibs": ['morse.middleware.pocolibs.sensors.pom.PomSensorPoster',
                         'morse.middleware.pocolibs.sensors.pom.PomPoster'],
            "moos": 'morse.middleware.moos.pose.PoseNotifier'
            }
        },
    "morse.sensors.proximity.Proximity": {
        "default": {
            "ros": 'morse.middleware.ros.StringPublisher',
            "socket": INTERFACE_DEFAULT_OUT,
            "yarp": INTERFACE_DEFAULT_OUT,
            "yarp_json": INTERFACE_DEFAULT_OUT,
            "moos": INTERFACE_DEFAULT_OUT,
            "text": INTERFACE_DEFAULT_OUT,
            }
        },
    "morse.sensors.ptu_posture.PTUPosture": {
        "default": {
            "ros": 'morse.middleware.ros.jointstate.JointStatePublisher',
            "socket": INTERFACE_DEFAULT_OUT,
            "yarp": INTERFACE_DEFAULT_OUT,
            "moos": INTERFACE_DEFAULT_OUT,
            "text": INTERFACE_DEFAULT_OUT,
            "pocolibs": 'morse.middleware.pocolibs.sensors.platine_posture.PlatinePoster'
            }
        },
    "morse.sensors.radar_altimeter.RadarAltimeter": {
        "default": {
            "socket": INTERFACE_DEFAULT_OUT,
            "yarp": INTERFACE_DEFAULT_OUT,
            "moos": INTERFACE_DEFAULT_OUT,
            "text": INTERFACE_DEFAULT_OUT
            }
        },
    "morse.sensors.stereo_unit.StereoUnit": {
        "default": {
            "pocolibs": 'morse.middleware.pocolibs.sensors.viam.ViamPoster'
            }
        },
    "morse.sensors.search_and_rescue.SearchAndRescue": {
        "default": {
            "socket": INTERFACE_DEFAULT_OUT,
            "yarp": INTERFACE_DEFAULT_OUT,
            "yarp_json": INTERFACE_DEFAULT_OUT,
            "moos": INTERFACE_DEFAULT_OUT,
            "text": INTERFACE_DEFAULT_OUT,
            "pocolibs": 'morse.middleware.pocolibs.sensors.target.TargetPoster'
            }
        },
    "morse.sensors.semantic_camera.SemanticCamera": {
        "default": {
            "ros": ['morse.middleware.ros.semantic_camera.SemanticCameraPublisher',
                    'morse.middleware.ros.semantic_camera.SemanticCameraPublisherLisp'],
            "socket": INTERFACE_DEFAULT_OUT,
            "yarp": INTERFACE_DEFAULT_OUT,
            "pocolibs": 'morse.middleware.pocolibs.sensors.viman.VimanPoster'
            }
        },
    "morse.sensors.thermometer.Thermometer": {
        "default": {
            "socket": INTERFACE_DEFAULT_OUT,
            "yarp": INTERFACE_DEFAULT_OUT,
            "moos": INTERFACE_DEFAULT_OUT,
            "text": INTERFACE_DEFAULT_OUT,
            }
        },
    "morse.sensors.velocity.Velocity": {
        "default": {
            "ros": ['morse.middleware.ros.velocity.TwistStampedPublisher'],
            "socket": INTERFACE_DEFAULT_OUT,
            "yarp": INTERFACE_DEFAULT_OUT,
            "moos": INTERFACE_DEFAULT_OUT,
            "text": INTERFACE_DEFAULT_OUT,
            }
        },
    "morse.sensors.video_camera.VideoCamera": {
        "default": {
            "ros": 'morse.middleware.ros.video_camera.VideoCameraPublisher',
            "socket": 'morse.middleware.sockets.video_camera.VideoCameraPublisher',
            "yarp": 'morse.middleware.yarp.video_camera.YarpImagePublisher',
            "pocolibs": 'morse.middleware.pocolibs.sensors.viam.ViamPoster'
            }
        },
    "morse.sensors.depth_camera.DepthVideoCamera": {
        "default": {
            "socket": 'morse.middleware.sockets.depth_camera.DepthCameraPublisher',
            "ros": 'morse.middleware.ros.video_camera.DepthCameraPublisher',
            "yarp": 'morse.middleware.yarp.video_depth_camera.YarpImageFloatPublisher',
            "pocolibs": 'morse.middleware.pocolibs.sensors.viam.ViamPoster'
            }
        },
    "morse.sensors.Dvl.DvlPublisher": {
        "default": {
            #"socket":   'morse.middleware.sockets.depth_camera.DepthCameraPublisher',
            "ros":      'morse.middleware.ros.dvl.DvlPublisher'
            #"yarp":     'morse.middleware.yarp.video_depth_camera.YarpImageFloatPublisher',
            #"pocolibs": 'morse.middleware.pocolibs.sensors.viam.ViamPoster'
            }
        },

    "morse.actuators.armature.Armature": {
        "default": {
            "socket": INTERFACE_DEFAULT_IN,
            "yarp": INTERFACE_DEFAULT_IN,
            "pocolibs": 'morse.middleware.pocolibs.actuators.lwr.LwrPoster'
            }
        },
    "morse.actuators.arucomarker.Arucomarker": {
        "default": {
            "ros": 'morse.middleware.ros.read_pose.PoseReader',
            "socket": INTERFACE_DEFAULT_IN,
            "yarp": INTERFACE_DEFAULT_IN,
            }
        },
    "morse.actuators.destination.Destination": {
        "default": {
            "ros": 'morse.middleware.ros.destination.PointReader',
            "socket": INTERFACE_DEFAULT_IN,
            "yarp": INTERFACE_DEFAULT_IN,
            }
        },
    "morse.actuators.force_torque.ForceTorque": {
        "default": {
            "socket": INTERFACE_DEFAULT_IN,
            "yarp": INTERFACE_DEFAULT_IN,
            "ros": 'morse.middleware.ros.force_torque.WrenchReader',
            }
        },
    "morse.actuators.external_force.ExternalForce": {
        "default": {
            "socket": INTERFACE_DEFAULT_IN,
            "yarp": INTERFACE_DEFAULT_IN,
            "ros": 'morse.middleware.ros.force_torque.WrenchReader',
            }
        },
    "morse.actuators.gripper.Gripper": {
        "default": {
            "socket": INTERFACE_DEFAULT_IN,
            "yarp": INTERFACE_DEFAULT_IN,
            }
        },
    "morse.actuators.light.Light": {
        "default": {
            "ros": 'morse.middleware.ros.light.BoolReader',
            "socket": INTERFACE_DEFAULT_IN,
            "yarp": INTERFACE_DEFAULT_IN,
            "moos": 'morse.middleware.moos.light.LightReader',
            }
        },
    "morse.actuators.mocap_control.MocapControl": {
        "default": {
            "pocolibs": 'morse.middleware.pocolibs.actuators.niut.NiutPoster'
            }
        },
    "morse.actuators.orientation.Orientation": {
        "default": {
            "ros": 'morse.middleware.ros.orientation.QuaternionReader',
            "socket": INTERFACE_DEFAULT_IN,
            "yarp": INTERFACE_DEFAULT_IN,
            }
        },
    "morse.actuators.ptu.PTU": {
        "default": {
            "ros": 'morse.middleware.ros.platine.Vector3Reader',
            "socket": INTERFACE_DEFAULT_IN,
            "yarp": INTERFACE_DEFAULT_IN,
            "pocolibs": ['morse.middleware.pocolibs.actuators.platine.PlatinePoster',
                         'morse.middleware.pocolibs.actuators.platine.PlatineAxisPoster']
            }
        },
    "morse.actuators.quadrotor_dynamic_control.QuadrotorDynamicControl": {
        "default": {
            "ros": 'morse.middleware.ros.quadrotor_dynamic.QuadrotorDynamicReader',
            "socket": INTERFACE_DEFAULT_IN,
            "yarp": INTERFACE_DEFAULT_IN,
            }
        },
    "morse.actuators.rotorcraft_attitude.RotorcraftAttitude": {
        "default": {
            "ros": 'morse.middleware.ros.rotorcraft_attitude.RotorcraftAttitudeReader',
            "socket": INTERFACE_DEFAULT_IN,
            "yarp": INTERFACE_DEFAULT_IN,
            "mavlink": "morse.middleware.mavlink.read_attitude_target.AttitudeTarget",
            }
        },
    "morse.actuators.rotorcraft_velocity.RotorcraftVelocity": {
        "default": {
            "ros": 'morse.middleware.ros.read_twist.TwistReader',
            "socket": INTERFACE_DEFAULT_IN,
            }
        },
    "morse.actuators.rotorcraft_waypoint.RotorcraftWaypoint": {
        "default": {
            "ros": 'morse.middleware.ros.read_pose.PoseReader',
            "socket": INTERFACE_DEFAULT_IN,
            "yarp": INTERFACE_DEFAULT_IN,
            "mavlink": 'morse.middleware.mavlink.local_position_ned_to_waypoint.WaypointActuator',
            }
        },
    "morse.actuators.stabilized_quadrotor.StabilizedQuadrotor": {
        "default": {
            "socket": INTERFACE_DEFAULT_IN,
            "yarp": INTERFACE_DEFAULT_IN,
            }
        },
        "morse.actuators.steer_force.SteerForce": {
        "default": {
            "socket": INTERFACE_DEFAULT_IN,
            "yarp": INTERFACE_DEFAULT_IN,
            "moos" : 'morse.middleware.moos.motion.MotionReader'
            }
        },
        "morse.actuators.v_omega.MotionVW": {
        "default": {
            "ros": 'morse.middleware.ros.motion_vw.TwistReader',
            "socket": INTERFACE_DEFAULT_IN,
            "yarp": INTERFACE_DEFAULT_IN,
            "pocolibs": 'morse.middleware.pocolibs.actuators.genpos.GenPosPoster',
            "moos" : 'morse.middleware.moos.motion.MotionReader'
            }
        },
    "morse.actuators.v_omega_diff_drive.MotionVWDiff": {
        "default": {
            "ros": 'morse.middleware.ros.motion_vw.TwistReader',
            "socket": INTERFACE_DEFAULT_IN,
            "yarp": INTERFACE_DEFAULT_IN,
            "pocolibs": 'morse.middleware.pocolibs.actuators.genpos.GenPosPoster',
            "moos" : 'morse.middleware.moos.motion.MotionReader'
            }
        },
    "morse.actuators.xy_omega.MotionXYW": {
        "default": {
            "ros": 'morse.middleware.ros.motion_xyw.TwistReader',
            "socket": INTERFACE_DEFAULT_IN,
            "yarp": INTERFACE_DEFAULT_IN,
            }
        },
    "morse.actuators.teleport.Teleport": {
        "default": {
            "ros": 'morse.middleware.ros.read_pose.PoseReader',
            "socket": INTERFACE_DEFAULT_IN,
            "yarp": INTERFACE_DEFAULT_IN,
            "moos" : 'morse.middleware.moos.pose.PoseReader',
            "pprzlink" : 'morse.middleware.pprzlink.set_rotorcraft_pose.RotorcraftPose'
            }
        },
    "morse.actuators.waypoint.Waypoint": {
        "default": {
            "socket": INTERFACE_DEFAULT_IN,
            "yarp": INTERFACE_DEFAULT_IN,
            "yarp_json": INTERFACE_DEFAULT_IN,
            }
        },
    "morse.actuators.bluefin21_hydrodynamics.Hydrodynamics": {
        "default": {
            "ros": 'morse.middleware.ros.thruster.TailconeCtrlReader'
            }
        },
    "morse.actuators.fixedthrusters.Fixedthrusters": {
        "default": {
            "ros": 'morse.middleware.ros.thruster.FixedCtrlReader'
            }
        }
}

MORSE_SERVICE_DICT = {
    "socket": "morse.middleware.socket_request_manager.SocketRequestManager",
    "yarp": "morse.middleware.yarp_request_manager.YarpRequestManager",
    "yarp_json": "morse.middleware.yarp_json_request_manager.YarpRequestManager",
    "pocolibs": "morse.middleware.pocolibs_request_manager.PocolibsRequestManager",
    "ros": "morse.middleware.ros_request_manager.RosRequestManager",
}
