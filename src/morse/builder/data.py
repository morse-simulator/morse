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

MORSE_COMPONENTS = os.path.join(os.getenv('MORSE_ROOT', '/usr/local'), \
                                'share', 'morse', 'data')

MORSE_RESOURCE_PATH = ':'.join([MORSE_COMPONENTS, \
                                os.getenv('MORSE_RESOURCE_PATH', '')])

MORSE_MODIFIERS = {
    'NED': 'morse.modifiers.ned.MorseNEDClass',
    'UTM': 'morse.modifiers.utm.MorseUTMClass',
    'GPSNoise': 'morse.modifiers.gps_noise.MorseGPSNoiseClass',
    'OdometryNoise': 'morse.modifiers.odometry_noise.MorseOdometryNoiseClass',
    'IMUNoise': 'morse.modifiers.imu_noise.MorseIMUNoiseClass',
    'PoseNoise': 'morse.modifiers.pose_noise.MorsePoseNoiseClass',
}

MORSE_DATASTREAM_MODULE = {
    'ros': 'morse.middleware.ros_datastream.ROS',
    'socket': 'morse.middleware.socket_datastream.Socket',
    'yarp': 'morse.middleware.yarp_datastream.Yarp',
    'pocolibs': 'morse.middleware.pocolibs_datastream.Pocolibs',
    'text': 'morse.middleware.text_datastream.Text',
    'moos': 'morse.middleware.moos_datastream.MOOS'
}

MORSE_MODIFIER_DICT = {
    'NED': {
        'pose': [MORSE_MODIFIERS['NED'], 'blender_to_ned'],
        'gps': [MORSE_MODIFIERS['NED'], 'blender_to_ned'],
        'gyroscope': [MORSE_MODIFIERS['NED'], 'blender_to_ned_angle'],
        'destination': [MORSE_MODIFIERS['NED'], 'ned_to_blender'],
        'waypoint': [MORSE_MODIFIERS['NED'], 'ned_to_blender'],
        'orientation': [MORSE_MODIFIERS['NED'], 'ned_angle_to_blender'],
        'teleport': [MORSE_MODIFIERS['NED'], 'ned_to_blender'],
    },
    'UTM' : {
        'pose': [MORSE_MODIFIERS['UTM'], 'blender_to_utm'],
        'gps': [MORSE_MODIFIERS['UTM'], 'blender_to_utm'],
        'destination': [MORSE_MODIFIERS['UTM'], 'utm_to_blender'],
        'waypoint': [MORSE_MODIFIERS['UTM'], 'utm_to_blender'],
    },
    'OdometryNoise' : {
        'odometry': [MORSE_MODIFIERS['OdometryNoise'], 'noisify']
    },
    'Noise' : {
        'imu': [MORSE_MODIFIERS['IMUNoise'], 'noisify'],
        'pose': [MORSE_MODIFIERS['PoseNoise'], 'noisify'],
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
            "text": INTERFACE_DEFAULT_OUT,
            }
        },
    "morse.sensors.armature_pose.ArmaturePose": {
        "default": {
            "ros": 'morse.middleware.ros.jointstate.JointStatePublisher',
            "socket": INTERFACE_DEFAULT_OUT,
            "yarp": INTERFACE_DEFAULT_OUT,
            "text": INTERFACE_DEFAULT_OUT
            }
        },
    "morse.sensors.battery.Battery": {
        "default": {
            "ros": 'morse.middleware.ros.battery.Float32Publisher',
            "socket": INTERFACE_DEFAULT_OUT,
            "yarp": INTERFACE_DEFAULT_OUT,
            "text": INTERFACE_DEFAULT_OUT,
            }
        },
    "clock": {
        "default": {
            "ros": 'morse.middleware.ros.clock.ClockPublisher',
            }
        },
    "morse.sensors.depth_camera.DepthCamera": {
        "default": {
            "ros": 'morse.middleware.ros.depth_camera.DepthCameraPublisher',
            'pocolibs': 'morse.middleware.pocolibs.sensors.stereopixel.Spix3DImagePoster'
            }
        },
    "morse.sensors.gps.GPS": {
        "default": {
            "ros": 'morse.middleware.ros.gps.NavSatFixPublisher',
            "socket": INTERFACE_DEFAULT_OUT,
            "yarp": INTERFACE_DEFAULT_OUT,
            "yarp_json": INTERFACE_DEFAULT_OUT,
            "text": INTERFACE_DEFAULT_OUT,
            "pocolibs": ['morse.middleware.pocolibs.sensors.pom.PomSensorPoster',
                         'morse.middleware.pocolibs.sensors.pom.PomPoster'],
            "moos": 'morse.middleware.moos.gps.GPSNotifier'
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
    "morse.sensors.kinect.Kinect": {
        "default": {
            "ros": 'morse.middleware.ros.kinect.XYZRGBPublisher',
            }
        },
    "morse.sensors.laserscanner.LaserScanner": {
        "default": {
            "ros": ['morse.middleware.ros.laserscanner.LaserScanPublisher',
                    'morse.middleware.ros.laserscanner.PointCloud2Publisher'],
            "socket": INTERFACE_DEFAULT_OUT,
            "yarp": 'morse.middleware.yarp.laserscanner.YarpLaserScannerPublisher',
            'moos': 'morse.middleware.moos.sick.LIDARNotifier'
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
            "text": INTERFACE_DEFAULT_OUT
            },
        "raw": {
            "socket": INTERFACE_DEFAULT_OUT,
            "yarp": INTERFACE_DEFAULT_OUT,
            "text": INTERFACE_DEFAULT_OUT
            },
        "integrated": {
            "ros": 'morse.middleware.ros.odometry.OdometryPublisher',
            "socket": INTERFACE_DEFAULT_OUT,
            "yarp": INTERFACE_DEFAULT_OUT,
            "text": INTERFACE_DEFAULT_OUT,
            "pocolibs": ['morse.middleware.pocolibs.sensors.pom.PomSensorPoster',
                         'morse.middleware.pocolibs.sensors.pom.PomPoster']
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
            'moos': 'morse.middleware.moos.pose.PoseNotifier'
            }
        },
    "morse.sensors.proximity.Proximity": {
        "default": {
            "ros": 'morse.middleware.ros.StringPublisher',
            "socket": INTERFACE_DEFAULT_OUT,
            "yarp": INTERFACE_DEFAULT_OUT,
            "yarp_json": INTERFACE_DEFAULT_OUT,
            "text": INTERFACE_DEFAULT_OUT,
            }
        },
    "morse.sensors.ptu_posture.PTUPosture": {
        "default": {
            "socket": INTERFACE_DEFAULT_OUT,
            "yarp": INTERFACE_DEFAULT_OUT,
            "text": INTERFACE_DEFAULT_OUT,
            "pocolibs": 'morse.middleware.pocolibs.sensors.platine_posture.PlatinePoster'
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
            "text": INTERFACE_DEFAULT_OUT,
            }
        },
    "morse.sensors.video_camera.VideoCamera": {
        "default": {
            "ros": 'morse.middleware.ros.video_camera.VideoCameraPublisher',
            "socket": 'morse.middleware.sockets.video_camera.VideoPublisher',
            "yarp": 'morse.middleware.yarp_datastream.YarpImagePublisher',
            "pocolibs": 'morse.middleware.pocolibs.sensors.viam.ViamPoster'
            }
        },

    "morse.actuators.armature.Armature": {
        "default": {
            "socket": INTERFACE_DEFAULT_IN,
            "yarp": INTERFACE_DEFAULT_IN,
            "pocolibs": 'morse.middleware.pocolibs.actuators.lwr.LwrPoster'
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
    "morse.actuators.rotorcraft_waypoint.RotorcraftAttittude": {
        "default": {
            "socket": INTERFACE_DEFAULT_IN,
            }
        },
    "morse.actuators.rotorcraft_waypoint.RotorcraftWaypoint": {
        "default": {
            "ros": 'morse.middleware.ros.read_pose.PoseReader',
            "socket": INTERFACE_DEFAULT_IN,
            }
        },
    "morse.actuators.stabilized_quadrotor.StabilizedQuadrotor": {
        "default": {
            "socket": INTERFACE_DEFAULT_IN,
            }
        },
        "morse.actuators.steer_force.SteerForce": {
        "default": {
            "socket": INTERFACE_DEFAULT_IN,
            "yarp": INTERFACE_DEFAULT_IN,
            'moos' : 'morse.middleware.moos.motion.MotionReader'
            }
        },
        "morse.actuators.v_omega.MotionVW": {
        "default": {
            "ros": 'morse.middleware.ros.motion_vw.TwistReader',
            "socket": INTERFACE_DEFAULT_IN,
            "yarp": INTERFACE_DEFAULT_IN,
            "pocolibs": 'morse.middleware.pocolibs.actuators.genpos.GenPosPoster',
            'moos' : 'morse.middleware.moos.motion.MotionReader'
            }
        },
    "morse.actuators.v_omega_diff_drive.MotionVWDiff": {
        "default": {
            "ros": 'morse.middleware.ros.motion_vw.TwistReader',
            "socket": INTERFACE_DEFAULT_IN,
            "yarp": INTERFACE_DEFAULT_IN,
            "pocolibs": 'morse.middleware.pocolibs.actuators.genpos.GenPosPoster',
            'moos' : 'morse.middleware.moos.motion.MotionReader'
            }
        },
    "morse.actuators.xy_omega.MotionXYW": {
        "default": {
            "ros": 'morse.middleware.ros.motion_xyw.TwistReader',
            "socket": INTERFACE_DEFAULT_IN,
            }
        },
    "morse.actuators.teleport.Teleport": {
        "default": {
            "ros": 'morse.middleware.ros.read_pose.PoseReader',
            "socket": INTERFACE_DEFAULT_IN,
            "yarp": INTERFACE_DEFAULT_IN,
            }
        },
    "morse.actuators.waypoint.Waypoint": {
        "default": {
            "socket": INTERFACE_DEFAULT_IN,
            "yarp": INTERFACE_DEFAULT_IN,
            "yarp_json": INTERFACE_DEFAULT_IN,
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
