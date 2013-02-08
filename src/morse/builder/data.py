import os

"""
MORSE_COMPONENTS:
path to the Morse components
"""

MORSE_COMPONENTS = os.path.join(os.getenv('MORSE_ROOT', '/usr/local'), \
                                'share', 'morse', 'data')

MORSE_RESOURCE_PATH = ':'.join([MORSE_COMPONENTS, \
                                os.getenv('MORSE_RESOURCE_PATH', '')])

"""
MORSE_MODIFIERS:
path to the modifiers modules
"""
MORSE_MODIFIERS = {
    'NED': 'morse.modifiers.ned.MorseNEDClass',
    'UTM': 'morse.modifiers.utm.MorseUTMClass',
    'GPSNoise': 'morse.modifiers.gps_noise.MorseGPSNoiseClass',
    'OdometryNoise': 'morse.modifiers.odometry_noise.MorseOdometryNoiseClass',
    'IMUNoise': 'morse.modifiers.imu_noise.MorseIMUNoiseClass',
    'PoseNoise': 'morse.modifiers.pose_noise.MorsePoseNoiseClass',
}

"""
MORSE_DATASTREAM_MODULE:
path to the middleware modules
"""
MORSE_DATASTREAM_MODULE = {
    'ros': 'morse.middleware.ros_datastream.ROS',
    'socket': 'morse.middleware.socket_datastream.Socket',
    'yarp': 'morse.middleware.yarp_datastream.Yarp',
    'pocolibs': 'morse.middleware.pocolibs_datastream.Pocolibs',
    'text': 'morse.middleware.text_datastream.Text',
}

"""
MORSE_MODIFIER_DICT:
associate a modifier function to a component.
"""
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

"""
middleware-dictionary-convention:
{
    .blend-middleware: {
        .blend-component: ['MW', 'method', 'path']
    }
}
"""

INTERFACE_DEFAULT_OUT = {
        "socket": "morse.middleware.socket_datastream.SocketPublisher",
        "yarp": ["post_message"],
        "yarp_json": ['post_json_message', 'morse/middleware/yarp/json_mod'],
        "text": ['write_data'],
}

INTERFACE_DEFAULT_IN = {
        "socket": "morse.middleware.socket_datastream.SocketReader",
        "yarp": ["read_message"],
        "yarp_json": ['read_json_message', 'morse/middleware/yarp/json_mod'],
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
            "ros": 'morse.middleware.ros.depth_camera.DepthCameraPublisher'
            }
        },
    "morse.sensors.gps.GPS": {
        "default": {
            "ros": 'morse.middleware.ros.gps.NavSatFixPublisher',
            "socket": INTERFACE_DEFAULT_OUT,
            "yarp": INTERFACE_DEFAULT_OUT,
            "yarp_json": INTERFACE_DEFAULT_OUT,
            "text": INTERFACE_DEFAULT_OUT,
            "pocolibs": 'morse.middleware.pocolibs.sensors.pom.PomSensorPoster'
            }
        },
    "morse.sensors.gyroscope.Gyroscope": {
        "default": {
            "ros": 'morse.middleware.ros.StringPublisher',
            "socket": INTERFACE_DEFAULT_OUT,
            "yarp": INTERFACE_DEFAULT_OUT,
            "text": INTERFACE_DEFAULT_OUT,
            "pocolibs": 'morse.middleware.pocolibs.sensors.pom.PomSensorPoster'
            }
        },
    "morse.sensors.human_posture.HumanPosture": {
        "default": {
            "pocolibs": 'morse.middleware.pocolibs.sensors.human_posture.HumanPoster'
            }
        },
    "morse.sensors.imu.IMU": {
        "default": {
            "ros": 'morse.middleware.ros.imu.ImuPublisher',
            "socket": INTERFACE_DEFAULT_OUT,
            "yarp": INTERFACE_DEFAULT_OUT,
            "text": INTERFACE_DEFAULT_OUT,
            }
        },
    "morse.sensors.kinect.Kinect": {
        "default": {
            "ros": 'morse.middleware.ros.kinect.XYZRGBPublisher',
            }
        },
    "morse.sensors.laserscanner.LaserScanner": {
        "default": {
            "ros": 'morse.middleware.ros.laserscanner.LaserScanPublisher',
            "socket": INTERFACE_DEFAULT_OUT,
            "yarp": ['post_sick_message', 'morse/middleware/yarp/sick'],
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
            "pocolibs": 'morse.middleware.pocolibs.sensors.pom.PomSensorPoster'
            }
        },
    "morse.sensors.pose.Pose": {
        "default": {
            "ros": 'morse.middleware.ros.pose.PoseStampedPublisher',
            "socket": INTERFACE_DEFAULT_OUT,
            "yarp": INTERFACE_DEFAULT_OUT,
            "text": INTERFACE_DEFAULT_OUT,
            "pocolibs": 'morse.middleware.pocolibs.sensors.pom.PomSensorPoster'
            }
        },
    "morse.sensors.proximity.Proximity": {
        "default": {
            "ros": 'morse.middleware.ros.StringPublisher',
            "socket": INTERFACE_DEFAULT_OUT,
            "yarp": ['post_dictionary_data', 'morse/middleware/yarp/dictionary'],
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
            "ros": 'morse.middleware.ros.semantic_camera.SemanticCameraPublisher',
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
            "yarp": ['post_image_RGBA'],
            }
        },

    "morse.actuators.armature.Armature": {
        "default": {
            "ros": 'morse.middleware.ros.jointtrajectorycontrollers.JointTrajectoryControllerStatePublisher',
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
            "pocolibs": 'morse.middleware.pocolibs.actuators.platine.PlatinePoster'
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
            }
        },
        "morse.actuators.v_omega.MotionVW": {
        "default": {
            "ros": 'morse.middleware.ros.motion_vw.TwistReader',
            "socket": INTERFACE_DEFAULT_IN,
            "yarp": INTERFACE_DEFAULT_IN,
            "pocolibs": 'morse.middleware.pocolibs.actuators.genpos.GenPosPoster'
            }
        },
    "morse.actuators.v_omega_diff_drive.MotionVWDiff": {
        "default": {
            "socket": INTERFACE_DEFAULT_IN,
            "yarp": INTERFACE_DEFAULT_IN,
            "pocolibs": 'morse.middleware.pocolibs.actuators.genpos.GenPosPoster'
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
