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
MORSE_DATASTREAM_DICT = {
    'ros': {
        'accelerometer': ['morse.middleware.ros.accelerometer.TwistPublisher'],
        'armature': ['morse.middleware.ros.jointtrajectorycontrollers.JointTrajectoryControllerStatePublisher'],
        'armature_pose': ['morse.middleware.ros.jointstate.JointStatePublisher'],
        'battery': ['morse.middleware.ros.battery.Float32Publisher'],
        'clock': ['morse.middleware.ros.clock.ClockPublisher'],
        'gps': ['morse.middleware.ros.gps.NavSatFixPublisher'],
        'gyroscope': ['morse.middleware.ros.StringPublisher'],
        'infrared': ['morse.middleware.ros.infrared.RangePublisher'],
        'imu': ['morse.middleware.ros.imu.ImuPublisher'],
        'sick': ['morse.middleware.ros.laserscanner.LaserScanPublisher'],
        'hokuyo': ['morse.middleware.ros.laserscanner.LaserScanPublisher'],
        'light': ['morse.middleware.ros.light.BoolReader'],
        'odometry': ['morse.middleware.ros.odometry.OdometryPublisher'],
        'pose': ['morse.middleware.ros.pose.PoseStampedPublisher'],
        'proximity': ['morse.middleware.ros.StringPublisher'],
        'semantic_camera': ['morse.middleware.ros.semantic_camera.SemanticCameraPublisher'],
        'video_camera': ['morse.middleware.ros.video_camera.VideoCameraPublisher'],
        'depth_camera': ['morse.middleware.ros.depth_camera.DepthCameraPublisher'],

        'light': ['morse.middleware.ros.light.BoolReader'],
        'ptu': ['morse.middleware.ros.platine.Vector3Reader'],
        'v_omega': ['morse.middleware.ros.motion_vw.TwistReader'],
        'xy_omega': ['morse.middleware.ros.motion_xyw.TwistReader'],
        'destination': ['morse.middleware.ros.destination.PointReader'],
        'force_torque': ['morse.middleware.ros.force_torque.WrenchReader'],
        'orientation': ['morse.middleware.ros.orientation.QuaternionReader'],
        'teleport': ['morse.middleware.ros.read_pose.PoseReader'],
        'rotorcraft_waypoint': ['morse.middleware.ros.read_pose.PoseReader'],
    },

    'socket': {
        'accelerometer': ['morse.middleware.socket_datastream.SocketPublisher'],
        'armature_pose': ['morse.middleware.socket_datastream.SocketPublisher'],
        'battery': ['morse.middleware.socket_datastream.SocketPublisher'],
        'gps': ['morse.middleware.socket_datastream.SocketPublisher'],
        'gyroscope': ['morse.middleware.socket_datastream.SocketPublisher'],
        'imu': ['morse.middleware.socket_datastream.SocketPublisher'],
        'sick': ['morse.middleware.socket_datastream.SocketPublisher'],
        'hokuyo': ['morse.middleware.socket_datastream.SocketPublisher'],
        'odometry': ['morse.middleware.socket_datastream.SocketPublisher'],
        'pose': ['morse.middleware.socket_datastream.SocketPublisher'],
        'proximity': ['morse.middleware.socket_datastream.SocketPublisher'],
        'ptu_posture': ['morse.middleware.socket_datastream.SocketPublisher'],
        'rosace': ['morse.middleware.socket_datastream.SocketPublisher'],
        'thermometer': ['morse.middleware.socket_datastream.SocketPublisher'],
        'semantic_camera' : ['morse.middleware.socket_datastream.SocketPublisher'],
        'video_camera' : ['morse.middleware.sockets.video_camera.VideoPublisher'],

        'armature': ['morse.middleware.socket_datastream.SocketReader'],
        'destination': ['morse.middleware.socket_datastream.SocketReader'],
        'gripper': ['morse.middleware.socket_datastream.SocketReader'],
        'kuka_lwr': ['morse.middleware.socket_datastream.SocketReader'],
        'light': ['morse.middleware.socket_datastream.SocketReader'],
        'orientation': ['morse.middleware.socket_datastream.SocketReader'],
        'ptu': ['morse.middleware.socket_datastream.SocketReader'],
        'rotorcraft_attitude' : ['morse.middleware.socket_datastream.SocketReader'],
        'rotorcraft_waypoint' : ['morse.middleware.socket_datastream.SocketReader'],
        'stabilized_quadrotor' : ['morse.middleware.socket_datastream.SocketReader'],
        'steer_force': ['morse.middleware.socket_datastream.SocketReader'],
        'v_omega': ['morse.middleware.socket_datastream.SocketReader'],
        'v_omega_diff_drive': ['morse.middleware.socket_datastream.SocketReader'],
        'xy_omega': ['morse.middleware.socket_datastream.SocketReader'],
        'waypoint': ['morse.middleware.socket_datastream.SocketReader'],
        'teleport': ['morse.middleware.socket_datastream.SocketReader'],
    },

    'yarp': {
        'accelerometer': ['post_message'],
        'armature_pose': ['post_message'],
        'battery': ['post_message'],
        'gps': ['post_message'],
        'gyroscope': ['post_message'],
        'imu': ['post_message'],
        'odometry': ['post_message'],
        'pose': ['post_message'],
        'proximity': ['post_dictionary_data', 'morse/middleware/yarp/dictionary'],
        'ptu_posture': ['post_message'],
        'rosace': ['post_message'],
        'semantic_camera': ['post_message'],
        'sick': ['post_sick_message', 'morse/middleware/yarp/sick'],
        'thermometer': ['post_message'],
        'video_camera': ['post_image_RGBA'],

        'armature': ['read_message'],
        'destination': ['read_message'],
        'gripper': ['read_message'],
        'light': ['read_message'],
        'orientation': ['read_message'],
        'ptu': ['read_message'],
        'steer_force': ['read_message'],
        'v_omega': ['read_message'],
        'v_omega_diff_drive': ['read_message'],
        'waypoint': ['read_message'],
        'teleport': ['read_message'],
    },

    'yarp_json': {
        'rosace': ['post_json_message', 'morse/middleware/yarp/json_mod'],
        'gps': ['post_json_message', 'morse/middleware/yarp/json_mod'],
        'proximity': ['post_json_message', 'morse/middleware/yarp/json_mod'],

        'waypoint': ['read_json_message', 'morse/middleware/yarp/json_mod'],
    },


    'pocolibs': {
        'stereo_unit': ['write_viam', 'morse/middleware/pocolibs/sensors/viam', 'viamMorseBench'],
        'ptu': ['read_platine', 'morse/middleware/pocolibs/actuators/platine', 'platine_orientation'],
        'armature': ['read_lwr_config', 'morse/middleware/pocolibs/actuators/lwr', 'lwrCurrentPoseArmRight'],
        'gyroscope': ['write_pom', 'morse/middleware/pocolibs/sensors/pom', 'MorseGyroscopeMEPos'],
        'odometry': ['write_pom', 'morse/middleware/pocolibs/sensors/pom', 'MorseOdometryMEPos'],
        'pose': ['write_pom', 'morse/middleware/pocolibs/sensors/pom', 'MorsePoseMEPos'],
        'GPS': ['write_pom', 'morse/middleware/pocolibs/sensors/pom', 'MorseGPSMEPos'],
        'v_omega': ['read_genpos', 'morse/middleware/pocolibs/actuators/genpos', 'simu_locoSpeedRef'],
        'v_omega_diff_drive': ['read_genpos', 'morse/middleware/pocolibs/actuators/genpos', 'simu_locoSpeedRef'],
        'semantic_camera': ['write_viman', 'morse/middleware/pocolibs/sensors/viman', 'morse_viman'],
        'human_posture': ['export_posture', 'morse/middleware/pocolibs/sensors/human_posture', 'human_posture'],
        'rosace': ['write_target', 'morse/middleware/pocolibs/sensors/target', 'targetPos'],
        'ptu_posture': ['write_platine_posture', 'morse/middleware/pocolibs/sensors/platine_posture', 'platineState'],
        'ik_control': ['read_niut_ik_positions', 'morse/middleware/pocolibs/actuators/niut', 'niutHuman'],
        'mocap_control': ['read_niut_ik_positions', 'morse/middleware/pocolibs/actuators/niut', 'niutHuman']
    },

    'text': {
        'accelerometer': ['write_data'],
        'battery': ['write_data'],
        'gps': ['write_data'],
        'gyroscope': ['write_data'],
        'imu': ['write_data'],
        'odometry': ['write_data'],
        'pose': ['write_data'],
        'proximity': ['write_data'],
        'ptu_posture': ['write_data'],
        'rosace': ['write_data'],
        'thermometer': ['write_data'],
    },

}


MORSE_SERVICE_DICT = {
    "socket": "morse.middleware.socket_request_manager.SocketRequestManager",
    "yarp": "morse.middleware.yarp_request_manager.YarpRequestManager",
    "yarp_json": "morse.middleware.yarp_json_request_manager.YarpRequestManager",
    "pocolibs": "morse.middleware.pocolibs_request_manager.PocolibsRequestManager",
    "ros": "morse.middleware.ros_request_manager.RosRequestManager",
}
