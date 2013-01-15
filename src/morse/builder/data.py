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
        'accelerometer': ['post_twist', 'morse/middleware/ros/accelerometer'],
        'battery': ['post_float32', 'morse/middleware/ros/battery'],
        'clock': ['post_clock', 'morse/middleware/ros/clock'],
        'gps': ['post_message'],
        'gyroscope': ['post_message'],
        'infrared': ['post_range', 'morse/middleware/ros/infrared'],
        'imu': ['post_imu', 'morse/middleware/ros/imu'],
        'light': ['read_switch', 'morse/middleware/ros/light'],
        'odometry': ['post_odometry', 'morse/middleware/ros/odometry'],
        'pose': ['post_odometry', 'morse/middleware/ros/pose'],
        'proximity': ['post_message'],
        'pr2_posture': ['post_jointState', 'morse/middleware/ros/pr2_posture'],
        'semantic_camera': ['post_string', 'morse/middleware/ros/semantic_camera'],
        'sick': ['post_2DLaserScan', 'morse/middleware/ros/sick'],
        'video_camera': ['post_image', 'morse/middleware/ros/camera'],
        'depth_camera': ['post_pointcloud2', 'morse/middleware/ros/depth_camera'],

        'light': ['read_switch', 'morse/middleware/ros/light'],
        'ptu': ['read_Vector3', 'morse/middleware/ros/platine'],
        #'kuka_lwr': ['read_jointState', 'morse/middleware/ros/kuka_jointState'],
        'v_omega': ['read_twist', 'morse/middleware/ros/read_vw_twist'],
        'v_omega_diff_drive': ['read_twist', 'morse/middleware/ros/read_vw_twist'],
        'xy_omega': ['read_twist', 'morse/middleware/ros/read_xyw_twist'],
        'destination': ['read_point', 'morse/middleware/ros/destination'],
        'force_torque': ['read_wrench', 'morse/middleware/ros/read_wrench'],
        'orientation': ['read_quaternion', 'morse/middleware/ros/read_quaternion'],
        'teleport': ['read_pose', 'morse/middleware/ros/read_pose'],
        'rotorcraft_waypoint': ['read_pose', 'morse/middleware/ros/read_pose'],
    },

    'socket': {
        'accelerometer': ['post_message'],
        'armature_pose': ['post_message'],
        'battery': ['post_message'],
        'gps': ['post_message'],
        'gyroscope': ['post_message'],
        'imu': ['post_message'],
        'odometry': ['post_message'],
        'pose': ['post_message'],
        'proximity': ['post_message'],
        'ptu_posture': ['post_message'],
        'rosace': ['post_message'],
        'thermometer': ['post_message'],
        'semantic_camera' : ['post_message'],
        'video_camera' : ['post_video_camera', 'morse/middleware/sockets/video_camera'],

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
