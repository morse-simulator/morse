import os

"""
MORSE_COMPONENTS: 
path to the Morse components
"""

MORSE_COMPONENTS = os.path.join(os.environ["MORSE_ROOT"], "share", "morse", "data")

"""
MORSE_MODIFIERS:
path to the modifiers modules
"""
MORSE_MODIFIERS = {
    'NED': 'morse.modifiers.ned.MorseNEDClass',
    'UTM': 'morse.modifiers.utm.MorseUTMClass',
    'GPSNoise': 'morse.modifiers.gps_noise.MorseGPSNoiseClass',
}

"""
MORSE_MIDDLEWARE_MODULE:
path to the middleware modules
"""
MORSE_MIDDLEWARE_MODULE = {
    'ros': 'morse.middleware.ros_mw.ROSClass',
    'socket': 'morse.middleware.socket_mw.MorseSocketClass',
    'yarp': 'morse.middleware.yarp_mw.MorseYarpClass',
    'pocolibs': 'morse.middleware.pocolibs_mw.MorsePocolibsClass',
    'text': 'morse.middleware.text_mw.TextOutClass',
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
    },
    'UTM' : {
        'pose': [MORSE_MODIFIERS['UTM'], 'blender_to_utm'],
        'gps': [MORSE_MODIFIERS['UTM'], 'blender_to_utm'],
        'destination': [MORSE_MODIFIERS['UTM'], 'utm_to_blender'],
        'waypoint': [MORSE_MODIFIERS['UTM'], 'utm_to_blender'],
    },
}

"""
middleware-dictionary-convention:
{
    .blend-middleware: {
        .blend-component: ['MW', 'method', 'path']
    }
}
"""
MORSE_MIDDLEWARE_DICT = {
    'ros': {
        'accelerometer': [MORSE_MIDDLEWARE_MODULE['ros'], 'post_twist', 'morse/middleware/ros/accelerometer'],
        'battery': [MORSE_MIDDLEWARE_MODULE['ros'], 'post_float32', 'morse/middleware/ros/battery'],
        'clock': [MORSE_MIDDLEWARE_MODULE['ros'], 'post_clock', 'morse/middleware/ros/clock'],
        'gps': [MORSE_MIDDLEWARE_MODULE['ros'], 'post_message'],
        'gyroscope': [MORSE_MIDDLEWARE_MODULE['ros'], 'post_message'],
        'infrared': [MORSE_MIDDLEWARE_MODULE['ros'], 'post_range', 'morse/middleware/ros/infrared'],
        'imu': [MORSE_MIDDLEWARE_MODULE['ros'], 'post_odometry', 'morse/middleware/ros/imu'],
        'light': [MORSE_MIDDLEWARE_MODULE['ros'], 'read_switch', 'morse/middleware/ros/light'],
        'pose': [MORSE_MIDDLEWARE_MODULE['ros'], 'post_odometry', 'morse/middleware/ros/pose'],
        'proximity': [MORSE_MIDDLEWARE_MODULE['ros'], 'post_message'],
        'pr2_posture': [MORSE_MIDDLEWARE_MODULE['ros'], 'post_jointState', 'morse/middleware/ros/pr2_posture'],
        'semantic_camera': [MORSE_MIDDLEWARE_MODULE['ros'], 'post_string', 'morse/middleware/ros/semantic_camera'],
        'sick': [MORSE_MIDDLEWARE_MODULE['ros'], 'post_2DLaserScan', 'morse/middleware/ros/sick'],
        'video_camera': [MORSE_MIDDLEWARE_MODULE['ros'], 'post_image', 'morse/middleware/ros/camera'],

        'light': [MORSE_MIDDLEWARE_MODULE['ros'], 'read_switch', 'morse/middleware/ros/light'],
        'ptu': [MORSE_MIDDLEWARE_MODULE['ros'], 'read_Vector3', 'morse/middleware/ros/platine'],
        'kuka_lwr': [MORSE_MIDDLEWARE_MODULE['ros'], 'read_joinState', 'morse/middleware/ros/kuka_joinState'],
        'v_omega': [MORSE_MIDDLEWARE_MODULE['ros'], 'read_twist', 'morse/middleware/ros/read_vw_twist'],
        'xy_omega': [MORSE_MIDDLEWARE_MODULE['ros'], 'read_twist', 'morse/middleware/ros/read_xyw_twist'], 
    },

    'socket': {
        'accelerometer': [MORSE_MIDDLEWARE_MODULE['socket'], 'post_message'],
        'battery': [MORSE_MIDDLEWARE_MODULE['socket'], 'post_message'],
        'gps': [MORSE_MIDDLEWARE_MODULE['socket'], 'post_message'],
        'gyroscope': [MORSE_MIDDLEWARE_MODULE['socket'], 'post_message'],
        'imu': [MORSE_MIDDLEWARE_MODULE['socket'], 'post_message'],
        'odometry': [MORSE_MIDDLEWARE_MODULE['socket'], 'post_message'],
        'pose': [MORSE_MIDDLEWARE_MODULE['socket'], 'post_message'],
        'proximity': [MORSE_MIDDLEWARE_MODULE['socket'], 'post_message'],
        'ptu_posture': [MORSE_MIDDLEWARE_MODULE['socket'], 'post_message'],
        'rosace': [MORSE_MIDDLEWARE_MODULE['socket'], 'post_message'],
        'thermometer': [MORSE_MIDDLEWARE_MODULE['socket'], 'post_message'],
        'semantic_camera' : [MORSE_MIDDLEWARE_MODULE['socket'], 'post_semantic_camera', 'morse/middleware/sockets/semantic_camera'],

        'destination': [MORSE_MIDDLEWARE_MODULE['socket'], 'read_message'],
        'gripper': [MORSE_MIDDLEWARE_MODULE['socket'], 'read_message'],
        'light': [MORSE_MIDDLEWARE_MODULE['socket'], 'read_message'],
        'orientation': [MORSE_MIDDLEWARE_MODULE['socket'], 'read_message'],
        'ptu': [MORSE_MIDDLEWARE_MODULE['socket'], 'read_message'],
        'steer_force': [MORSE_MIDDLEWARE_MODULE['socket'], 'read_message'],
        'v_omega': [MORSE_MIDDLEWARE_MODULE['socket'], 'read_message'],
        'waypoint': [MORSE_MIDDLEWARE_MODULE['socket'], 'read_message'],
    },

    'yarp': {
        'accelerometer': [MORSE_MIDDLEWARE_MODULE['yarp'], 'post_message'],
        'battery': [MORSE_MIDDLEWARE_MODULE['yarp'], 'post_message'],
        'gps': [MORSE_MIDDLEWARE_MODULE['yarp'], 'post_message'],
        'gyroscope': [MORSE_MIDDLEWARE_MODULE['yarp'], 'post_message'],
        'imu': [MORSE_MIDDLEWARE_MODULE['yarp'], 'post_message'],
        'odometry': [MORSE_MIDDLEWARE_MODULE['yarp'], 'post_message'],
        'pose': [MORSE_MIDDLEWARE_MODULE['yarp'], 'post_message'],
        'proximity': [MORSE_MIDDLEWARE_MODULE['yarp'], 'post_dictionary_data', 'morse/middleware/yarp/dictionary'],
        'ptu_posture': [MORSE_MIDDLEWARE_MODULE['yarp'], 'post_message'],
        'rosace': [MORSE_MIDDLEWARE_MODULE['yarp'], 'post_message'],
        'semantic_camera': [MORSE_MIDDLEWARE_MODULE['yarp'], 'post_message'],
        'sick': [MORSE_MIDDLEWARE_MODULE['yarp'], 'post_sick_message', 'morse/middleware/yarp/sick'],
        'thermometer': [MORSE_MIDDLEWARE_MODULE['yarp'], 'post_message'],
        'video_camera': [MORSE_MIDDLEWARE_MODULE['yarp'], 'post_image_RGBA'],

        'armature_actuator': [MORSE_MIDDLEWARE_MODULE['yarp'], 'read_message'],
        'destination': [MORSE_MIDDLEWARE_MODULE['yarp'], 'read_message'],
        'gripper': [MORSE_MIDDLEWARE_MODULE['yarp'], 'read_message'],
        'kuka_lwr': [MORSE_MIDDLEWARE_MODULE['yarp'], 'read_message'],
        'light': [MORSE_MIDDLEWARE_MODULE['yarp'], 'read_message'],
        'orientation': [MORSE_MIDDLEWARE_MODULE['yarp'], 'read_message'],
        'ptu': [MORSE_MIDDLEWARE_MODULE['yarp'], 'read_message'],
        'steer_force': [MORSE_MIDDLEWARE_MODULE['yarp'], 'read_message'],
        'v_omega': [MORSE_MIDDLEWARE_MODULE['yarp'], 'read_message'],
        'waypoint': [MORSE_MIDDLEWARE_MODULE['yarp'], 'read_message'],
    },

    'yarp_json': {
        'rosace': [MORSE_MIDDLEWARE_MODULE['yarp'], 'post_json_message', 'morse/middleware/yarp/json_mod'],
        'gps': [MORSE_MIDDLEWARE_MODULE['yarp'], 'post_json_message', 'morse/middleware/yarp/json_mod'],
        'proximity': [MORSE_MIDDLEWARE_MODULE['yarp'], 'post_json_message', 'morse/middleware/yarp/json_mod'],

        'waypoint': [MORSE_MIDDLEWARE_MODULE['yarp'], 'read_json_message', 'morse/middleware/yarp/json_mod'],
    },


    'pocolibs': {
        'stereo_unit': [MORSE_MIDDLEWARE_MODULE['pocolibs'], 'write_viam', 'morse/middleware/pocolibs/sensors/viam', 'viamMorseBench'],
        'ptu': [MORSE_MIDDLEWARE_MODULE['pocolibs'], 'read_platine', 'morse/middleware/pocolibs/actuators/platine', 'platine_orientation'],
        'kuka_lwr': [MORSE_MIDDLEWARE_MODULE['pocolibs'], 'read_lwr_config', 'morse/middleware/pocolibs/actuators/lwr', 'lwrCurrentPoseArmRight'],
        'gyroscope': [MORSE_MIDDLEWARE_MODULE['pocolibs'], 'write_pom', 'morse/middleware/pocolibs/sensors/pom', 'MorseMEPos'],
        'v_omega': [MORSE_MIDDLEWARE_MODULE['pocolibs'], 'read_genpos', 'morse/middleware/pocolibs/actuators/genpos', 'simu_locoSpeedRef'],
        'semantic_camera': [MORSE_MIDDLEWARE_MODULE['pocolibs'], 'write_viman', 'morse/middleware/pocolibs/sensors/viman', 'morse_viman'],
        'human_posture': [MORSE_MIDDLEWARE_MODULE['pocolibs'], 'export_posture', 'morse/middleware/pocolibs/sensors/human_posture', 'human_posture'],
        'rosace': [MORSE_MIDDLEWARE_MODULE['pocolibs'], 'write_target', 'morse/middleware/pocolibs/sensors/target', 'targetPos'],
        'ptu_posture': [MORSE_MIDDLEWARE_MODULE['pocolibs'], 'write_platine_posture', 'morse/middleware/pocolibs/sensors/platine_posture', 'platineState']
    },

    'text': {
        'accelerometer': [MORSE_MIDDLEWARE_MODULE['text'], 'write_data'],
        'battery': [MORSE_MIDDLEWARE_MODULE['text'], 'write_data'],
        'gps': [MORSE_MIDDLEWARE_MODULE['text'], 'write_data'],
        'gyroscope': [MORSE_MIDDLEWARE_MODULE['text'], 'write_data'],
        'imu': [MORSE_MIDDLEWARE_MODULE['text'], 'write_data'],
        'odometry': [MORSE_MIDDLEWARE_MODULE['text'], 'write_data'],
        'pose': [MORSE_MIDDLEWARE_MODULE['text'], 'write_data'],
        'proximity': [MORSE_MIDDLEWARE_MODULE['text'], 'write_data'],
        'ptu_posture': [MORSE_MIDDLEWARE_MODULE['text'], 'write_data'],
        'rosace': [MORSE_MIDDLEWARE_MODULE['text'], 'write_data'],
        'thermometer': [MORSE_MIDDLEWARE_MODULE['text'], 'write_data'],
    },


}


MORSE_SERVICE_DICT = {
    "socket": "morse.middleware.socket_request_manager.SocketRequestManager",
    "yarp": "morse.middleware.yarp_request_manager.YarpRequestManager",
    "yarp_json": "morse.middleware.yarp_json_request_manager.YarpRequestManager",
    "pocolibs": "morse.middleware.pocolibs_request_manager.PocolibsRequestManager",
    "ros": "morse.middleware.ros_request_manager.RosRequestManager",
}
