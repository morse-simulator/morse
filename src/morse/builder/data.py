import os

"""
MORSE_COMPONENTS: 
path to the Morse components
"""

MORSE_COMPONENTS = os.path.join(os.environ["MORSE_ROOT"], "share", "data", "morse")

"""
MORSE_MW_MODULES:
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
middleware-dictionary-convention:
{
  .blend-middleware: {
    .blend-component: ['MW', 'method', 'path']
  }
}
"""
MORSE_MIDDLEWARE_DICT = {
  'ros': {
    'v_omega': [MORSE_MIDDLEWARE_MODULE['ros'], 'read_twist', 'morse/middleware/ros/read_vw_twist'],
    'video_camera': [MORSE_MIDDLEWARE_MODULE['ros'], 'post_image', 'morse/middleware/ros/camera'],
    'sick': [MORSE_MIDDLEWARE_MODULE['ros'], 'post_2DLaserScan', 'morse/middleware/ros/sick'], # TMP
    'odometry': [MORSE_MIDDLEWARE_MODULE['ros'], 'post_twist', 'morse/middleware/ros/odometry_sensor'],
    'pose': [MORSE_MIDDLEWARE_MODULE['ros'], 'post_odometry_transform', 'morse/middleware/ros/pose'],
    'gps': [MORSE_MIDDLEWARE_MODULE['ros'], 'post_message'],
    'gyroscope': [MORSE_MIDDLEWARE_MODULE['ros'], 'post_message'],
    'proximity': [MORSE_MIDDLEWARE_MODULE['ros'], 'post_message'],
  },

  'socket': {
    'accelerometer': [MORSE_MIDDLEWARE_MODULE['socket'], 'post_message'],
    'gyroscope': [MORSE_MIDDLEWARE_MODULE['socket'], 'post_message'],
    'gps': [MORSE_MIDDLEWARE_MODULE['socket'], 'post_message'],
    'odometry': [MORSE_MIDDLEWARE_MODULE['socket'], 'post_message'],
    'pose': [MORSE_MIDDLEWARE_MODULE['socket'], 'post_message'],
    'thermometer': [MORSE_MIDDLEWARE_MODULE['socket'], 'post_message'],

    'destination': [MORSE_MIDDLEWARE_MODULE['socket'], 'read_message'],
    'orientation': [MORSE_MIDDLEWARE_MODULE['socket'], 'read_message'],
    'v_omega': [MORSE_MIDDLEWARE_MODULE['socket'], 'read_message'],
    'waypoint': [MORSE_MIDDLEWARE_MODULE['socket'], 'read_message'],
  },

  'yarp': {
    'video_camera': [MORSE_MIDDLEWARE_MODULE['yarp'], 'post_image_RGBA'],
    'sick': [MORSE_MIDDLEWARE_MODULE['yarp'], 'post_sick_message', 'morse/middleware/yarp/sick'],
    'proximity': [MORSE_MIDDLEWARE_MODULE['yarp'], 'post_dictionary_data', 'morse/middleware/yarp/dictionary'],
    'accelerometer': [MORSE_MIDDLEWARE_MODULE['yarp'], 'post_message'],
    'gyroscope': [MORSE_MIDDLEWARE_MODULE['yarp'], 'post_message'],
    'odometry': [MORSE_MIDDLEWARE_MODULE['yarp'], 'post_message'],
    'pose': [MORSE_MIDDLEWARE_MODULE['yarp'], 'post_message'],
    'gps': [MORSE_MIDDLEWARE_MODULE['yarp'], 'post_message'],
    'thermometer': [MORSE_MIDDLEWARE_MODULE['yarp'], 'post_message'],

    'destination': [MORSE_MIDDLEWARE_MODULE['yarp'], 'read_message'],
    'orientation': [MORSE_MIDDLEWARE_MODULE['yarp'], 'read_message'],
    'ptu': [MORSE_MIDDLEWARE_MODULE['yarp'], 'read_message'],
    'v_omega': [MORSE_MIDDLEWARE_MODULE['yarp'], 'read_message'],
    'waypoint': [MORSE_MIDDLEWARE_MODULE['yarp'], 'read_message'],
    'steer_force': [MORSE_MIDDLEWARE_MODULE['yarp'], 'read_message'],
  },

  'pocolibs': {
    'stereo_unit': [MORSE_MIDDLEWARE_MODULE['pocolibs'], 'write_viam', 'morse/middleware/pocolibs/sensors/viam', 'viamMorseBench'],
    'gyroscope': [MORSE_MIDDLEWARE_MODULE['pocolibs'], 'write_pom', 'morse/middleware/pocolibs/sensors/pom', 'MorseMEPos'],

    'v_omega': [MORSE_MIDDLEWARE_MODULE['pocolibs'], 'read_genpos', 'morse/middleware/pocolibs/actuators/genpos', 'simu_locoSpeedRef'],
  },
}


MORSE_SERVICE_DICT = {
    "socket": "morse.middleware.socket_request_manager.SocketRequestManager",
    "yarp": "morse.middleware.yarp_request_manager.YarpRequestManager",
    "yarp_json": "morse.middleware.yarp_json_request_manager.YarpRequestManager",
    "pocolibs": "morse.middleware.pocolibs_request_manager.PocolibsRequestManager",
}
