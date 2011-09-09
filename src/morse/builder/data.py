import os
from morse.builder.components import MORSE_COMPONENTS_DICT

"""
MORSE_COMPONENTS: 
path to the Morse components
"""

MORSE_COMPONENTS = os.path.join(os.environ["MORSE_ROOT"], "share", "data", "morse")

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
    'v_omega': ['ROS', 'read_twist', 'morse/middleware/ros/read_vw_twist'],
    'video_camera': ['ROS', 'post_image', 'morse/middleware/ros/camera'],
    'sick': ['ROS', 'post_2DLaserScan', 'morse/middleware/ros/sick'], # TMP
    'odometry': ['ROS', 'post_twist', 'morse/middleware/ros/odometry_sensor'],
    'pose': ['ROS', 'post_odometry_transform', 'morse/middleware/ros/pose'],
    'gps': ['ROS', 'post_message'],
    'gyroscope': ['ROS', 'post_message'],
    'proximity': ['ROS', 'post_message'],
  },

  'socket': {
    'accelerometer': ['Socket', 'post_message'],
    'gyroscope': ['Socket', 'post_message'],
    'gps': ['Socket', 'post_message'],
    'odometry': ['Socket', 'post_message'],
    'pose': ['Socket', 'post_message'],
    'thermometer': ['Socket', 'post_message'],

    'destination': ['Socket', 'read_message'],
    'orientation': ['Socket', 'read_message'],
    'v_omega': ['Socket', 'read_message'],
    'waypoint': ['Socket', 'read_message'],
  },

  'yarp': {
    'video_camera': ['Yarp', 'post_image_RGBA'],
    'sick': ['Yarp', 'post_sick_message', 'morse/middleware/yarp/sick'],
    'proximity': ['Yarp', 'post_dictionary_data', 'morse/middleware/yarp/dictionary'],
    'accelerometer': ['Yarp', 'post_message'],
    'gyroscope': ['Yarp', 'post_message'],
    'odometry': ['Yarp', 'post_message'],
    'pose': ['Yarp', 'post_message'],
    'gps': ['Yarp', 'post_message'],
    'thermometer': ['Yarp', 'post_message'],

    'destination': ['Yarp', 'read_message'],
    'orientation': ['Yarp', 'read_message'],
    'v_omega': ['Yarp', 'read_message'],
    'waypoint': ['Yarp', 'read_message'],
    'steer_force': ['Yarp', 'read_message'],
  }
}


MORSE_SERVICE_DICT = {
    "socket": "morse.middleware.socket_request_manager.SocketRequestManager",
    "yarp": "morse.middleware.yarp_request_manager.YarpRequestManager",
    "pocolibs": "morse.middleware.pocolibs_request_manager.PocolibsRequestManager",
}
