import os
from morse.builder.components import MORSE_COMPONENTS_DICT

"""
MORSE_COMPONENTS: 
path to the Morse components
"""

MORSE_COMPONENTS = os.path.join(os.environ["MORSE_ROOT"], "share", "data", "morse", "components")

"""
middleware-dictionary-convention:
{
  .blend-middleware: {
    .blend-component: ['MW', 'method', 'path']
  }
}
"""
MORSE_MIDDLEWARE_DICT = {
  'ros_empty': {
    'morse_vw_control': ['ROS', 'read_twist', 'morse/middleware/ros/read_vw_twist'],
    'morse_camera': ['ROS', 'post_image', 'morse/middleware/ros/camera'],
    'morse_sick': ['ROS', 'post_2DLaserScan', 'morse/middleware/ros/sick'], # TMP
    'morse_sick_180': ['ROS', 'post_2DLaserScan', 'morse/middleware/ros/sick'],
    'morse_sick_270': ['ROS', 'post_2DLaserScan', 'morse/middleware/ros/sick'],
    'morse_odometry': ['ROS', 'post_twist', 'morse/middleware/ros/odometry_sensor'],
    'morse_pose': ['ROS', 'post_odometry_transform', 'morse/middleware/ros/pose'],
    'morse_GPS': ['ROS', 'post_message'],
    'morse_gyroscope': ['ROS', 'post_message'],
    'morse_proximity': ['ROS', 'post_message']
  },

  'socket_empty': {
    'morse_gyroscope': ['Socket', 'post_message'],
    'morse_vw_control': ['Socket', 'read_message']
  },

  'yarp_empty': {
    'morse_camera': ['Yarp', 'post_image_RGBA'],
    'morse_sick': ['Yarp', 'post_sick_message', 'morse/middleware/yarp/sick'],
    'morse_proximity': ['Yarp', 'post_dictionary_data', 'morse/middleware/yarp/dictionary'],
    'morse_gyroscope': ['Yarp', 'post_message'],
    'morse_odometry': ['Yarp', 'post_message'],
    'morse_pose': ['Yarp', 'post_message'],
    'morse_GPS': ['Yarp', 'post_message'],

    'morse_vw_control': ['Yarp', 'read_message']
  }
}
