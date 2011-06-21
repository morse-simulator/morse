from morse.builder.components import MORSE_COMPONENTS_DICT

"""
MORSE_COMPONENTS: 
path to the Morse components
"""

# XXX Hardcoded PATH, must be fixed
MORSE_COMPONENTS = '/usr/local/share/data/morse/components'

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
  }
}

