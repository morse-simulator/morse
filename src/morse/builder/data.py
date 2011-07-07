import os
from morse.builder.components import MORSE_COMPONENTS_DICT

"""
MORSE_COMPONENTS: 
path to the Morse components
"""
root = "/usr/local" # default install
if 'MORSE_ROOT' in os.environ:
  root = os.environ['MORSE_ROOT']
elif b'MORSE_ROOT' in os.environ: # python3.2 , Blender 2.57+
  root = os.environ[b'MORSE_ROOT'] # root = "b'/usr/local'" <class 'str'>
  root = eval(root) # XXX root = b'/usr/local' <class 'bytes'>
  root = root.decode() # root = '/usr/local' <class 'str'>
MORSE_COMPONENTS = os.path.join(root, 
    "share/data/morse/components")

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
    'morse_sick_180': ['ROS', 'post_2DLaserScan', 'morse/middleware/ros/sick'],
    'morse_sick_270': ['ROS', 'post_2DLaserScan', 'morse/middleware/ros/sick'],
    'morse_odometry': ['ROS', 'post_twist', 'morse/middleware/ros/odometry_sensor'],
    'morse_pose': ['ROS', 'post_odometry_transform', 'morse/middleware/ros/pose'],
    'morse_GPS': ['ROS', 'post_message'],
    'morse_gyroscope': ['ROS', 'post_message'],
    'morse_proximity': ['ROS', 'post_message'],
    'morse_light': ["ROS", "read_switch", "morse/middleware/ros/light"],
    'morse_battery': ["ROS", "post_float32", "morse/middleware/ros/battery"],
    'morse_infrared': ["ROS", "post_range", "morse/middleware/ros/infrared"]
  },
  'socket_empty': {
    'morse_gyroscope': ['Socket', 'post_message'],
    'morse_vw_control': ['Socket', 'read_message']
  }
}

