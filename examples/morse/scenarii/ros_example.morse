import morse.builder.morsebuilder as morse

# Test the API
# http://www.openrobots.org/morse/doc/latest/user/tutorial.html

# Add ATRV robot to the scene
atrv = morse.Robot('atrv')

# Link an actuator
motion = morse.Controller('morse_vw_control')
motion.location = (0, 0, 0.3)
atrv.append(motion)

# Link a Gyroscope sensor
gyroscope = morse.Sensor('morse_gyroscope')
gyroscope.location = (0, 0, 0.83)
atrv.append(gyroscope)

# Insert the middleware object
socket = morse.Middleware('socket_empty')
ros = morse.Middleware('ros_empty')

conf = morse.Config()
# Associate each component to a middleware method
conf.link(gyroscope, ['ROS', 'post_message'])
conf.link(motion, ['ROS', 'read_twist', 'morse/middleware/ros/read_vw_twist'])

conf.init()


"""
TODO easier way to link component to middleware, like:
  conf.linkV2(gyroscope, morse.middleware.ros.post_message)
  conf.linkV2(motion, morse.middleware.ros.read_vw_twist)

TIPS use Blender in debug mode (-d) to watch bpy calls
  blender -d 2>/dev/null | grep bpy
"""

