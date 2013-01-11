from morse.builder import *

# Append ATRV robot to the scene
atrv = Robot('atrv')

# Append an actuator
motion = Actuator('v_omega')
motion.translate(z=0.3)
motion.name = 'Motion'
atrv.append(motion)
motion.configure_mw('ros')

# Append an Odometry sensor
odometry = Sensor('odometry')
odometry.name = 'Odometry'
odometry.translate(z=0.73)
atrv.append(odometry)
odometry.configure_mw('ros')

# Append a camera
camera = Sensor('depth_camera')
camera.name = 'DepthCamera'
camera.translate(x = 0.3, z = 0.76)
camera.properties(cam_width = 640, cam_height = 480,
                  cam_near = 1.0, cam_far = 20.0,
                  capturing = True, Depth = True)
camera.frequency(15)
atrv.append(camera)
camera.configure_mw('ros')

env = Environment('indoors-1/indoor-1')
env.aim_camera([1.0470, 0, 0.7854])
env.show_framerate()
env.profile(camera)
env.profile(odometry)
