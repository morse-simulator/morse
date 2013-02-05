from morse.builder import *

# Append ATRV robot to the scene
atrv = ATRV()

# Append an actuator
motion = MotionVW()
motion.translate(z=0.3)
motion.name = 'Motion'
atrv.append(motion)
motion.add_stream('ros')

# Append an Odometry sensor
odometry = Odometry()
odometry.name = 'Odometry'
odometry.translate(z=0.73)
atrv.append(odometry)
odometry.add_stream('ros')

# Append a camera
camera = DepthCamera()
camera.name = 'DepthCamera'
camera.translate(x = 0.3, z = 0.76)
camera.properties(cam_width = 640, cam_height = 480,
                  cam_near = 1.0, cam_far = 20.0,
                  capturing = True, Depth = True)
camera.frequency(15)
atrv.append(camera)
camera.add_stream('ros')

env = Environment('indoors-1/indoor-1')
env.aim_camera([1.0470, 0, 0.7854])
env.show_framerate()
camera.profile()
odometry.profile()
