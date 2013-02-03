from morse.builder import *

# Append ATRV robot to the scene
robot = ATRV()

# Append an actuator
motion = MotionVW()
robot.append(motion)

# Append an odometry sensor
odometry = Odometry()
odometry.translate(z = 0.75)
robot.append(odometry)

# Append a Sick LIDAR
sick = Sick()
sick.translate(x = 0.17, z = 0.83)
robot.append(sick)
# Set the scanner properties
sick.properties(scan_window = 180, resolution = 1, range = 10)
# Create the laser arc with those properties
sick.create_laser_arc()
# Lower the frequency, in our demo we don't need a lot of scan, free some CPU
sick.frequency(5)

# Append a Camera (for the show)
camera = VideoCamera()
robot.append(camera)
camera.translate(x = .34, z = .72)
# Lower the frequency, in our demo we don't need a lot of image, free some CPU
camera.frequency(5)

# Auto-configure all robot's components for ROS
robot.add_default_interface('ros')
robot.rotate(z = 2.1)
robot.translate(x = 2)

# Select the environement
env = Environment('indoors-1/indoor-1')
# Set the Camera aim (where the simulation is looking at the beginning)
env.aim_camera([1.0470, 0, 0.7854])
# Show some debug
env.show_framerate()
camera.profile()
sick.profile()

# Save the file to let Travis know the script was successfully built
env.save("travis-succeed.blend", True)
