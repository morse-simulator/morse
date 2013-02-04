from morse.builder import *

def equipped_robot(mw='yarp'):
    # Append ATRV robot to the scene
    atrv = ATRV()

    # Append an actuator
    motion = MotionVW()
    atrv.append(motion)

    # Append a Pose sensor (GPS + Gyroscope)
    pose = Pose()
    pose.translate(x=0.2,z=0.83)
    atrv.append(pose)

    # Append a sick laser
    sick = Sick()
    sick.translate(x=0.18,z=0.94)
    atrv.append(sick)
    sick.properties(resolution = 1)
    sick.properties(laser_range = 5.0)

    # Append a camera
    cam = VideoCamera()
    cam.translate(x=0.3,z=1.1)
    atrv.append(cam)
    cam.properties(cam_width = 128, cam_height = 128)

    # Configure the middlewares
    motion.add_stream(mw)
    pose.add_stream(mw)
    sick.add_stream(mw)
    cam.add_stream(mw)

    return (atrv)
