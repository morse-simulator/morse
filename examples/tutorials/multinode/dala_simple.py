from morse.builder import *

def equipped_robot(mw='yarp'):
    # Append ATRV robot to the scene
    atrv = Robot('atrv')

    # Append an actuator
    motion = Actuator('v_omega')
    atrv.append(motion)

    # Append a Pose sensor (GPS + Gyroscope)
    pose = Sensor('pose')
    pose.translate(x=0.2,z=0.83)
    atrv.append(pose)

    # Append a sick laser
    sick = Sensor('sick')
    sick.translate(x=0.18,z=0.94)
    atrv.append(sick)
    sick.properties(resolution = 1)
    sick.properties(laser_range = 5.0)

    # Append a camera
    cam = Sensor('video_camera')
    cam.translate(x=0.3,z=1.1)
    atrv.append(cam)
    cam.properties(cam_width = 128, cam_height = 128)

    # Configure the middlewares
    motion.configure_mw(mw)
    pose.configure_mw(mw)
    sick.configure_mw(mw)
    cam.configure_mw(mw)

    return (atrv)
