from morse.builder import *

# Land robot
atrv = ATRV()
atrv.translate(z=0.1000)

platine = PTU()
platine.translate(x=0.2000, z=0.9000)
atrv.append(platine)

stereo = StereoUnit()
stereo.translate(z=0.0400)
platine.append(stereo)

CameraL = VideoCamera()
CameraL.translate(x=0.1000, y=0.2000, z=0.0700)
stereo.append(CameraL)
CameraL.properties(capturing = True)
CameraL.properties(cam_width = 320)
CameraL.properties(cam_height = 240)
CameraL.properties(cam_focal = 25.0000)

CameraR = VideoCamera()
CameraR.translate(x=0.1000, y=-0.2000, z=0.0700)
stereo.append(CameraR)
CameraR.properties(capturing = True)
CameraR.properties(cam_width = 320)
CameraR.properties(cam_height = 240)
CameraR.properties(cam_focal = 25.0000)

gps = GPS()
gps.translate(x=-0.2000, z=0.9000)
atrv.append(gps)

gyroscope = Gyroscope()
gyroscope.translate(z=0.9000)
atrv.append(gyroscope)
gyroscope.properties(reference_frame = 'pomGPSFramePos')
gyroscope.properties(confidence = 0.1000)

motion = MotionVW()
atrv.append(motion)

# Helicopter robot
rmax = RMax()
rmax.translate(x=5.0, y=0.0, z=9.0)

gps01 = GPS()
rmax.append(gps01)

camera02 = VideoCamera()
camera02.translate(x=1.4000)
camera02.rotate(x=-1.5708)
rmax.append(camera02)
camera02.properties(capturing = True)
camera02.properties(cam_width = 512)
camera02.properties(cam_height = 512)
camera02.properties(cam_focal = 35.0000)

gyroscope01 = Gyroscope()
gyroscope01.translate(x=0.8000, z=1.1000)
rmax.append(gyroscope01)

motion01 = Destination()
rmax.append(motion01)


# Scene configuration
gps01.add_stream('yarp')
camera02.add_stream('yarp')
motion01.add_stream('yarp')
gyroscope01.add_stream('yarp')

platine.add_stream('yarp')

motion.add_stream('pocolibs')
stereo.add_stream('pocolibs')
gyroscope.add_stream('pocolibs')

gps01.configure_modifier('NED')
motion01.configure_modifier('NED')
gyroscope01.configure_modifier('NED')

#env = Environment('land-1/buildings_2')
env = Environment('land-1/trees')
env.aim_camera([1.0470, 0, 0.7854])
