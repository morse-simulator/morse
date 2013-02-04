from morse.builder import *

# Land robot
ATRV = ATRV()
ATRV.translate(z=0.1000)

Platine = PTU()
Platine.translate(x=0.2000, z=0.9000)
ATRV.append(Platine)

Stereo = StereoUnit()
Stereo.translate(z=0.0400)
Platine.append(Stereo)

CameraL = VideoCamera()
CameraL.translate(x=0.1000, y=0.2000, z=0.0700)
Stereo.append(CameraL)
CameraL.properties(capturing = True)
CameraL.properties(cam_width = 320)
CameraL.properties(cam_height = 240)
CameraL.properties(cam_focal = 25.0000)

CameraR = VideoCamera()
CameraR.translate(x=0.1000, y=-0.2000, z=0.0700)
Stereo.append(CameraR)
CameraR.properties(capturing = True)
CameraR.properties(cam_width = 320)
CameraR.properties(cam_height = 240)
CameraR.properties(cam_focal = 25.0000)

GPS = GPS()
GPS.translate(x=-0.2000, z=0.9000)
ATRV.append(GPS)

Gyroscope = Gyroscope()
Gyroscope.translate(z=0.9000)
ATRV.append(Gyroscope)
Gyroscope.properties(reference_frame = 'pomGPSFramePos')
Gyroscope.properties(confidence = 0.1000)

Motion_Controller_001 = MotionVW()
ATRV.append(Motion_Controller_001)

# Helicopter robot
RMax = RMax()
RMax.translate(x=5.0, y=0.0, z=9.0)

GPS_001 = GPS()
RMax.append(GPS_001)

CameraMain_002 = VideoCamera()
CameraMain_002.translate(x=1.4000)
CameraMain_002.rotate(x=-1.5708)
RMax.append(CameraMain_002)
CameraMain_002.properties(capturing = True)
CameraMain_002.properties(cam_width = 512)
CameraMain_002.properties(cam_height = 512)
CameraMain_002.properties(cam_focal = 35.0000)

Gyroscope_001 = Gyroscope()
Gyroscope_001.translate(x=0.8000, z=1.1000)
RMax.append(Gyroscope_001)

Motion_Controller = Destination()
RMax.append(Motion_Controller)


# Scene configuration
GPS_001.add_stream('yarp')
CameraMain_002.add_stream('yarp')
Motion_Controller.add_stream('yarp')
Gyroscope_001.add_stream('yarp')

Platine.add_stream('yarp')

Motion_Controller_001.add_stream('pocolibs')
Stereo.add_stream('pocolibs')
Gyroscope.add_stream('pocolibs')

GPS_001.configure_modifier('NED')
Motion_Controller.configure_modifier('NED')
Gyroscope_001.configure_modifier('NED')

#env = Environment('land-1/buildings_2')
env = Environment('land-1/trees')
env.aim_camera([1.0470, 0, 0.7854])
