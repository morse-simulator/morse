from morse.builder import *

# Land robot
ATRV = Robot('atrv')
ATRV.translate(z=0.1000)

Platine = Actuator('ptu')
Platine.translate(x=0.2000, z=0.9000)
ATRV.append(Platine)

Stereo = Sensor('stereo_unit')
Stereo.translate(z=0.0400)
Platine.append(Stereo)

CameraL = Sensor('video_camera')
CameraL.translate(x=0.1000, y=0.2000, z=0.0700)
Stereo.append(CameraL)
CameraL.properties(capturing = True)
CameraL.properties(cam_width = 320)
CameraL.properties(cam_height = 240)
CameraL.properties(cam_focal = 25.0000)

CameraR = Sensor('video_camera')
CameraR.translate(x=0.1000, y=-0.2000, z=0.0700)
Stereo.append(CameraR)
CameraR.properties(capturing = True)
CameraR.properties(cam_width = 320)
CameraR.properties(cam_height = 240)
CameraR.properties(cam_focal = 25.0000)

GPS = Sensor('gps')
GPS.translate(x=-0.2000, z=0.9000)
ATRV.append(GPS)

Gyroscope = Sensor('gyroscope')
Gyroscope.translate(z=0.9000)
ATRV.append(Gyroscope)
Gyroscope.properties(reference_frame = 'pomGPSFramePos')
Gyroscope.properties(confidence = 0.1000)

Motion_Controller_001 = Actuator('v_omega')
ATRV.append(Motion_Controller_001)

# Helicopter robot
RMax = Robot('rmax')
RMax.translate(x=5.0, y=0.0, z=9.0)

GPS_001 = Sensor('gps')
RMax.append(GPS_001)

CameraMain_002 = Sensor('video_camera')
CameraMain_002.translate(x=1.4000)
CameraMain_002.rotate(x=-1.5708)
RMax.append(CameraMain_002)
CameraMain_002.properties(capturing = True)
CameraMain_002.properties(cam_width = 512)
CameraMain_002.properties(cam_height = 512)
CameraMain_002.properties(cam_focal = 35.0000)

Gyroscope_001 = Sensor('gyroscope')
Gyroscope_001.translate(x=0.8000, z=1.1000)
RMax.append(Gyroscope_001)

Motion_Controller = Actuator('destination')
RMax.append(Motion_Controller)


# Scene configuration
GPS_001.configure_mw('yarp')
CameraMain_002.configure_mw('yarp')
Motion_Controller.configure_mw('yarp')
Gyroscope_001.configure_mw('yarp')

Platine.configure_mw('yarp')

Motion_Controller_001.configure_mw('pocolibs')
Stereo.configure_mw('pocolibs')
Gyroscope.configure_mw('pocolibs')

GPS_001.configure_modifier('NED')
Motion_Controller.configure_modifier('NED')
Gyroscope_001.configure_modifier('NED')

#env = Environment('land-1/buildings_2')
env = Environment('land-1/trees')
env.aim_camera([1.0470, 0, 0.7854])
