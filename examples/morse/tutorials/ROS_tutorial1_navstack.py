from morse.builder.morsebuilder import *

#Human
Human = Robot('human')
Human.translate(x=-5.3009)
Human.properties(Speed = 0.0100)

# Robot
Jido = Robot('jido')
Jido.translate(x=-5)

# Components
Gyroscope = Sensor('gyroscope')
Jido.append(Gyroscope)

IMU = Sensor('imu')
Jido.append(IMU)

jido_posture = Sensor('jido_posture')
Jido.append(jido_posture)
jido_posture.properties(KUKAname = 'kuka_armature')
jido_posture.properties(PTUname = 'PTU')

keyboard_control = Actuator('keyboard')
keyboard_control.name = 'keyboard_control'
Jido.append(keyboard_control)

kuka_base = Actuator('kuka_lwr')
kuka_base.translate(x=0.1850, y=0.2000, z=0.9070)
kuka_base.rotate(x=1.5708, y=0) 
Jido.append(kuka_base)

Motion_Controller = Actuator('v_omega')
Jido.append(Motion_Controller)

Odometry = Sensor('odometry')
Jido.append(Odometry)

Platine = Actuator('ptu')
Platine.translate(x=0.25, y=0, z=1.336)
Jido.append(Platine)
Platine.properties(Speed = 1.0000)
Platine.properties(Manual = False)

Pose_sensor = Sensor('pose')
Pose_sensor.name = 'Pose_sensor'
Jido.append(Pose_sensor)

SemanticCamera = Sensor('semantic_camera')
SemanticCamera.translate(x=0.065, z=0.06)
Platine.append(SemanticCamera)

Sick = Sensor('sick')
Sick.translate(x=0.37, z=0.3)
Jido.append(Sick)
Sick.properties(Visible_arc = False)
Sick.properties(laser_range = 30.0000)
Sick.properties(resolution = 1.0000)
Sick.properties(scan_window = 180.0000)

HandLocation = Sensor('pose')
HandLocation.name = 'Handlocation'
kuka_base.append(HandLocation)

# NOT YET WORKING:

#VideoCamera = Sensor('video_camera')
#VideoCamera.translate(x=0.065, z=0.06)
#VideoCamera.name = 'JidoCam'
#Platine.append(VideoCamera)

#Pose = Sensor('pose')
#Pose.translate(x=-5.3203)
#Human.append(Pose)

#Object_tracker = Sensor('object_tracker')
#Jido.append(Object_tracker)

# Scene configuration
#Object_tracker.configure_mw('ros', ['ROS', 'post_lisp_code', 'morse/middleware/ros/object_tracker'])
#SemanticCamera.configure_mw('ros', ['ROS', 'post_lisp_code', 'morse/middleware/ros/semantic_camera'])
#Pose.configure_mw('ros', ['ROS', 'post_poseStamped', 'morse/middleware/ros/pose'])

Pose_sensor.configure_mw('ros')
#IMU.configure_mw('ros') # does NOT work
IMU.configure_mw('ros', ['ROS', 'post_velocity_twist', 'morse/middleware/ros/imu'])
Sick.configure_mw('ros')
Motion_Controller.configure_mw('ros')
HandLocation.configure_mw('ros')
kuka_base.configure_mw('ros', ['ROS', 'read_jointState', 'morse/middleware/ros/kuka_jointState'])
Platine.configure_mw('ros')
#VideoCamera.configure_mw('ros')
jido_posture.configure_mw('ros', ['ROS', 'post_jointState', 'morse/middleware/ros/jido_posture'])

env = Environment('indoors-1/indoor-1')
