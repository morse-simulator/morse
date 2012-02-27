from morse.builder.morsebuilder import *

# Robot
Jido = Robot('jido')
Jido.translate(x=-5)

IMU = Sensor('imu')
Jido.append(IMU)

jido_posture = Sensor('jido_posture')
Jido.append(jido_posture)
jido_posture.properties(KUKAname = 'kuka_armature')
jido_posture.properties(PTUname = 'PTU')

Platine = Actuator('ptu')
Platine.translate(x=0.25, y=0, z=1.336)
Jido.append(Platine)
Platine.properties(Speed = 1.0000)
Platine.properties(Manual = False)

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

Pose_sensor = Sensor('pose')
Pose_sensor.name = 'Pose_sensor'
Jido.append(Pose_sensor)

Sick = Sensor('sick')
Sick.translate(x=0.37, z=0.3)
Jido.append(Sick)
Sick.properties(Visible_arc = False)
Sick.properties(laser_range = 30.0000)
Sick.properties(resolution = 1.0000)
Sick.properties(scan_window = 180.0000)

Pose_sensor.configure_mw('ros')
IMU.configure_mw('ros', ['ROS', 'post_velocity_twist', 'morse/middleware/ros/imu'])
Sick.configure_mw('ros')
Motion_Controller.configure_mw('ros')
kuka_base.configure_mw('ros', ['ROS', 'read_jointState', 'morse/middleware/ros/kuka_jointState'])
jido_posture.configure_mw('ros', ['ROS', 'post_jointState', 'morse/middleware/ros/jido_posture'])

env = Environment('indoors-1/indoor-1')
