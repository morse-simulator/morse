"""
MORSE_COMPONENTS: 
path to the Morse components
"""
MORSE_COMPONENTS = '/usr/local/share/data/morse/components'

"""
middleware-dictionary-convention:
{
  .blend-middleware: {
    .blend-component: ['MW', 'method', 'path']
  }
}
"""
MORSE_MIDDLEWARE_DICT = {
  'ros_empty': {
    'morse_vw_control': ['ROS', 'read_twist', 'morse/middleware/ros/read_vw_twist'],
    'morse_camera': ['ROS', 'post_image', 'morse/middleware/ros/camera'],
    'morse_sick': ['ROS', 'post_2DLaserScan', 'morse/middleware/ros/sick'],
    'morse_odometry': ['ROS', 'post_twist', 'morse/middleware/ros/odometry_sensor'],
    'morse_GPS': ['ROS', 'post_message'],
    'morse_gyroscope': ['ROS', 'post_message'],
    'morse_proximity': ['ROS', 'post_message']
  },
  'socket_empty': {
    'morse_gyroscope': ['Socket', 'post_message'],
    'morse_vw_control': ['Socket', 'read_message']
  }
}

"""
MORSE_COMPONENTS_DICT: 
components-dictionary-convention:
{
  'component-directory': {
    '.blend-file': ['main-object-name', 'child1-name', ...]
  }
}
"""
MORSE_COMPONENTS_DICT = {
 "modifiers": {
  "ned_empty": [
   "NED_Empty"
  ], 
  "gps_noise_empty": [
   "gps_noise_empty"
  ], 
  "utm_empty": [
   "UTM_Empty"
  ]
 }, 
 "middleware": {
  "socket_empty": [
   "Socket_Empty"
  ], 
  "ros_empty": [
   "ROS_Empty"
  ], 
  "yarp_empty": [
   "Yarp_Empty"
  ], 
  "pocolibs_empty": [
   "Pocolibs_Empty"
  ], 
  "text_empty": [
   "Text_Empty"
  ]
 }, 
 "robots": {
  "kuka_arm": [
   "kuka_base", 
   "kuka_7", 
   "kuka_6", 
   "kuka_5", 
   "kuka_4", 
   "kuka_3", 
   "kuka_2", 
   "kuka_1"
  ], 
  "jido-kuka": [
   "Jido", 
   "Wheels", 
   "TopCam", 
   "Target_Empty", 
   "Target_Ball", 
   "Sicks", 
   "MotorWheels", 
   "kuka_base", 
   "kuka_7", 
   "kuka_6", 
   "kuka_5", 
   "kuka_4", 
   "kuka_3", 
   "kuka_2", 
   "kuka_1", 
   "FrontScreen", 
   "Computer", 
   "Base"
  ], 
  "ressac-action": [
   "rotor", 
   "Ressac", 
   "Motion_Controller", 
   "Gyroscope", 
   "Gyro_box", 
   "GPS_box", 
   "GPS", 
   "CameraRobot", 
   "CameraMain", 
   "CameraLens", 
   "CameraCube", 
   "airframe"
  ], 
  "submarine": [
   "Sphere", 
   "Lamp", 
   "Circle.001", 
   "Circle", 
   "Camera"
  ], 
  "jido": [
   "Jido", 
   "Wheels", 
   "TopCam", 
   "Sicks", 
   "MotorWheels", 
   "FrontScreen", 
   "Computer", 
   "Base"
  ], 
  "daurade": [
   "Daurade"
  ], 
  "kuka_arm-rig": [
   "kuka_base", 
   "Target_Empty", 
   "Target_Ball", 
   "Lamp", 
   "kuka_armature", 
   "kuka_7", 
   "kuka_6", 
   "kuka_5", 
   "kuka_4", 
   "kuka_3", 
   "kuka_2", 
   "kuka_1", 
   "Empty.003", 
   "Empty.002", 
   "Empty.001", 
   "Empty", 
   "Camera"
  ], 
  "atrv": [
   "ATRV", 
   "Wheel.4", 
   "Wheel.3", 
   "Wheel.2", 
   "Wheel.1"
  ], 
  "ressac": [
   "Ressac", 
   "rotor", 
   "airframe"
  ], 
  "jido-armature": [
   "Jido", 
   "Wheels", 
   "TopCam", 
   "Target_Empty", 
   "Target_Ball", 
   "Sicks", 
   "MotorWheels", 
   "kuka_base", 
   "kuka_armature", 
   "kuka_7", 
   "kuka_6", 
   "kuka_5", 
   "kuka_4", 
   "kuka_3", 
   "kuka_2", 
   "kuka_1", 
   "GSocle", 
   "GPA10-6", 
   "GPA10-5", 
   "GPA10-4", 
   "GPA10-3", 
   "gpa10-2", 
   "GPA10-1", 
   "FrontScreen", 
   "Computer", 
   "Base"
  ], 
  "PA10": [
   "Socle", 
   "pince2Sright", 
   "pince2sleft", 
   "pince2", 
   "pince1S", 
   "pince1", 
   "PA10-6", 
   "PA10-5", 
   "PA10-4", 
   "PA10-3", 
   "PA10-2", 
   "PA10-1", 
   "GSocle", 
   "Gripper", 
   "GPA10-6", 
   "GPA10-5", 
   "GPA10-4", 
   "GPA10-3", 
   "gpa10-2", 
   "GPA10-1", 
   "Empty.003", 
   "Empty.002", 
   "Empty.001", 
   "Empty"
  ], 
  "dala-action": [
   "ATRV"
   "Wheel.4", 
   "Wheel.3", 
   "Wheel.2", 
   "Wheel.1", 
   "PTU", 
   "Motion_Controller", 
   "Gyroscope", 
   "Gyro_box", 
   "GPS_box", 
   "GPS", 
   "CameraRobot.001", 
   "CameraRobot", 
   "CameraMain.001", 
   "CameraMain", 
   "CameraLens.001", 
   "CameraLens", 
   "CameraCube.001", 
   "CameraCube", 
   "Camera_Support", 
  ], 
  "sa_hand-8-semi_ok": [
   "palmRight", 
   "thumbBaseRight", 
   "padProx.003", 
   "padProx.002", 
   "padProx.001", 
   "padProx", 
   "padPalmRight", 
   "padDist.003", 
   "padDist.002", 
   "padDist.001", 
   "padDist", 
   "fingerPhaProx.003", 
   "fingerPhaProx.002", 
   "fingerPhaProx.001", 
   "fingerPhaProx", 
   "fingerPhaMid.003", 
   "fingerPhaMid.002", 
   "fingerPhaMid.001", 
   "fingerPhaMid", 
   "fingerPhaDist.003", 
   "fingerPhaDist.002", 
   "fingerPhaDist.001", 
   "fingerPhaDist", 
   "fingerBase.003", 
   "fingerBase.002", 
   "fingerBase.001", 
   "fingerBase"
  ], 
  "kuka_arm-1": [
   "kuka_base", 
   "kuka_7", 
   "kuka_6", 
   "kuka_5", 
   "kuka_4", 
   "kuka_3", 
   "kuka_2", 
   "kuka_1"
  ], 
  "PA10-Armature": [
   "Socle", 
   "Target_Empty", 
   "pince2Sright", 
   "pince2sleft", 
   "pince2", 
   "pince1S", 
   "pince1", 
   "PA10-6", 
   "PA10-5", 
   "PA10-4", 
   "PA10-3", 
   "PA10-2", 
   "PA10-1", 
   "GSocle", 
   "Gripper", 
   "GPA10-6", 
   "GPA10-5", 
   "GPA10-4", 
   "GPA10-3", 
   "gpa10-2", 
   "GPA10-1", 
   "Empty.003", 
   "Empty.002", 
   "Empty.001", 
   "Empty", 
   "Armature"
  ], 
  "PA10-Game-2.5": [
   "Socle", 
   "Target_Empty", 
   "pince2Sright", 
   "pince2sleft", 
   "pince2", 
   "pince1S", 
   "pince1", 
   "PA10-6", 
   "PA10-5", 
   "PA10-4", 
   "PA10-3", 
   "PA10-2", 
   "PA10-1", 
   "Mesh", 
   "Lamp", 
   "GSocle", 
   "Gripper", 
   "GPA10-6", 
   "GPA10-5", 
   "GPA10-4", 
   "GPA10-3", 
   "gpa10-2", 
   "GPA10-1", 
   "Empty.003", 
   "Empty.002", 
   "Empty.001", 
   "Empty", 
   "Armature"
  ], 
  "dala-rosace": [
   "ATRV", 
   "Wheel.4", 
   "Wheel.3", 
   "Wheel.2", 
   "Wheel.1", 
   "Thermometer", 
   "Thermo_box", 
   "Proximity_Sensor", 
   "Proximity_Antena", 
   "Motion_Controller", 
   "Gyroscope", 
   "Gyro_box", 
   "GPS_box", 
   "GPS"
  ]
 }, 
 "scenes": {}, 
 "controllers": {
  "morse_xyw_control": [
   "Motion_Controller"
  ], 
  "morse_destination_control": [
   "Motion_Controller"
  ], 
  "morse_platine_control": [
   "Platine", 
   "TiltBase", 
   "PanBase"
  ], 
  "morse_healer_beam": [
   "Healer_Beam", 
   "Healer_Mesh"
  ], 
  "morse_vw_control": [
   "Motion_Controller"
  ], 
  "morse_waypoint_control": [
   "Motion_Controller", 
   "Radar.R", 
   "Radar.L"
  ], 
  "morse_manual_control": [
   "Motion_Controller"
  ], 
  "morse_orientation_control": [
   "Motion_Controller"
  ]
 }, 
 "human": {
  "default_human": [
   "UpLegBB.R", 
   "UpLegBB.L", 
   "UpLeg.R", 
   "UpLeg.L", 
   "UpArmBB.R", 
   "UpArmBB.L", 
   "UpArm.R", 
   "UpArm.L", 
   "Target_Empty", 
   "pelvisBB", 
   "Pelvis", 
   "LoLegBB.L.001", 
   "LoLegBB.L", 
   "LoLeg.R", 
   "LoLeg.L", 
   "LoArmBB.R", 
   "LoArmBB.L", 
   "LoArm.R", 
   "LoArm.L", 
   "IK_Target_Empty.R", 
   "IK_Target_Empty.L", 
   "HumanArmature", 
   "Human_Camera", 
   "Human", 
   "HeadBB", 
   "Head", 
   "HandBB.R", 
   "HandBB.L", 
   "Hand_Grab.R", 
   "Hand_Grab.L", 
   "Hand.R", 
   "Hand.L", 
   "FootBB.R", 
   "FootBB.L", 
   "Foot.R", 
   "Foot.L", 
   "ChestBB", 
   "Chest"
  ], 
  "achile-2.5": [
   "UpLegBB.R", 
   "UpLegBB.L", 
   "UpLeg.R", 
   "UpLeg.L", 
   "UpArmBB.R", 
   "UpArmBB.L", 
   "UpArm.R", 
   "UpArm.L", 
   "Target_Empty", 
   "Target_Ball.R", 
   "Target_Ball.L", 
   "Target_Ball", 
   "Plane", 
   "pelvisBB", 
   "Pelvis", 
   "LoLegBB.L.001", 
   "LoLegBB.L", 
   "LoLeg.R", 
   "LoLeg.L", 
   "LoArmBB.R", 
   "LoArmBB.L", 
   "LoArm.R", 
   "LoArm.L", 
   "Lamp.003", 
   "Lamp.002", 
   "Lamp.001", 
   "IK_Target_Empty.R", 
   "IK_Target_Empty.L", 
   "Human_Camera", 
   "Human", 
   "HeadBB", 
   "Head", 
   "HandBB.R", 
   "HandBB.L", 
   "Hand.R", 
   "Hand.L", 
   "FootBB.R", 
   "FootBB.L", 
   "Foot.R", 
   "Foot.L", 
   "ChestBB", 
   "Chest", 
   "Achille_Armature"
  ], 
  "achile-robot": [
   "UpLegBB.R", 
   "UpLegBB.L", 
   "UpLeg.R", 
   "UpLeg.L", 
   "UpArmBB.R", 
   "UpArmBB.L", 
   "UpArm.R", 
   "UpArm.L", 
   "Target_Empty", 
   "Target_Ball.R", 
   "Target_Ball.L", 
   "Target_Ball", 
   "pelvisBB", 
   "Pelvis", 
   "LoLegBB.L.001", 
   "LoLegBB.L", 
   "LoLeg.R", 
   "LoLeg.L", 
   "LoArmBB.R", 
   "LoArmBB.L", 
   "LoArm.R", 
   "LoArm.L", 
   "IK_Target_Empty.R", 
   "IK_Target_Empty.L", 
   "Human_Camera", 
   "Human", 
   "HeadBB", 
   "Head", 
   "HandBB.R", 
   "HandBB.L", 
   "Hand_Grab.R", 
   "Hand_Grab.L", 
   "Hand.R", 
   "Hand.L", 
   "FootBB.R", 
   "FootBB.L", 
   "Foot.R", 
   "Foot.L", 
   "ChestBB", 
   "Chest", 
   "Achille_Armature"
  ], 
  "achile3": [
   "UpLegBB.R", 
   "UpLegBB.L", 
   "UpLeg.R", 
   "UpLeg.L", 
   "UpArmBB.R", 
   "UpArmBB.L", 
   "UpArm.R", 
   "UpArm.L", 
   "Target_R", 
   "Target_L", 
   "Plane", 
   "pelvisBB", 
   "pelvis", 
   "LoLegBB.L.001", 
   "LoLegBB.L", 
   "LoLeg.R", 
   "LoLeg.L", 
   "LoArmBB.R", 
   "LoArmBB.L", 
   "LoArm.R", 
   "LoArm.L", 
   "Lamp.002", 
   "Lamp.001", 
   "Lamp", 
   "Human", 
   "HeadBB", 
   "Head", 
   "HandBB.R", 
   "HandBB.L", 
   "Hand.R", 
   "Hand.L", 
   "FootBB.R", 
   "FootBB.L", 
   "Foot.R", 
   "Foot.L", 
   "ChestBB", 
   "Chest", 
   "CameraHum", 
   "Camera.001", 
   "Armature"
  ]
 }, 
 "sensors": {
  "morse_stereo_cameras": [
   "PTU", 
   "CameraRobot.001", 
   "CameraRobot", 
   "CameraMain.001", 
   "CameraMain", 
   "CameraLens.001", 
   "CameraLens", 
   "CameraCube.001", 
   "CameraCube", 
   "Camera_Support"
  ], 
  "morse_sick": [
   "Sick", 
   "Sick_Model", 
   #"Sick_Model_Heavy", 
   #"Arc_270", 
   "Arc_180"
  ], 
  "morse_proximity": [
   "Proximity", 
   "Proximity_Antena"
  ], 
  "morse_odometry": [
   "Odometry",
   "Odometry_mesh"
  ], 
  "morse_GPS": [
   "GPS",
   "GPS_box" 
  ], 
  "morse_camera": [
   "CameraMain", 
   "CameraRobot", 
   "CameraLens", 
   "CameraCube"
  ], 
  "morse_ptu": [
   "PTU", 
   "Camera_Support"
  ], 
  "morse_accelerometer": [
   "Accelerometer", 
   "AccelBox"
  ], 
  "morse_kuka_posture": [
   "kuka_posture", 
   "GPS_box"
  ], 
  "morse_gyroscope": [
   "Gyroscope", 
   "Gyro_box"
  ], 
  "morse_pose": [
   "Pose_sensor", 
   "Pose_mesh"
  ], 
  "morse_velodyne": [
   "Velodyne", 
   "VelodyneMesh", 
   "Arc_31"
  ], 
  "morse_thermometer": [
   "Thermometer", 
   "Thermo_box"
  ]
 }
}

