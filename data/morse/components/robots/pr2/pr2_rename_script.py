from collections import defaultdict

import bpy

# function to automaticly shorten names
def name_shortener(og_name):
	new_name = og_name
	# check name length
	if (len(new_name) > 20):
		new_name = new_name.replace('camera', 'cam')
	if (len(new_name) > 20):
		new_name = new_name.replace('optical', 'opt')		
	if (len(new_name) > 20):
		new_name = new_name.replace('gripper', 'grip')
	if (len(new_name) > 20):
		new_name = new_name.replace('finger', 'fing')
	if (len(new_name) > 20):
		new_name = new_name.replace('gazebo', 'gaz')
	if (len(new_name) > 20):
		new_name = new_name.replace('accelerometer', 'acc')
	if (len(new_name) > 20):
		new_name = new_name.replace('motor', 'mtr')
	if (len(new_name) > 20):
		new_name = new_name.replace('_frame', '')
	if (len(new_name) > 20):
		new_name = new_name.replace('_link', '')
	if (len(new_name) > 20):
		new_name = new_name.replace('_joint', '')		
	if (len(new_name) > 20):
		new_name = new_name.replace('_stereo_camera', '')	
	if (len(new_name) > 20):
		new_name = new_name.replace('_stereo_cam', '')
	if (len(new_name) > 20):
		new_name = new_name.replace('rotation', 'rot')
	return new_name
	

			
# Define the dictionary with all the name mappings
# Dictonary is copied from a script created by ruben who mapped the names of the resulting .dae file to the original urdf names.
# The regular nodes are actually the joints (and not links)
# The visual nodes (not in this dictionary) represent the meshes of the links.
frames = {}
frames["v1.node0"] = "base_footprint"
frames["v1.node1"] = "base_link"
frames["v1.node2"] = "base_laser_link"
frames["v1.node3"] = "bl_caster_rotation_link"
frames["v1.node4"] = "bl_caster_l_wheel_link"
frames["v1.node5"] = "bl_caster_r_wheel_link"
frames["v1.node6"] = "br_caster_rotation_link"
frames["v1.node7"] = "br_caster_l_wheel_link"
frames["v1.node8"] = "br_caster_r_wheel_link"
frames["v1.node9"] = "fl_caster_rotation_link"
frames["v1.node10"] = "fl_caster_l_wheel_link"
frames["v1.node11"] = "fl_caster_r_wheel_link"
frames["v1.node12"] = "fr_caster_rotation_link"
frames["v1.node13"] = "fr_caster_l_wheel_link"
frames["v1.node14"] = "fr_caster_r_wheel_link"
frames["v1.node15"] = "torso_lift_link"
frames["v1.node16"] = "head_pan_link"
frames["v1.node17"] = "head_tilt_link"
frames["v1.node18"] = "head_plate_frame"
frames["v1.node19"] = "sensor_mount_link"
frames["v1.node20"] = "double_stereo_link"
frames["v1.node21"] = "narrow_stereo_link"
frames["v1.node22"] = "narrow_stereo_gazebo_l_stereo_camera_frame"
frames["v1.node23"] = "narrow_stereo_gazebo_l_stereo_camera_optical_frame"
frames["v1.node24"] = "narrow_stereo_gazebo_r_stereo_camera_frame"
frames["v1.node25"] = "narrow_stereo_gazebo_r_stereo_camera_optical_frame"
frames["v1.node26"] = "narrow_stereo_optical_frame"
frames["v1.node27"] = "wide_stereo_link"
frames["v1.node28"] = "wide_stereo_gazebo_l_stereo_camera_frame"
frames["v1.node29"] = "wide_stereo_gazebo_l_stereo_camera_optical_frame"
frames["v1.node30"] = "wide_stereo_gazebo_r_stereo_camera_frame"
frames["v1.node31"] = "wide_stereo_gazebo_r_stereo_camera_optical_frame"
frames["v1.node32"] = "wide_stereo_optical_frame"
frames["v1.node33"] = "high_def_frame"
frames["v1.node34"] = "high_def_optical_frame"
frames["v1.node35"] = "imu_link"
frames["v1.node36"] = "l_shoulder_pan_link"
frames["v1.node37"] = "l_shoulder_lift_link"
frames["v1.node38"] = "l_upper_arm_roll_link"
frames["v1.node39"] = "l_upper_arm_link"
frames["v1.node40"] = "l_elbow_flex_link"
frames["v1.node41"] = "l_forearm_roll_link"
frames["v1.node42"] = "l_forearm_cam_frame"
frames["v1.node43"] = "l_forearm_cam_optical_frame"
frames["v1.node44"] = "l_forearm_link"
frames["v1.node45"] = "l_wrist_flex_link"
frames["v1.node46"] = "l_wrist_roll_link"
frames["v1.node47"] = "l_gripper_palm_link"
frames["v1.node48"] = "l_gripper_l_finger_link"
frames["v1.node49"] = "l_gripper_l_finger_tip_link"
frames["v1.node50"] = "l_gripper_led_frame"
frames["v1.node51"] = "l_gripper_motor_accelerometer_link"
frames["v1.node52"] = "l_gripper_r_finger_link"
frames["v1.node53"] = "l_gripper_r_finger_tip_link"
frames["v1.node54"] = "l_gripper_tool_frame"
frames["v1.node55"] = "laser_tilt_mount_link"
frames["v1.node56"] = "laser_tilt_link"
frames["v1.node57"] = "r_shoulder_pan_link"
frames["v1.node58"] = "r_shoulder_lift_link"
frames["v1.node59"] = "r_upper_arm_roll_link"
frames["v1.node60"] = "r_upper_arm_link"
frames["v1.node61"] = "r_elbow_flex_link"
frames["v1.node62"] = "r_forearm_roll_link"
frames["v1.node63"] = "r_forearm_cam_frame"
frames["v1.node64"] = "r_forearm_cam_optical_frame"
frames["v1.node65"] = "r_forearm_link"
frames["v1.node66"] = "r_wrist_flex_link"
frames["v1.node67"] = "r_wrist_roll_link"
frames["v1.node68"] = "r_gripper_palm_link"
frames["v1.node69"] = "r_gripper_l_finger_link"
frames["v1.node70"] = "r_gripper_l_finger_tip_link"
frames["v1.node71"] = "r_gripper_led_frame"
frames["v1.node72"] = "r_gripper_motor_accelerometer_link"
frames["v1.node73"] = "r_gripper_r_finger_link"
frames["v1.node74"] = "r_gripper_r_finger_tip_link"
frames["v1.node75"] = "r_gripper_tool_frame"


# Script starts here:

# Add Stringproperty to objects:
bpy.types.Object.dae_name = bpy.props.StringProperty(name="dae_name")
bpy.types.Object.urdf_name = bpy.props.StringProperty(name="urdf_name")

# Iterate over all objects
for obj in bpy.data.objects:
	obj.dae_name = str(obj.name)
	new_name = ""
	
	if obj.name in frames: # obj.name is in frames
		# Get new name
		new_name = frames[obj.name]
		# check if new name has "link"
		if '_link' in new_name:
			new_name = new_name.replace('_link', '_joint')
		
		obj.urdf_name = str(new_name)
		
		# Check new name length
		if len(new_name) > 20:
			new_name = name_shortener(new_name)
		# while len(new_name) > 20:
		# 	print("" + new_name + " has length: " + str(len(new_name)) + ". This is too long (max 20)")
		# 	new_name = input("Give a new name: ")
			
	else: # obj.name is not in frames
		# obj is .visual obj
		if '.visual' in obj.name:
			name = obj.name.replace('.visual', '')
			if name in frames: # obj.name is in frames
				# Get new name
				new_name = frames[name]
				obj.urdf_name = str(new_name)
				# Check new name length
				if len(new_name) > 20:
					new_name = name_shortener(new_name)
		else:	# obj is not .visual
			new_name = obj.name
			

	if obj.name == new_name:
		print(str(obj.name) + " doesn't change")
	else:					
		print("Change " + str(obj.name) + " into " + new_name)
	# Change name
	obj.name = new_name
	
print('Rename Complete!')
