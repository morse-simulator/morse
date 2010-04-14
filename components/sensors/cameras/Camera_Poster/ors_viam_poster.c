#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "ors_viam_poster.h"

//#include <posterLib.h>


void* init_data (char*	poster_name)
{
	void* id;

	STATUS s = posterCreate (poster_name, sizeof(POM_POS), &id);
	if (s == ERROR)
	{
		char buf[1024];
		h2getErrMsg(errnoGet(), buf, sizeof(buf));
		printf ("Unable to create the POM poster : %s\n",buf);
		return (NULL);
	}
	/*
	else
		printf ("INIT ID = %p (pointer)   %d(integer)\n", id);
	*/

	return (id);
}



POM_SENSOR_POS create_pom_sensor_pos( int blender_date,
						double robot_x,
						double robot_y,
						double robot_z,
						double robot_yaw,
						double robot_pitch,
						double robot_roll,
						double sensor_x,
						double sensor_y,
						double sensor_z,
						double sensor_yaw,
						double sensor_pitch,
						double sensor_roll)
{
	POM_SENSOR_POS local_sensor_pos;

	// Declare local versions of the structures used
	POM_EULER_V local_sensor_to_main;
	POM_EULER_V local_main_to_base;
	POM_EULER_V local_main_to_origin;
	POM_EULER_V local_v_local;


	// Fill in the Sensor to Main
	POM_EULER local_stm_euler;
	local_stm_euler.yaw = sensor_yaw;
	local_stm_euler.pitch = sensor_pitch;
	local_stm_euler.roll = sensor_roll;

	local_stm_euler.x = sensor_x;
	local_stm_euler.y = sensor_y;
	local_stm_euler.z = sensor_z;

	POM_EULER_VARIANCES sensor_var;

	// Fill in the POM_POS_EULER_V
	local_sensor_to_main.euler = local_stm_euler;
	local_sensor_to_main.var = sensor_var;


	// Fill in the Main to Origin
	POM_EULER local_mto_euler;
	local_mto_euler.yaw = robot_yaw;
	local_mto_euler.pitch = robot_pitch;
	local_mto_euler.roll = robot_roll;

	local_mto_euler.x = robot_x;
	local_mto_euler.y = robot_y;
	local_mto_euler.z = robot_z;

	POM_EULER_VARIANCES robot_var;

	// Fill in the POM_POS_EULER_V
	local_sensor_to_main.euler = local_mto_euler;
	local_sensor_to_main.var = robot_var;



	// Fill in the SENSOR_POS
	local_sensor_pos.date = blender_date;
	local_sensor_pos.pad = 0;
	local_sensor_pos.sensorToMain = local_sensor_to_main;
	local_sensor_pos.mainToBase = local_main_to_origin;
	local_sensor_pos.mainToOrigin = local_main_to_origin;
	local_sensor_pos.VLocal = local_v_local;

	return local_sensor_pos;
}


viam_cameracalibration_t create_calibration(void)
{
	viam_cameracalibration_t local_calibration;
	double calibration_matrix[9] = {14.63, 0.0, 256.0, 0.0, 14.63, 256.0, 0.0, 0.0, 1.0}; 

	int i;
	for (i=0; i<9; i++)
		local_calibration.intrinsic[i] = calibration_matrix[i];
	/*
	local_calibration.intrirect = {};
	local_calibration.distortion = {};
	local_calibration.rectification = {};
	local_calibration.rotation = {};
	*/
	local_calibration.width = 512;
	local_calibration.height = 512;

	return local_calibration;
}


int post_viam_poster(	void* id,
						char* camera_name,
						int blender_date,
						double robot_x,
						double robot_y,
						double robot_z,
						double robot_yaw,
						double robot_pitch,
						double robot_roll,
						double sensor_x,
						double sensor_y,
						double sensor_z,
						double sensor_yaw,
						double sensor_pitch,
						double sensor_roll,
						unsigned long tacq_sec,
						unsigned long tacq_usec,
						int cam_channels,
						int cam_depth,
						int cam_width,
						int cam_height,
						unsigned char* image_data)
{
	// Create the structure to fill
	ViamImageHeader localViamImageheader;
	int offset = 0;

	// Place the data sent from MORSE
	//strncpy ( (char*)localViamImageheader.name, camera_name, VIAM_ID_MAX);
	localViamImageheader.tacq_sec = tacq_sec;
	localViamImageheader.tacq_usec = tacq_usec;

	POM_SENSOR_POS local_pom_sensor_pos = create_pom_sensor_pos (blender_date, robot_x, robot_y, robot_z, robot_pitch, robot_yaw, robot_roll, sensor_x, sensor_y, sensor_z, sensor_yaw, sensor_pitch, sensor_roll);
	localViamImageheader.pos = local_pom_sensor_pos;

	localViamImageheader.nChannels = cam_channels;
	localViamImageheader.depth = cam_depth;
	localViamImageheader.width = cam_width;
	localViamImageheader.height = cam_height;
	int widthStep = cam_width * cam_channels * cam_depth;
	localViamImageheader.widthStep = widthStep;
	localViamImageheader.imageSize = cam_height * widthStep;

	viam_cameracalibration_t cam_calibration = create_calibration();
	localViamImageheader.calibration = cam_calibration;

	localViamImageheader.dataOffset = 0;
	localViamImageheader.data = image_data[0];

	// Write the structure just created as a poster
	posterWrite (id, offset, &localViamImageheader, sizeof(ViamImageHeader));

	return 0;
}


int finalize (void* id)
{
	posterDelete(id);

	return 0;
}
