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

POM_SENSOR_POS create_pom_sensor_pos( int blender_date, double x, double y, double z, double yaw, double pitch, double roll )
{
	POM_SENSOR_POS local_sensor_pos;

	// Declare local versions of the structures used
	POM_EULER_V local_sensor_to_main;
	POM_EULER_V local_main_to_base;
	POM_EULER_V local_main_to_origin;
	POM_EULER_V local_v_local;

	// Fill in the Sensor to Main
	POM_EULER local_euler;
	local_euler.yaw = yaw;
	local_euler.pitch = pitch;
	local_euler.roll = roll;

	local_euler.x = x;
	local_euler.y = y;
	local_euler.z = z;

	POM_EULER_VARIANCES var;

	// Fill in the POM_POS_EULER_V
	local_sensor_to_main.euler = local_euler;
	local_sensor_to_main.var = var;

	// Fill in the Sensor to Main
	POM_EULER local_stm_euler;
	local_stm_euler.yaw = yaw;
	local_stm_euler.pitch = pitch;
	local_stm_euler.roll = roll;

	local_stm_euler.x = x;
	local_stm_euler.y = y;
	local_stm_euler.z = z;

	POM_EULER_VARIANCES var;

	// Fill in the POM_POS_EULER_V
	local_sensor_to_main.euler = local_euler;
	local_sensor_to_main.var = var;


	// Fill in the Main to Origin
	POM_EULER local_mto_euler;
	local_mto_euler.yaw = yaw;
	local_mto_euler.pitch = pitch;
	local_mto_euler.roll = roll;

	local_mto_euler.x = x;
	local_mto_euler.y = y;
	local_mto_euler.z = z;

	POM_EULER_VARIANCES var;

	// Fill in the POM_POS_EULER_V
	local_sensor_to_main.euler = local_mto_euler;
	local_sensor_to_main.var = var;



	// Fill in the SENSOR_POS
	local_sensor_pos.date = blender_date;
	local_sensor_pos.pad = 0;
	local_sensor_pos.sensorToMain = local_sensor_to_main;
	local_sensor_pos.mainToBase = local_main_to_origin;
	local_sensor_pos.mainToOrigin = local_main_to_origin;
	local_sensor_pos.VLocal = local_v_local;

	return 0;
}


int finalize (void* id)
{
	posterDelete(id);

	return 0;
}
