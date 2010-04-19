#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "gyro_poster.hh"


PosterWriter<POM_POS> init_data (const char* poster_name)
{
	PosterWriter<POM_POS> writer(poster_name);

	return writer;
}

int post_data( PosterWriter<POM_POS> writer, double x, double y, double z, double yaw, double pitch, double roll, int date )
{
	// Variables to use for writing the poster
	int offset = 0;

	// Declare local versions of the structures used
	POM_POS local_pom_pos;
	POM_EULER local_pom_euler;
	POM_EULER_V local_pom_euler_v;

	// Fill in the POM_POS_EULER
	local_pom_euler.yaw = yaw;
	local_pom_euler.pitch = pitch;
	local_pom_euler.roll = roll;

	local_pom_euler.x = x;
	local_pom_euler.y = y;
	local_pom_euler.z = z;

	// Fill in the POM_POS_EULER_V
	local_pom_euler_v.euler = local_pom_euler;
	//local_pom_euler_v.var = ?????;

	// Fill in the POM_POS
	local_pom_pos.date = date;
	local_pom_pos.pomTickDate = local_pom_pos.date;
	local_pom_pos.mainToOrigin = local_pom_euler_v;
	local_pom_pos.mainToBase = local_pom_euler_v;
	local_pom_pos.VLocal = local_pom_euler_v;

	printf ("ABOUT TO DO THE ACTUAL 'posterWrite'\n");
	writer.put (&local_pom_pos);

	printf ("FROM C POSTER MODULE:");
	printf ("\tyaw = %.4f", yaw);
	printf ("\tpitch = %.4f", pitch);
	printf ("\troll = %.4f\n", roll);

	return 0;
}


int finalize (PosterWriter<POM_POS> writer)
{
	//delete (writer);

	return 0;
}
