#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "ors_pom_poster.h"

#include <pom/pomStruct.h>
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

int post_data( void* id, double x, double y, double z, double yaw, double pitch, double roll )
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
	//local_pom_pos.date = ????;
	local_pom_pos.pomTickDate = local_pom_pos.date;
	local_pom_pos.mainToOrigin = local_pom_euler_v;
	local_pom_pos.mainToBase = local_pom_euler_v;
	local_pom_pos.VLocal = local_pom_euler_v;

	// printf ("ABOUT TO DO THE ACTUAL 'posterWrite'\n");
	// printf ("ID = %p\n", id);
	posterWrite (id, offset, &local_pom_pos, sizeof(POM_POS));

	/*
	printf ("FROM C POSTER MODULE:");
	printf ("\tyaw = %.4f", yaw);
	printf ("\tpitch = %.4f", pitch);
	printf ("\troll = %.4f\n", roll);
	*/

	return 0;
}


int finalize (void* id)
{
	posterDelete(id);

	return 0;
}
