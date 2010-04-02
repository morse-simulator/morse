#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "poster.h"

// #include <poster_read.hh>
// #include <poster_write.hh>

#include <pomStruct.h>
#include <posterLib.h>

unsigned char*	poster_name = "MORSE_POM_POSTER";
POSTER_ID id;

int init_data (void)
{
	STATUS s = posterCreate (poster_name, sizeof(POM_POS), &id);
	if (s == ERROR)
	{
		printf ("Unable to create the POM poster\n");
		return 1;
	}

	return 0;
}

int post_data( double x, double y, double z, double yaw, double pitch, double roll )
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

	posterWrite (id, offset, &local_pom_pos, sizeof(POM_POS));

	printf ("THIS .... ISN'T .... REALLY .... HAPPENING!!!!!!!!\n");
	printf ("FROM C POSTER MODULE:\n");
	printf ("\tyaw = %.4f\n", yaw);
	printf ("\tpitch = %.4f\n", pitch);
	printf ("\troll = %.4f\n", roll);

	return 0;
}


int finalize (void)
{
	posterDelete(id);

	return 0;
}

int main (void)
{
	init_data();
	post_data( 4, 3, 2, 0.0, 0.0, 0.0);
	finalize();
	return 0;
}
