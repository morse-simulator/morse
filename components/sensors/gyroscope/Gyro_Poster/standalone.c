#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include <pom/pomStruct.h>
#include <posterLib.h>

POSTER_ID init_data (char* poster_name)
{
	POSTER_ID id;

	STATUS s = posterCreate (poster_name, sizeof(POM_POS), &id);
	if (s == ERROR)
	{
		printf ("Unable to create the POM poster\n");
		return (NULL);
	}

	return (id);
}

int post_data( POSTER_ID id, double x, double y, double z, double yaw, double pitch, double roll )
{
	printf ("FROM C POSTER MODULE:\n");

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
	printf ("\tyaw = %.4f\n", yaw);
	printf ("\tpitch = %.4f\n", pitch);
	printf ("\troll = %.4f\n", roll);

	return 0;
}


int finalize (POSTER_ID id)
{
	posterDelete(id);

	return 0;
}

int main (void)
{
	unsigned char*	poster_name = "MORSE_POM_POSTER";
	POSTER_ID id;

	id = init_data(poster_name);
	printf ("THIS IS STANDALONE SPEAKING!");
	post_data( id, 4, 3, 2, 0.5, 8.0, 1.0);
	finalize(id);
	return 0;
}
