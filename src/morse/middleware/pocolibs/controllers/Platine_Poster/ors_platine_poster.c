#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "ors_platine_poster.h"


POSTER_ID locate_poster (char*	poster_name, int* ok)
{
	POSTER_ID id;

	STATUS s = posterFind (poster_name, &id);
	if (s == ERROR)
	{
		char buf[1024];
		h2getErrMsg(errnoGet(), buf, sizeof(buf));
		printf ("Unable to locate the %s poster : %s\n", poster_name, buf);
		*ok = 0;
		return (NULL);
	}
	else
		printf ("INIT ID = %p (pointer)\n", id);
		*ok = 1;

	return (id);
}



// Return the data structure to Python.
// It will understand, since the definition of the structure is
//  in the files included in SWIG
POM_EULER read_platine_data( POSTER_ID id )
{
	POM_SE_POSTER local_se;
    POM_EULER_V local_euler_v;
    POM_EULER local_euler;
	int offset = 0;
	// float pan;
	// float tilt;

	posterRead (id, offset, &local_se, sizeof(POM_SE_POSTER));
    local_euler_v = local_se.seConfig;
    local_euler = local_euler_v.euler;

	// Read the variables we need for the speed
	// pan = local_euler.yaw;
	// tilt = local_euler.pitch;

	// printf ("Reading from poster ID = %p (pointer)   %d(integer)\n", id, id);
	// printf ("DATA READ FROM POSTER:");
	// printf ("\tpan = %.4f", pan);
	// printf ("\ttilt = %.4f\n", tilt);

	return (local_euler);
}


int finalize (POSTER_ID id)
{
	posterDelete(id);

	return 0;
}
