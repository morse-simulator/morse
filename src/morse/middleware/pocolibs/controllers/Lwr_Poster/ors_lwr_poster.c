#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "ors_lwr_poster.h"


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
Gb_q7 read_lwr_data( POSTER_ID id )
{
	LWR_ARM_INST local_lwr;
    Gb_q7 local_q7;
	int offset = 0;
    int bytes;

	bytes = posterRead (id, offset, &local_lwr, sizeof(LWR_ARM_INST));
	printf ("SWIG: Reading %d bytes from poster ID = %p (pointer)   %d(integer)\n", bytes, id, id);

	// Read the variables we need for the segment angles
    local_q7 = local_lwr.currConf;

    /*
    printf ("Read some poster data: ");
    printf ("%.4f ", local_lwr.currConf.q1);
    printf ("%.4f ", local_lwr.currConf.q2);
    printf ("%.4f ", local_lwr.currConf.q3);
    printf ("%.4f ", local_lwr.currConf.q4);
    printf ("%.4f ", local_lwr.currConf.q5);
    printf ("%.4f ", local_lwr.currConf.q6);
    printf ("%.4f\n", local_lwr.currConf.q7);

    printf ("%.4f ", local_q7.q1);
    printf ("%.4f ", local_q7.q2);
    printf ("%.4f ", local_q7.q3);
    printf ("%.4f ", local_q7.q4);
    printf ("%.4f ", local_q7.q5);
    printf ("%.4f ", local_q7.q6);
    printf ("%.4f\n", local_q7.q7);
    */

	return (local_q7);
}


int finalize (POSTER_ID id)
{
	posterDelete(id);

	return 0;
}
