#include "ors_lwr_poster.h"

// Return the data structure to Python.
// It will understand, since the definition of the structure is
//  in the files included in SWIG
Gb_q7 read_lwr_data( PosterHandler* handler, int *ok)
{

	Gb_q7 local_q7;
	read_poster(handler, ok, &local_q7, sizeof(Gb_q7));

    /*
	printf ("SWIG: Reading %d bytes from poster ID = %p (pointer)   %d(integer)\n", bytes, id, id);
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


