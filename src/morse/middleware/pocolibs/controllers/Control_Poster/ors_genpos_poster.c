#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "ors_genpos_poster.h"


POSTER_ID locate_poster (char*	poster_name)
{
	POSTER_ID id;

	STATUS s = posterFind (poster_name, &id);
	if (s == ERROR)
	{
		char buf[1024];
		h2getErrMsg(errnoGet(), buf, sizeof(buf));
		printf ("Unable to locate the %s poster : %s\n", poster_name, buf);
		return (NULL);
	}
	else
		printf ("INIT ID = %p (pointer)   %d(integer)\n", id, id);

	return (id);
}



// Return the data structure to Python.
// It will understand, since the definition of the structure is
//  in the files included in SWIG
GENPOS_CART_SPEED read_genPos_data( POSTER_ID id )
{
	GENPOS_CART_SPEED local_genPos;
	int offset = 0;
	// float v;
	// float w;

	posterRead (id, offset, &local_genPos, sizeof(GENPOS_CART_SPEED));

	// Read the variables we need for the speed
	// v = local_genPos.v;
	// w = local_genPos.w;

	// printf ("Reading from poster ID = %p (pointer)   %d(integer)\n", id, id);
	// printf ("DATA READ FROM POSTER:");
	// printf ("\tv = %.4f", v);
	// printf ("\tw = %.4f\n", w);

	return (local_genPos);
}


int finalize (POSTER_ID id)
{
	posterDelete(id);

	return 0;
}
