#include "ors_genpos_poster.h"

// Return the data structure to Python.
// It will understand, since the definition of the structure is
//  in the files included in SWIG
GENPOS_CART_SPEED read_genPos_data( PosterHandler* handler, int* ok)
{
	GENPOS_CART_SPEED local_genPos;

	read_poster(handler, ok, &local_genPos, sizeof(local_genPos));

	return (local_genPos);
}
