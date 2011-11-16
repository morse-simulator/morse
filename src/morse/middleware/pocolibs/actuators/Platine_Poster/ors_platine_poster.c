#include "ors_platine_poster.h"


// Return the data structure to Python.
// It will understand, since the definition of the structure is
//  in the files included in SWIG
POM_EULER read_platine_data( PosterHandler* handler, int* ok)
{
	POM_SE_POSTER local_se;
    POM_EULER_V local_euler_v;
    POM_EULER local_euler;
	// float pan;
	// float tilt;

	read_poster(handler, ok, &local_se, sizeof(POM_SE_POSTER));
	if (*ok) {
		local_euler_v = local_se.seConfig;
		local_euler = local_euler_v.euler;
	}

	return (local_euler);
}

PLATINE_AXIS_STR read_platine_axis( PosterHandler* handler, int* ok)
{
	PLATINE_AXIS_STR axis;

	read_poster(handler, ok, &axis, sizeof(axis));
	return axis;
}

