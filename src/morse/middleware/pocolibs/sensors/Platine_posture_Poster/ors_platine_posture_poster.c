#include <assert.h>
#include <stdio.h>
#include <string.h>
#include <math.h>
#include <pom/pomStruct.h>
#include <platine/platineStruct.h>

#include "ors_platine_posture_poster.h"

/*
 * Return a POSTER_ID on success, NULL otherwise
 *
 * you must call finalize when you don't use anymore the POSTER_ID
 */
POSTER_ID init_data (const char*	poster_name, int* ok)
{
	size_t poster_size = 0;
	POSTER_ID id;

	poster_size = sizeof(PLATINE_STATES);

	STATUS s = posterCreate (poster_name, poster_size, &id);
	if (s == ERROR)
	{
		char buf[1024];
		h2getErrMsg(errnoGet(), buf, sizeof(buf));
		printf ("Unable to create the %s poster : %s\n",poster_name, buf);
        *ok = 0;
		return NULL;
	}

	printf("Succesfully created poster %s of size %zd\n", poster_name, poster_size); 
	printf("INIT ID = %p (pointer)\n", id);

    *ok = 1;

	PLATINE_STATES* points = posterAddr(id);
	memset(points, 0, sizeof(PLATINE_STATES));

	return id;
}

#define RAD_TO_DEG(x) (x * 180.0 / M_PI)

int post_platine_posture(POSTER_ID id,  double pan, double tilt)
{
	PLATINE_STATES* states = posterAddr(id);
	states->stateRad.pos.pan = pan;
	states->stateRad.pos.tilt = tilt;

	states->stateDeg.pos.pan = RAD_TO_DEG(pan);
	states->stateDeg.pos.tilt = RAD_TO_DEG(tilt);

	return 0;
}

int finalize (POSTER_ID id)
{
	posterDelete(id);
	return 0;
}
