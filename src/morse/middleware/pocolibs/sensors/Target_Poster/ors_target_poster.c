#include <assert.h>
#include <stdio.h>
#include <genPos/genPosStruct.h>

#include "ors_target_poster.h"



/*
 * Return a POSTER_ID on success, NULL otherwise
 *
 * you must call finalize when you don't use anymore the POSTER_ID
 */
POSTER_ID init_data (const char*	poster_name, int* ok)
{
	size_t poster_size = 0;
	POSTER_ID id;

	poster_size = sizeof(GENPOS_TRAJ_POINTS);

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

	GENPOS_TRAJ_POINTS* points = posterAddr(id);
	points->numRef = 0;
	points->nbPts = 0;

	return id;
}

int post_target_poster(POSTER_ID id, int has_target, double x, double y)
{
	GENPOS_TRAJ_POINTS* points = posterAddr(id);
	points->numRef++;
	points->nbPts = has_target;
	if (has_target) {
		points->points[0].x = x;
		points->points[0].y = y;
	}

	return 0;
}

int finalize (POSTER_ID id)
{
	posterDelete(id);
	return 0;
}
