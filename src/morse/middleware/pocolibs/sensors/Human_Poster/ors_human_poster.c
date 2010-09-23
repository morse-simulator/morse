#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdbool.h>
#include <pthread.h>

#include <genBasic/genBasicStruct.h>
#include <genManip/genManipStruct.h>

#include "ors_human_poster.h"

POSTER_ID global_id;
MHP_HUMAN_CONFIGURATION global_human_data;

/*
 * Create a poster, and fill it with information that don't change during the
 * execution
 *
 * Return a POSTER_ID on success, NULL otherwise
 *
 * you must call finalize when you don't use anymore the POSTER_ID
 */
POSTER_ID init_data (char*	poster_name, int* ok)
{
	size_t poster_size = 0;
	POSTER_ID id;

	poster_size = sizeof(MHP_HUMAN_CONFIGURATION);

	STATUS s = posterCreate (poster_name, poster_size, &id);
	if (s == ERROR)
	{
		char buf[1024];
		h2getErrMsg(errnoGet(), buf, sizeof(buf));
		printf ("Unable to create the %s poster : %s\n",poster_name, buf);
		*ok = 0;
		return (NULL);
	}

	//printf("Poster '%s' created of type VIMAN (size %zd)\n", poster_name, poster_size); 
	//printf ("INIT ID = %p (pointer)\n", id);

	*ok = 1;

	return (id);
}

void set_dof(MHP_HUMAN_CONFIGURATION* human_data, double dof[40])
{
	memcpy(human_data->dof, dof, sizeof(dof));
}

/*
 * Create a data structure to hold the posture of the human
 * Returns the newly created structure
 */
MHP_HUMAN_CONFIGURATION generate_human_struct()
{
	return global_human_data;
}

int post_human_poster(POSTER_ID id, MHP_HUMAN_CONFIGURATION* human_data)
{
	size_t offset = 0;
	posterWrite(id, offset, human_data, sizeof(MHP_HUMAN_CONFIGURATION));
	return 0;
}

int finalize (POSTER_ID id)
{
	posterDelete(id);
	return 0;
}
