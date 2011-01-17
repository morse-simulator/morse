#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include <sparkStruct.h>

#include "ors_human_posture_poster.h"

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

	poster_size = sizeof(SPARK_CONFIGURATION);

	STATUS s = posterCreate (poster_name, poster_size, &id);
	if (s == ERROR)
	{
		char buf[1024];
		h2getErrMsg(errnoGet(), buf, sizeof(buf));
		printf ("Unable to create the %s poster : %s\n",poster_name, buf);
		*ok = 0;
		return (NULL);
	}

	*ok = 1;

	return (id);
}

int post_human_poster(POSTER_ID id, double dof[45])
{
	SPARK_CONFIGURATION human_data;
	
	size_t offset = 0;
	
	human_data.dofNb = 45;
	memcpy(human_data.dof, dof, human_data.dofNb * sizeof(double));
	human_data.changed = 1;
	
	
	posterWrite(id, offset, &human_data, sizeof(SPARK_CONFIGURATION));
	return 0;
}

int finalize (POSTER_ID id)
{
	posterDelete(id);
	return 0;
}
