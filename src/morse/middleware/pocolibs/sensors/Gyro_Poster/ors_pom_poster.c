#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

#include <pom/pomStruct.h>

#include "ors_pom_poster.h"

static char* ref_name;
static POSTER_ID ref_id;

POSTER_ID init_data (const char* poster_name, const char* reference_frame, 
					 float confidence, int* ok)
{
	POSTER_ID id;

	STATUS s = posterCreate (poster_name, sizeof(POM_ME_POS), &id);
	if (s == ERROR)
	{
		char buf[1024];
		h2getErrMsg(errnoGet(), buf, sizeof(buf));
		printf ("Unable to create the %s poster : %s\n", poster_name, buf);
        *ok = 0;
		return (NULL);
	}

	printf ("INIT ID = %p (pointer)\n", id);
	ref_name = strdup(reference_frame);

	POM_ME_POS* pos = posterAddr(id);
	memset(pos, 0, sizeof(POM_ME_POS));
	pos->kind = POM_ME_ABSOLUTE;
	pos->confidence = confidence;

    *ok = 1;

	return (id);
}


/*
 * Fill a POM_SENSOR_POS on the base of information returned by the simulator
 */
int
create_pom_sensor_pos( int blender_date, 
							POM_SENSOR_POS* pos,			 
							const struct pom_position* robot, 
							const struct pom_position* sensor)
{
	// Fill in the Sensor to Main
	POM_EULER* local_stm_euler = & (pos->sensorToMain.euler);
	local_stm_euler->yaw = sensor->yaw;
	local_stm_euler->pitch = sensor->pitch;
	local_stm_euler->roll = sensor->roll;

	local_stm_euler->x = sensor->x;
	local_stm_euler->y = sensor->y;
	local_stm_euler->z = sensor->z;

	// Fill in the Main to Origin
	POM_EULER* local_mto_euler = &(pos->mainToOrigin.euler);
	local_mto_euler->yaw = robot->yaw;
	local_mto_euler->pitch = robot->pitch;
	local_mto_euler->roll = robot->roll;

	local_mto_euler->x = robot->x;
	local_mto_euler->y = robot->y;
	local_mto_euler->z = robot->z;

	memcpy(&pos->mainToBase.euler, &pos->mainToOrigin.euler, sizeof(POM_EULER));

	pos->date = blender_date;
	pos->pad = 0;

	return 0;
}




/*
 * Yaw, pitch, roll are in degree in input
 */
int post_data( POSTER_ID id, const struct pom_position* robot, 
                             const struct pom_position* sensor) 
{
	// Variables to use for writing the poster
	int offset = 0;

	// try to get the pom reference frame
	// if we can't get it, just returns
	if (ref_id == NULL) {
		fprintf(stderr, "ref id is NULL : searching for %s\n", ref_name);
		if (posterFind(ref_name, &ref_id) == ERROR) {
			fprintf(stderr, "can't find %s : looping\n", ref_name);
			ref_id = NULL;
			return -1;
			}
	}

	// Declare local versions of the structures used
	POM_SENSOR_POS framePos;

	posterRead(ref_id, 0, &framePos, sizeof(POM_SENSOR_POS));

	//POM_ME_POS* pos = posterAddr(id);
	POM_SENSOR_POS* pos = posterAddr(id);
	posterTake(id, POSTER_WRITE);
	// Fill in the POM_POS_EULER
	// yaw, pitch, roll are expected in radian

	create_pom_sensor_pos(framePos.date, pos, robot, sensor);

	//pos->main.euler.yaw = yaw;
	//pos->main.euler.pitch = pitch;
	//pos->main.euler.roll = roll;

	//pos->main.euler.x = x;
	//pos->main.euler.y = y;
	//pos->main.euler.z = z;

    //pos->date1 = framePos.date;
	posterGive(id);
	return 0;
}


int finalize (POSTER_ID id)
{
	posterDelete(id);
	free(ref_name);

	return 0;
}
