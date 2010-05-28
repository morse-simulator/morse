#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdbool.h>
#include "ors_sick_poster.h"

static int fill_sick_data(POM_SENSOR_POS* pom_sensor_pos, char* sick_data);


/*
 * Create a poster, and fill it with information which don't change during the
 * execution, including bank_name, camera_name, nb_images, size of images,
 * camera calibration...
 *
 * Return a POSTER_ID on success, NULL otherwise
 *
 * you must call finalize when you don't use anymore the POSTER_ID
 */
POSTER_ID init_data (char* poster_name)
{
	POSTER_ID id;

	STATUS s = posterCreate (poster_name, sizeof(SICK_CART_MEASURES_STR), &id);
	if (s == ERROR)
	{
		char buf[1024];
		h2getErrMsg(errnoGet(), buf, sizeof(buf));
		printf ("Unable to create the %s poster : %s\n",poster_name, buf);
		return (NULL);
	}

	return (id);
}



/*
 * Fill a POM_SENSOR_POS on the base of information returned by the simulator
 * It is correct for the moment, but it is probably better to do it "in-place"
 */
int
create_pom_sensor_pos( int blender_date, 
							POM_SENSOR_POS* pos,			 
							const struct pom_position* robot, 
							const struct pom_position* sensor)
{
	// Fill in the Sensor to Main
#define DEG_TO_RAD(x) ((x)*M_PI/180.)
	POM_EULER* local_stm_euler = & (pos->sensorToMain.euler);
	local_stm_euler->yaw = DEG_TO_RAD(sensor->yaw);
	local_stm_euler->pitch = DEG_TO_RAD(sensor->pitch);
	local_stm_euler->roll = DEG_TO_RAD(sensor->roll);

	local_stm_euler->x = sensor->x;
	local_stm_euler->y = sensor->y;
	local_stm_euler->z = sensor->z;


	// Fill in the Main to Origin
	POM_EULER* local_mto_euler = &(pos->mainToOrigin.euler);
	local_mto_euler->yaw = DEG_TO_RAD(robot->yaw);
	local_mto_euler->pitch = DEG_TO_RAD(robot->pitch);
	local_mto_euler->roll = DEG_TO_RAD(robot->roll);
#undef DEG_TO_RAD

	local_mto_euler->x = robot->x;
	local_mto_euler->y = robot->y;
	local_mto_euler->z = robot->z;

	memcpy( &pos->mainToBase.euler, &pos->mainToOrigin.euler, sizeof(POM_EULER));

	pos->date = blender_date;
	pos->pad = 0;

	return 0;
}



/*
 * Fill the SICK_POINT array in the poster with the data from the simulator
 */
static int fill_sick_data(POM_SENSOR_POS* pom_sensor_pos, char* sick_data)
{
	/*
	image->tacq_sec = img->tacq_sec;
	image->tacq_usec = img->tacq_usec;
	create_pom_sensor_pos(img->pom_tag, &image->pos, robot, img->sensor);


	unsigned char* data = & image->data[image->dataOffset];
    size_t len = img->width * img->height;
	for (size_t j = 0 ; j < img->height; j++)
		for (size_t i = 0 ; i <  img->width; i++)
	    {
            size_t index = (j * img->width + i) * 4;
            size_t index_ = ((img->height - 1 - j) * img->width  + i);
	    	unsigned char r = (unsigned char) image_data[index];
	    	unsigned char g = (unsigned char) image_data[index+1];
	    	unsigned char b = (unsigned char) image_data[index+2];

	    	// RGB[A] -> GREY
	    	data[index_] = 0.299*r + 0.587*g + 0.114*b;
	    }
	*/

	return 0;
}

/*
 * Fill the complete poster with information computed by the simulator
 * We still have some issues with passing args from python to C, so the
 * prototyp of the function is not really correct
 */
int post_sick_poster(	POSTER_ID id,
						POM_SENSOR_POS* pom_sensor_pos,
						char* sick_data)
{
	posterTake(id, POSTER_WRITE);

	fill_sick_data(pom_sensor_pos, sick_data);

	posterGive(id);

	return 0;
}


int finalize (POSTER_ID id)
{
	posterDelete(id);

	return 0;
}
