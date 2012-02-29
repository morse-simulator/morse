#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "ors_viman_poster.h"

POSTER_ID global_id;
VimanObjectPublicArray global_viman_data;
VimanObjectPublicArray viman_data_copy;

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

	poster_size = sizeof(VimanObjectPublicArray);

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

/*
 * Create a data structure to hold the semantic information of the scene
 * Receive an array of strings with the names of the objects, and their number
 * Returns the newly created structure
 */
VimanObjectPublicArray generate_viman_struct()
{
	//VimanObjectPublicArray viman_data;

	return global_viman_data;
}

void set_name(VimanObjectPublicArray* viman_data, int index, char* name)
{
	strcpy(viman_data->objects[index].name, name);
}

void set_visible(VimanObjectPublicArray* viman_data, int index, int visible)
{
	viman_data->objects[index].found_Stereo = visible;
}

void set_tacq(VimanObjectPublicArray* viman_data, int index, unsigned long tacq_sec, unsigned long tacq_usec)
{
	viman_data->objects[index].tacq_sec = tacq_sec;
	viman_data->objects[index].tacq_usec = tacq_usec;
}

/*
 * Copy the values of the variables sent into the data structure
 *  at the specified index
 */
int write_matrix (VimanObjectPublicArray* viman_data, int index, int relative,
	double nx, double ny, double nz,
	double ox, double oy, double oz,
	double ax, double ay, double az,
	double px, double py, double pz)
{
    VimanThetaMat* mat; 
    if (relative){
        mat = &viman_data->objects[index].thetaMatRobot;
    }
    else {
        mat = &viman_data->objects[index].thetaMatOrigin;
    }

	mat->nx = nx;
	mat->ny = ny;
	mat->nz = nz;

	mat->ox = ox;
	mat->oy = oy;
	mat->oz = oz;

	mat->ax = ax;
	mat->ay = ay;
	mat->az = az;

	mat->px = px;
	mat->py = py;
	mat->pz = pz;

	return 0;
}

int real_post_viman_poster(POSTER_ID id, VimanObjectPublicArray* viman_data)
{
	size_t offset = 0;

	posterWrite(id, offset, viman_data, sizeof(VimanObjectPublicArray));

	return 0;
}

int finalize (POSTER_ID id)
{
	posterDelete(id);
	return 0;
}
