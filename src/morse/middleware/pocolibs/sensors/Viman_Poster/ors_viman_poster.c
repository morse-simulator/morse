#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdbool.h>
#include <pthread.h>

#include "ors_viman_poster.h"

//static int real_post_viman_poster(	POSTER_ID id, VimanObjectPublicArray viman_data);
//static void* thread_main(void* v);

POSTER_ID global_id;
VimanObjectPublicArray global_viman_data;
VimanObjectPublicArray viman_data_copy;

bool abort_thr = false;
pthread_t thr;
pthread_mutex_t data_mutex;
pthread_mutex_t cond_mutex;
pthread_cond_t cond;

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

	/*
	abort_thr = false;
	pthread_create(&thr, NULL, &thread_main, NULL);
	pthread_mutex_init(&data_mutex, NULL);
	pthread_mutex_init(&cond_mutex, NULL);
	pthread_cond_init(&cond, NULL);
	*/

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




/*
 * Copy the values of the variables sent into the data structure
 *  at the specified index
 */
int write_matrix (VimanObjectPublicArray* viman_data, int index, //matrixRef type,
	double nx, double ny, double nz,
	double ox, double oy, double oz,
	double ax, double ay, double az,
	double px, double py, double pz)
{
	viman_data->objects[index].thetaMatOrigin.nx = nx;
	viman_data->objects[index].thetaMatOrigin.ny = ny;
	viman_data->objects[index].thetaMatOrigin.nz = nz;

	viman_data->objects[index].thetaMatOrigin.ox = ox;
	viman_data->objects[index].thetaMatOrigin.oy = oy;
	viman_data->objects[index].thetaMatOrigin.oz = oz;

	viman_data->objects[index].thetaMatOrigin.ax = ax;
	viman_data->objects[index].thetaMatOrigin.ay = ay;
	viman_data->objects[index].thetaMatOrigin.az = az;

	viman_data->objects[index].thetaMatOrigin.px = px;
	viman_data->objects[index].thetaMatOrigin.py = py;
	viman_data->objects[index].thetaMatOrigin.pz = pz;

	return 0;
}


int post_viman_poster(POSTER_ID id, VimanObjectPublicArray viman_data)
{
	/*
	if (pthread_mutex_trylock(&data_mutex) == 0)
	{
		memcpy (&viman_data_copy, &viman_data, sizeof(VimanObjectPublicArray));

		pthread_mutex_unlock(&data_mutex);
		pthread_mutex_lock(&cond_mutex);
		pthread_cond_signal(&cond);
		pthread_mutex_unlock(&cond_mutex);
	} else {
		// if we can't get the lock, drop it
		fprintf(stderr, "Drop the information, we are too slow !!!\n");
	}
	*/

	return 0;
}

/*
 * Fill the complete poster with information computed by the simulator
 * We still have some issues with passing args from python to C, so the
 * prototyp of the function is not really correct (image_data must be in struct
 * simu_image, and it must be an array of struct ...). Needs investigation.
 */
int real_post_viman_poster(POSTER_ID id, VimanObjectPublicArray* viman_data)
{
	size_t offset = 0;

	//printf("viman_data.number: %d\n", viman_data->nbObjects);
	//printf("viman_data[7].name: %s\n", viman_data->objects[7].name);
	//printf("viman_data[7].thetaMatOrigin.nx: %lf\n", viman_data->objects[7].thetaMatOrigin.nx);

	/*
	ViamObjectArray* viman_array =  posterAddr(id);

	posterTake(id, POSTER_WRITE);

	// Fill in the data

	posterGive(id);
	*/

	posterWrite(id, offset, viman_data, sizeof(VimanObjectPublicArray));

	return 0;
}

/*
void* thread_main(void * unused)
{
	(void) unused;
	while (!abort_thr) 
	{
		pthread_mutex_lock(&cond_mutex);
		pthread_cond_wait(&cond, &cond_mutex);
		pthread_mutex_unlock(&cond_mutex);

		if (abort_thr)
			break;

		pthread_mutex_lock(&data_mutex);
		real_post_viman_poster(global_id, viman_data_copy);
		pthread_mutex_unlock(&data_mutex);
	}
	
	pthread_exit(NULL);
}
*/

int finalize (POSTER_ID id)
{
	posterDelete(id);

	abort_thr = true;

	pthread_mutex_lock(&cond_mutex);
	pthread_cond_signal(&cond);
	pthread_mutex_unlock(&cond_mutex);

	pthread_join(thr, NULL);
	pthread_cond_destroy(&cond);
	pthread_mutex_destroy(&cond_mutex);
	pthread_mutex_destroy(&data_mutex);

	//free(viman_data_copy);

	return 0;
}
