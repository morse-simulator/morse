#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdbool.h>
#include <pthread.h>

#include "ors_viman_poster.h"

static int real_post_viman_poster(	POSTER_ID id, VimanObjectArray viman_data);
static void* thread_main(void* v);

POSTER_ID global_id;
VimanObjectArray viman_data_copy;

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

	poster_size = sizeof(VimanObjectArray);

	STATUS s = posterCreate (poster_name, poster_size, &id);
	if (s == ERROR)
	{
		char buf[1024];
		h2getErrMsg(errnoGet(), buf, sizeof(buf));
		printf ("Unable to create the %s poster : %s\n",poster_name, buf);
        *ok = 0;
		return (NULL);
	}

	printf("Succesfully created poster %s of size %zd\n", poster_name, poster_size); 
	printf ("INIT ID = %p (pointer)\n", id);

    *ok = 1;

	abort_thr = false;
	pthread_create(&thr, NULL, &thread_main, NULL);
	pthread_mutex_init(&data_mutex, NULL);
	pthread_mutex_init(&cond_mutex, NULL);
	pthread_cond_init(&cond, NULL);

	return (id);
}

/*
 * Create a data structure to hold the semantic information of the scene
 * Receive an array of strings with the names of the objects, and their number
 * Returns the newly created structure
 */
VimanObjectArray create_viman_struct(char** scene_object_list, int list_length)
{
	printf ("THIS IS ME NOT KNOWING WHAT GOES ON!\n");

	VimanObjectArray viman_data;

	viman_data.nbObjects = list_length;

	/*
	for (int i=0; i<list_length; i++)
	{
		strcpy(viman_data.objects[i].name, scene_object_list[i]);
	}
	*/

	return viman_data;
}


/*
 * Copy the values of the variables sent into the data structure
 *  at the specified index
 */
int write_matrix (VimanObjectArray viman_data, int index, matrixRef type,
	double nx, double ny, double nz,
	double ox, double oy, double oz,
	double ax, double ay, double az,
	double px, double py, double pz)
{
	viman_data.objects[index].thetaMatRob30.nx = nx;
	viman_data.objects[index].thetaMatRob30.ny = ny;
	viman_data.objects[index].thetaMatRob30.nz = nz;

	viman_data.objects[index].thetaMatRob30.ox = ox;
	viman_data.objects[index].thetaMatRob30.oy = oy;
	viman_data.objects[index].thetaMatRob30.oz = oz;

	viman_data.objects[index].thetaMatRob30.ax = ax;
	viman_data.objects[index].thetaMatRob30.ay = ay;
	viman_data.objects[index].thetaMatRob30.az = az;

	viman_data.objects[index].thetaMatRob30.px = px;
	viman_data.objects[index].thetaMatRob30.py = py;
	viman_data.objects[index].thetaMatRob30.pz = pz;

	return 0;
}


int post_viman_poster(	POSTER_ID id, VimanObjectArray viman_data)
{
	if (pthread_mutex_trylock(&data_mutex) == 0)
	{
		memcpy (&viman_data_copy, &viman_data, sizeof(VimanObjectArray));

		pthread_mutex_unlock(&data_mutex);
		pthread_mutex_lock(&cond_mutex);
		pthread_cond_signal(&cond);
		pthread_mutex_unlock(&cond_mutex);
	} else {
		// if we can't get the lock, drop it
		fprintf(stderr, "Drop the information, we are too slow !!!\n");
	}

	return 0;
}

/*
 * Fill the complete poster with information computed by the simulator
 * We still have some issues with passing args from python to C, so the
 * prototyp of the function is not really correct (image_data must be in struct
 * simu_image, and it must be an array of struct ...). Needs investigation.
 */
int real_post_viman_poster(	POSTER_ID id, VimanObjectArray viman_data)
{
	size_t offset = 0;

	/*
	ViamObjectArray* viman_array =  posterAddr(id);

	posterTake(id, POSTER_WRITE);

	// Fill in the data

	posterGive(id);
	*/

	posterWrite(id, offset, &viman_data, sizeof(VimanObjectArray));

	return 0;
}

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
