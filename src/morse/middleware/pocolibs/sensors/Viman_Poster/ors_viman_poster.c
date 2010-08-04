#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdbool.h>
#include <pthread.h>

#include <vimanStruct.h>
#include <vimanConst.h>

#include "ors_viman_poster.h"

static int real_post_viman_poster(	POSTER_ID id, VimanObjectArray viman_data);
static void* thread_main(void* v);

bool abort_thr = false;
pthread_t thr;
pthread_mutex_t data_mutex;
pthread_mutex_t cond_mutex;
pthread_cond_t cond;

struct internal_args {
	POSTER_ID id;
	struct pom_position robot;
	size_t nb_images;
	struct simu_image img1;
    char* image_data1;
	struct simu_image img2;
	char* image_data2;
};

struct internal_args args;

/*
 * Create a poster, and fill it with information which don't change during the
 * execution, including bank_name, camera_name, nb_images, size of images,
 * camera calibration...
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



int post_viman_poster(	POSTER_ID id, VimanObjectArray viman_data)
{
	if (pthread_mutex_trylock(&data_mutex) == 0)
	{
		memcpy (&args, viman_data, sizeof(VimanObjectArray));

		///VVVVV	OLD STUFF  VVVVV///
		/*
		args.id = id;
		memcpy(&args.robot, robot, sizeof(struct pom_position));
		args.nb_images = nb_images;
		memcpy(&args.img1, img1, sizeof(struct simu_image));
		memcpy(&args.img2, img2, sizeof(struct simu_image));
		memcpy(args.image_data1, image_data1, img1->height * img1->width * 4);
		memcpy(args.image_data2, image_data2, img2->height * img2->width * 4);
		*/
		///^^^^^	OLD STUFF  ^^^^^///

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
	// printf ("ID at 'post_viman_poster': %d (%d cameras)\n", id, nb_images);
	ViamImageBank* bank =  posterAddr(id);
	if (bank == NULL) {
		fprintf(stderr, "calling %s but the poster has been destroyed\n", __func__);
		return -1;
	}
	posterTake(id, POSTER_WRITE);

	posterWrite(id, offset, &viman_data, sizeof(VimanObjectArray))

	posterGive(id);

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
		real_post_viman_poster(args.id, &args.robot, args.nb_images,
									   &args.img1, args.image_data1,
									   &args.img2, args.image_data2);
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

	free(args.image_data1);
	free(args.image_data2);

	return 0;
}
