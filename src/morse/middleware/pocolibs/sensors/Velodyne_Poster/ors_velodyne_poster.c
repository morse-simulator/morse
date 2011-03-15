#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdbool.h>
#include <pthread.h>

#include <pom/pomStruct.h>
#include <velodyne/velodyneClient.h>

#include "ors_velodyne_poster.h"

static int create_pom_sensor_pos(   int blender_date, 
                                    POM_SENSOR_POS* pos,
                                    const struct pom_position* robot, 
                                    const struct pom_position* sensor);

int real_post_velodyne_poster(	POSTER_ID id,
                                int pom_date,
                                const struct pom_position* robot,
                                const struct pom_position* sensor,
                                int packets,
                                int array_length,
                                double* coordinate_array);

static int fill_velodyne_struct(    velodyne3DImage* velodyne_struct,
                                    int packets,
                                    int array_length,
                                    double* coordinate_array);

static void* thread_main(void* v);

bool abort_thr = false;
pthread_t thr;
pthread_mutex_t data_mutex;
pthread_mutex_t cond_mutex;
pthread_cond_t cond;

struct internal_args {
	POSTER_ID id;
    int date;
	struct pom_position robot;
	struct pom_position sensor;
    int packets;
    int length;
    double* coordinates;
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

	poster_size = sizeof(velodyne3DImage);

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
 * Fill a POM_SENSOR_POS on the base of information returned by the simulator
 * It is correct for the moment, but it is probably better to do it "in-place"
 */
static int
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

	memcpy( &pos->mainToBase.euler, &pos->mainToOrigin.euler, sizeof(POM_EULER));

	pos->date = blender_date;
	pos->pad = 0;

	return 0;
}



int post_velodyne_poster(	POSTER_ID id,
                        int pom_date,
						const struct pom_position* robot,
						const struct pom_position* sensor,
                        int packets,
                        int array_length,
                        double* coordinate_array
					)
{
    real_post_velodyne_poster(id, pom_date, robot, sensor, packets, array_length, coordinate_array);
    /*
	if (pthread_mutex_trylock(&data_mutex) == 0)
	{
		args.id = id;
        args.date = pom_date;
		memcpy(&args.robot, robot, sizeof(struct pom_position));
		memcpy(&args.sensor, sensor, sizeof(struct pom_position));
        args.packets = packets;
        args.length = array_length;
		memcpy(&args.coordinates, coordinate_array, sizeof(double)*array_length);
		pthread_mutex_unlock(&data_mutex);

		pthread_mutex_lock(&cond_mutex);
		pthread_cond_signal(&cond);
		pthread_mutex_unlock(&cond_mutex);
	} else {
		// if we can't get the lock, drop it
		//fprintf(stderr, ".");
		//fprintf(stderr, "Drop the information, we are too slow !!!\n");
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
int real_post_velodyne_poster(	POSTER_ID id,
                        int pom_date,
						const struct pom_position* robot,
						const struct pom_position* sensor,
                        int packets,
                        int array_length,
                        double* coordinate_array
					)
{
	// printf ("ID at 'post_velodyne_poster': %d\n", id);
	velodyne3DImage* velodyne_struct =  posterAddr(id);
	if (velodyne_struct == NULL) {
		fprintf(stderr, "calling %s but the poster has been destroyed\n", __func__);
		return -1;
	}

	posterTake(id, POSTER_WRITE);
	create_pom_sensor_pos(pom_date, &velodyne_struct->position, robot, sensor);
    fill_velodyne_struct(velodyne_struct, packets, array_length, coordinate_array);
	posterGive(id);

	//posterWrite(id, offset, &human_data, sizeof(SPARK_CONFIGURATION));

	return 0;
}

/*
 * Fill the genom data structure with the points read in simulation
 *
 * We are filling too date tag, and position tag in this function
 */
static int
fill_velodyne_struct(velodyne3DImage* velodyne_struct, int packets, int array_length, double* coordinate_array)
{
    int i=0;
    int j=0;
    // Copy the coordinates form an array into the data structure
    for (i=0; i<array_length; i+=3)
    {
        velodyne_struct->points[j].coordinates[0] = coordinate_array[i];
        velodyne_struct->points[j].coordinates[1] = coordinate_array[i+1];
        velodyne_struct->points[j].coordinates[2] = coordinate_array[i+2];
        velodyne_struct->points[j].status = VELODYNE_GOOD_3DPOINT;
        j++;
    }
    // Number of velodyne packets that have been updated
    velodyne_struct->nbLines = packets;

	return 0;
}


static void* thread_main(void * unused)
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
		real_post_velodyne_poster(args.id, args.date, &args.robot, &args.sensor, args.packets, args.length, args.coordinates);
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

	return 0;
}
