#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdbool.h>
#include <pthread.h>

#include "ors_viam_poster.h"

static void create_camera_calibration(viam_cameracalibration_t*, 
									  const struct simu_image_init* init);
static void create_bank_calibration(viam_bankcalibration_t*, size_t nb_images, 
		double baseline, double pixel_size);
static int fill_image(ViamImageHeader* image, const struct pom_position* pos, 
					 const struct simu_image* img, char* image_data);
static int create_pom_sensor_pos( int blender_date, 
		POM_SENSOR_POS* pos,
		const struct pom_position* robot, 
		const struct pom_position* sensor);
static int real_post_viam_poster(	POSTER_ID id,
						const struct pom_position* robot,
						size_t nb_images,
						const struct simu_image* img1,
                        char* image_data1,
						const struct simu_image* img2,
						char* image_data2
					);

static void* thread_main(void* v);

#define MAGIC_CALIBRATION_STUFF		200

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

static void 
fill_static_data(size_t i, size_t nb_images, ViamImageHeader* header, 
				 const struct simu_image_init* init,
				 size_t* offset)
{
	strncpy ( header->name.id, init->camera_name, VIAM_ID_MAX);
	header->name.id[VIAM_ID_MAX - 1]= '\0';

	create_camera_calibration(& header->calibration, init);

	header->nChannels = 1;
	header->depth = 8;
	header->width = init->width;
	header->height = init->height;
	header->widthStep = init->width * header->depth / 8 * header->nChannels; 
	header->imageSize = init->height * header->widthStep;
	header->dataOffset = 
		(nb_images - (i + 1)) * sizeof(ViamImageHeader) + *offset;
	*offset+= init->height * init->width;
}

/*
 * Create a poster, and fill it with information which don't change during the
 * execution, including bank_name, camera_name, nb_images, size of images,
 * camera calibration...
 *
 * Return a POSTER_ID on success, NULL otherwise
 *
 * you must call finalize when you don't use anymore the POSTER_ID
 */
POSTER_ID init_data (char*	poster_name, const char* bank_name, size_t nb_images, 
				 double baseline,
                 const struct simu_image_init* init1, 
				 const struct simu_image_init* init2, int* ok)
{
	size_t poster_size = 0;
	POSTER_ID id;

	poster_size = sizeof(ViamImageBank);

	// Compute the size of poster
	switch (nb_images) {
		case 1:
			poster_size+= sizeof(ViamImageHeader) + init1->width * init1->height;
			break;
		case 2:
			poster_size+= sizeof(ViamImageHeader) + init1->width * init1->height;
			poster_size+= sizeof(ViamImageHeader) + init2->width * init2->height;
			break;
		default:
			assert(false);
	}

	args.image_data1 = malloc(init1->width * init1->height * 4);
	assert(args.image_data1 != NULL);
	args.image_data2 = malloc(init2->width * init2->height * 4);
	assert(args.image_data2 != NULL);

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
	printf ("INIT ID = %p (pointer)   %d(integer)\n", id);

	ViamImageBank* bank  = posterAddr(id);

	posterTake(id, POSTER_WRITE);

	// Fill the poster with information that don't change
	strncpy ( bank->name.id, bank_name, VIAM_ID_MAX);
	bank->name.id[VIAM_ID_MAX - 1]= '\0';

	create_bank_calibration(&bank->calibration, nb_images, baseline, MAGIC_CALIBRATION_STUFF);
	bank->nImages = nb_images;

	size_t offset = 0;
	switch (nb_images) {
		case 1:
			fill_static_data(0, nb_images, &bank->image[0], init1, &offset);
			break;
		case 2:
			fill_static_data(0, nb_images, &bank->image[0], init1, &offset);
			fill_static_data(1, nb_images, &bank->image[1], init2, &offset);
			break;
		default:
			assert(false);
	}

	posterGive(id);

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
 * Build camera calibration, it must be completly revisited (in particular,
 * don't harcode size in it
 */
void
create_camera_calibration(viam_cameracalibration_t* local_calibration,
	   	const struct simu_image_init* init)
{
	double identity[9] = {1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0 };

	memcpy(local_calibration->intrinsic, identity, sizeof(identity));
	memcpy(local_calibration->rectification, identity, sizeof(identity));
	memcpy(local_calibration->rotation, identity, sizeof(identity));

	local_calibration->intrinsic[2] = init->width / 2;
	local_calibration->intrinsic[5] = init->height / 2;
	local_calibration->intrinsic[0] = MAGIC_CALIBRATION_STUFF;
	local_calibration->intrinsic[4] = MAGIC_CALIBRATION_STUFF;

	memcpy(local_calibration->intrirect, local_calibration->intrinsic, sizeof(identity));

	for (size_t i = 0; i < 5; i++)
		local_calibration->distortion[i] = 0;


	local_calibration->width = init->width;
	local_calibration->height = init->height;
}

void
create_bank_calibration(viam_bankcalibration_t* bank_calib, size_t nb_images, 
		double baseline, double pixel_size)
{
	switch (nb_images) {
		case 1:
			bank_calib->type = VIAM_CAL_MONO;
			break;
		case 2:
			bank_calib->type = VIAM_CAL_STEREO;
			bank_calib->baseline = baseline;
			// probably unused
			bank_calib->pbaseline = baseline * pixel_size ;
			break;
		default:
			assert(false);
	}
}

/*
 * Fill one image with the information computed by the simulator
 * Information in blender are stored in column major, we assume line major,
 * hence the transformation.
 *
 * We are filling too date tag, and position tag in this function
 */
static int
fill_image(ViamImageHeader* image, const struct pom_position* robot, 
								   const struct simu_image* img,
                                   char* image_data)
{
	assert(image->height == img->height);
	assert(image->width == img->width);

	image->tacq_sec = img->tacq_sec;
	image->tacq_usec = img->tacq_usec;
	create_pom_sensor_pos(img->pom_tag, &image->pos, robot, img->sensor);


	unsigned char* data = & image->data[image->dataOffset];
    size_t len = img->width * img->height;
	for (size_t j = 0 ; j < img->height; j++)
		for (size_t i = 0 ; i <  img->width; i++)
	    {
			// printf("Computing index (%zu,%zu, %zu, %zu)\n",i,j, img->width, img->height);
            size_t index = (j * img->width + i) * 4;
            size_t index_ = ((img->height - 1 - j) * img->width  + i);

			// printf("Reading image at index %zu\n", index);
	    	unsigned char r = (unsigned char) image_data[index];
	    	unsigned char g = (unsigned char) image_data[index+1];
	    	unsigned char b = (unsigned char) image_data[index+2];

			// printf("Writing image at index %zu\n", index_);
	    	// RGB[A] -> GREY
	    	data[index_] = 0.299*r + 0.587*g + 0.114*b;
	    }

	return 0;
}

int post_viam_poster(	POSTER_ID id,
						const struct pom_position* robot,
						size_t nb_images,
						const struct simu_image* img1,
                        char* image_data1,
						const struct simu_image* img2,
						char* image_data2
					)
{
	if (pthread_mutex_trylock(&data_mutex) == 0)
	{
		args.id = id;
		memcpy(&args.robot, robot, sizeof(struct pom_position));
		args.nb_images = nb_images;
		memcpy(&args.img1, img1, sizeof(struct simu_image));
		memcpy(&args.img2, img2, sizeof(struct simu_image));
		memcpy(args.image_data1, image_data1, img1->height * img1->width * 4);
		memcpy(args.image_data2, image_data2, img2->height * img2->width * 4);
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
int real_post_viam_poster(	POSTER_ID id,
						const struct pom_position* robot,
						size_t nb_images,
						const struct simu_image* img1,
                        char* image_data1,
						const struct simu_image* img2,
						char* image_data2
					)
{
	// printf ("ID at 'post_viam_poster': %d (%d cameras)\n", id, nb_images);
	ViamImageBank* bank =  posterAddr(id);
	if (bank == NULL) {
		fprintf(stderr, "calling %s but the poster has been destroyed\n", __func__);
		return -1;
	}
	posterTake(id, POSTER_WRITE);

	assert(nb_images == bank->nImages);

	switch (nb_images) {
		case 1:
			fill_image(&bank->image[0], robot, img1, image_data1);
			break;
		case 2:
			fill_image(&bank->image[0], robot, img1, image_data1);
			fill_image(&bank->image[1], robot, img2, image_data2);
			break;
		default:
			assert(false);
	}

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
		real_post_viam_poster(args.id, &args.robot, args.nb_images,
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
