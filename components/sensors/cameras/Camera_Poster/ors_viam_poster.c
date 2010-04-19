#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "ors_viam_poster.h"

static void create_camera_calibration(viam_cameracalibration_t*);
static void create_bank_calibration(viam_bankcalibration_t*);
static int fill_image(ViamImageHeader* image, const struct pom_position* pos, 
					 const struct simu_image* img, char* image_data);
static POM_SENSOR_POS create_pom_sensor_pos( int blender_date, 
		const struct pom_position* robot, 
		const struct pom_position* sensor);

/*
 * Create a poster, and fill it with information which don't change during the
 * execution, including bank_name, camera_name, nb_images, size of images,
 * camera calibration...
 *
 * Return a POSTER_ID on success, NULL otherwise
 *
 * you must call finalize when you don't use anymore the POSTER_ID
 */
void* init_data (char*	poster_name, const char* bank_name, size_t nb_images, 
                 const struct simu_image_init* init)
{
	size_t poster_size = 0;
	void *id;

	poster_size = sizeof(ViamImageBank);

	// Compute the size of poster
	for (size_t i = 0; i < nb_images; i++)
		poster_size+= sizeof(ViamImageHeader) + init[i].width * init[i].height;

	STATUS s = posterCreate (poster_name, poster_size, &id);
	if (s == ERROR)
	{
		char buf[1024];
		h2getErrMsg(errnoGet(), buf, sizeof(buf));
		printf ("Unable to create the %s poster : %s\n",poster_name, buf);
		return (NULL);
	}

	printf("Succesfully create poster %s of size %zd\n", poster_name, poster_size); 

	ViamImageBank* bank  = posterAddr(id);

	posterTake(id, POSTER_WRITE);

	// Fill the poster with information that don't change
	strncpy ( bank->name.id, bank_name, VIAM_ID_MAX);
	bank->name.id[VIAM_ID_MAX - 1]= '\0';

	create_bank_calibration(&bank->calibration);
	bank->nImages = nb_images;

	size_t offset = 0;
	size_t size_headers = sizeof(ViamImageHeader) * nb_images;
	for (size_t i = 0; i < nb_images; i++)
	{
		strncpy ( bank->image[i].name.id, init[i].camera_name, VIAM_ID_MAX);
		bank->image[i].name.id[VIAM_ID_MAX - 1]= '\0';

		create_camera_calibration(& bank->image[i].calibration);

		bank->image[i].nChannels = 1;
		bank->image[i].depth = 8;
		bank->image[i].width = init[i].width;
		bank->image[i].height = init[i].height;
		bank->image[i].widthStep = init[i].width * bank->image[i].depth / 8 * bank->image[i].nChannels; 
		bank->image[i].imageSize = init[i].height * bank->image[i].widthStep;
		bank->image[i].dataOffset = 
			(nb_images - (i + 1)) * sizeof(ViamImageHeader) + offset;
		offset+= init[i].height * init[i].width;
	}

	posterGive(id);

	return (id);
}



/*
 * Fill a POM_SENSOR_POS on the base of information returned by the simulator
 * It is correct for the moment, but it is probably better to do it "in-place"
 */
POM_SENSOR_POS 
create_pom_sensor_pos( int blender_date, const struct pom_position* robot, 
										 const struct pom_position* sensor)
{
	POM_SENSOR_POS local_sensor_pos;

	// Declare local versions of the structures used
	POM_EULER_V local_sensor_to_main;
	POM_EULER_V local_main_to_base;
	POM_EULER_V local_main_to_origin;
	POM_EULER_V local_v_local;


	// Fill in the Sensor to Main
#define DEG_TO_RAD(x) ((x)*M_PI/180.)
	POM_EULER local_stm_euler;
	local_stm_euler.yaw = DEG_TO_RAD(sensor->yaw);
	local_stm_euler.pitch = DEG_TO_RAD(sensor->pitch);
	local_stm_euler.roll = DEG_TO_RAD(sensor->roll);

	local_stm_euler.x = sensor->x;
	local_stm_euler.y = sensor->y;
	local_stm_euler.z = sensor->z;

	POM_EULER_VARIANCES sensor_var;

	// Fill in the POM_POS_EULER_V
	local_sensor_to_main.euler = local_stm_euler;
	local_sensor_to_main.var = sensor_var;


	// Fill in the Main to Origin
	POM_EULER local_mto_euler;
	local_mto_euler.yaw = DEG_TO_RAD(robot->yaw);
	local_mto_euler.pitch = DEG_TO_RAD(robot->pitch);
	local_mto_euler.roll = DEG_TO_RAD(robot->roll);
#undef DEG_TO_RAD

	local_mto_euler.x = robot->x;
	local_mto_euler.y = robot->y;
	local_mto_euler.z = robot->z;

	POM_EULER_VARIANCES robot_var;

	// Fill in the POM_POS_EULER_V
	local_sensor_to_main.euler = local_mto_euler;
	local_sensor_to_main.var = robot_var;

	// Fill in the SENSOR_POS
	local_sensor_pos.date = blender_date;
	local_sensor_pos.pad = 0;
	local_sensor_pos.sensorToMain = local_sensor_to_main;
	local_sensor_pos.mainToBase = local_main_to_origin;
	local_sensor_pos.mainToOrigin = local_main_to_origin;
	local_sensor_pos.VLocal = local_v_local;

	return local_sensor_pos;
}


/*
 * Build camera calibration, it must be completly revisited (in particular,
 * don't harcode size in it
 */
void
create_camera_calibration(viam_cameracalibration_t* local_calibration)
{
	double calibration_matrix[9] = {14.63, 0.0, 256.0, 0.0, 14.63, 256.0, 0.0, 0.0, 1.0}; 

	int i;
	for (i=0; i<9; i++)
		local_calibration->intrinsic[i] = calibration_matrix[i];
	/*
	local_calibration->intrirect = {};
	local_calibration->distortion = {};
	local_calibration->rectification = {};
	local_calibration->rotation = {};
	*/
	local_calibration->width = 512;
	local_calibration->height = 512;
}

void
create_bank_calibration(viam_bankcalibration_t* bank_calib)
{
	(void) bank_calib;
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
	image->pos = create_pom_sensor_pos(img->pom_tag, robot, img->sensor);

	unsigned char* data = & image->data[image->dataOffset];
    size_t len = img->width * img->height;
	for (size_t i = 0 ; i <  img->width; i++)
	    for (size_t j = 0 ; j < img->height; j++)
	    {
            size_t index = (j * img->width + i) * 4;
            size_t index_ = ((img->height - j) * img->width + i);
	    	unsigned char r = (unsigned char) image_data[index];
	    	unsigned char g = (unsigned char) image_data[index+1];
	    	unsigned char b = (unsigned char) image_data[index+2];

	    	// RGB[A] -> GREY
	    	data[index_] = 0.299*r + 0.587*g + 0.114*b;
	    }

	return 0;
}

/*
 * Fill the complete poster with information computed by the simulator
 * We still have some issues with passing args from python to C, so the
 * prototyp of the function is not really correct (image_data must be in struct
 * simu_image, and it must be an array of struct ...). Needs investigation.
 */
int post_viam_poster(	void* id,
						const struct pom_position* robot,
						size_t nb_images,
						const struct simu_image* img,
                        char* image_data
					)
{
	ViamImageBank* bank =  posterAddr(id);
	posterTake(id, POSTER_WRITE);

	assert(nb_images == bank->nImages);

	for (size_t i = 0; i < nb_images; i++)
	{
		ViamImageHeader* image = &bank->image[i];
		fill_image(image, robot, &img[i], image_data);
	}
	posterGive(id);

	return 0;
}


int finalize (void* id)
{
	posterDelete(id);

	return 0;
}
