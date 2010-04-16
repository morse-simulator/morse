#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "ors_viam_poster.h"

//#include <posterLib.h>

static size_t poster_size;

static void create_camera_calibration(viam_cameracalibration_t*);
static void create_bank_calibration(viam_bankcalibration_t*);
static int fill_image(ViamImageHeader* image, const struct pom_position* pos, 
					 const struct simu_image* img);
static POM_SENSOR_POS create_pom_sensor_pos( int blender_date, 
		const struct pom_position* robot, 
		const struct pom_position* sensor);

void* init_data (char*	poster_name, size_t nb_images, ...)
{
	va_list ap;
	poster_size = 0;
	void *id;

	poster_size = sizeof(ViamImageBank);

	// We assume B&W images, so nChannel == 1 and depth == 1
	va_start(ap, nb_images);
	for (size_t i = 0; i < nb_images; i++)
	{
		size_t length, width;
		length = va_arg(ap, size_t);
		width = va_arg(ap, size_t);
		poster_size+= sizeof(ViamImageHeader) + length * width;
	}

	STATUS s = posterCreate (poster_name, poster_size, &id);
	if (s == ERROR)
	{
		char buf[1024];
		h2getErrMsg(errnoGet(), buf, sizeof(buf));
		printf ("Unable to create the %s poster : %s\n",poster_name, buf);
		return (NULL);
	}

	printf("Succesfully create poster %s of size %zd\n", poster_name, poster_size); 
	return (id);
}



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

static int
fill_image(ViamImageHeader* image, const struct pom_position* robot, 
								   const struct simu_image* img)
{
	strncpy( image->name.id, img->camera_name, VIAM_ID_MAX);
	image->name.id[VIAM_ID_MAX - 1] ='\0';

	image->tacq_sec = img->tacq_sec;
	image->tacq_usec = img->tacq_usec;
	image->pos = create_pom_sensor_pos(img->pom_tag, robot, img->sensor);

	create_camera_calibration(&image->calibration);

	image->nChannels = 1;
	image->depth = 8;
	image->width = img->width;
	image->height = img->height;
	image->widthStep = img->width * image->depth / 8 * image->nChannels; 
	image->imageSize = img->height * image->widthStep;

	image->dataOffset = 0;

	unsigned char* data = image->data;
	for (size_t i = 0; i < img->width * img->height; ++i)
	{
		unsigned char r = (unsigned char) img->image_data[i*4];
		unsigned char g = (unsigned char) img->image_data[i*4+1];
		unsigned char b = (unsigned char) img->image_data[i*4+2];

		// RGB[A] -> GREY
		data[i] = 0.299*r + 0.587*g + 0.114*b;
	}

	return 0;
}

int post_viam_poster(	void* id,
						const char* bank_name,
						const struct pom_position* robot,
						size_t nb_images,
						...
					)
{
	void* poster_image_ref = malloc(poster_size);
	if (poster_image_ref == NULL)
		return -1;

	ViamImageBank* bank = poster_image_ref;
	strncpy ( bank->name.id, bank_name, VIAM_ID_MAX);
	bank->name.id[VIAM_ID_MAX - 1]= '\0';

	create_bank_calibration(&bank->calibration);

	bank->nImages = nb_images;

	ViamImageHeader* image;

	va_list ap;
	va_start(ap, nb_images);
	image = poster_image_ref + sizeof(ViamImageBank);
	for (size_t i = 0; i < nb_images; i++)
	{
		struct simu_image* img = va_arg(ap, struct simu_image*);
		fill_image(image, robot, img);
		image = (void*)(image) + sizeof(ViamImageHeader) + img->width * img->height;
	}


	// Write the structure just created as a poster
	posterWrite (id, 0, poster_image_ref , poster_size);

	return 0;
}


int finalize (void* id)
{
	posterDelete(id);

	return 0;
}
