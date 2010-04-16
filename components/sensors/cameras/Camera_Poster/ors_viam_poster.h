#include <posterLib.h>
#include <stdarg.h>
#include <pom/pomStruct.h>
#include <viam/viamStruct.h>
#include <viam/viamStruct.h>
#include <viam/viamtypes.h>
#include <Python.h>

void* init_data (char*  poster_name, size_t nb_images, ...);

struct pom_position {
	double yaw, pitch, roll;
	double x, y, z;
};

/*
 * image_data is a RGBA image
 * and will be exported into GREY images
 */
struct simu_image {
	char* camera_name;
	size_t width, height;
	int pom_tag;
	const struct pom_position* sensor;
	unsigned long tacq_sec;
	unsigned long tacq_usec;
	unsigned char* image_data;
};



int post_viam_poster(	void* id,
						const char* bank_name,
						const struct pom_position* robot,
						size_t nb_images,
						...
					);

int finalize ( void* id );
