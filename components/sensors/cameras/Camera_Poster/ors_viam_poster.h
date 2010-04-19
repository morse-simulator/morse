#include <posterLib.h>
#include <stdarg.h>
#include <pom/pomStruct.h>
#include <viam/viamStruct.h>
#include <viam/viamStruct.h>
#include <viam/viamtypes.h>
#include <Python.h>

struct simu_image_init {
	const char* camera_name;
	size_t width, height;
};

/*
 * Expect a list of simu_image_init* after nb_images */
void* init_data(char*  poster_name, const char* bank_name, size_t nb_images,
                const struct simu_image_init* init);

struct pom_position {
	double yaw, pitch, roll;
	double x, y, z;
};


/*
 * image_data is a RGBA image
 * and will be exported into GREY images
 */
struct simu_image {
	size_t width, height;
	int pom_tag;
	const struct pom_position* sensor;
	unsigned long tacq_sec;
	unsigned long tacq_usec;
};


/*
 * Expect a list of simu_image* after nb_images
 */
int post_viam_poster(	void* id,
						const struct pom_position* robot,
						size_t nb_images,
						const struct simu_image* img,
                        char *img_data
					);

int finalize ( void* id );
