#include <posterLib.h>
#include <Python.h>

/*
 * XXX Swig has issues with varargs or C struct array, so for moment, we only
 * consider case where nb_images == 1 or 2, hence the crappy API
 */

struct simu_image_init {
	const char* camera_name;
	size_t width, height;
	double focal;
};

/*
 * Expect a list of simu_image_init* after nb_images */
POSTER_ID init_data(char*  poster_name, const char* bank_name, size_t nb_images,
				double baseline,
                const struct simu_image_init* init1,
				const struct simu_image_init* init2, int* ok);

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
int post_viam_poster(	POSTER_ID id,
						const struct pom_position* robot,
						size_t nb_images,
						const struct simu_image* img1,
                        PyObject* img_data1,
						const struct simu_image* img2,
						PyObject* img_data2
					);

int finalize ( POSTER_ID id );
