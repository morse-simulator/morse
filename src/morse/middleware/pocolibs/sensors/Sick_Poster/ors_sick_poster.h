#include <posterLib.h>
#include <posterLib.h>

#include <pom/pomStruct.h> // XXX Fix the interface

POSTER_ID init_data(char*  poster_name);

struct pom_position {
	double yaw, pitch, roll;
	double x, y, z;
};


/*
 * image_data is a RGBA image
 * and will be exported into GREY images
 */
struct sick_struct {
	size_t width, height;
	int pom_tag;
	const struct pom_position* sensor;
	unsigned long tacq_sec;
	unsigned long tacq_usec;
};



/*
 * Expect a list of sick_struct*
 */
int post_sick_poster(	POSTER_ID id,
						POM_SENSOR_POS* pom_sensor_pos,
						char* sick_data);

int finalize ( POSTER_ID id );
