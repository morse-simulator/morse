#include <posterLib.h>
#include <stdarg.h>
#include <pom/pomStruct.h>
#include <sick/sickStruct.h>
#include <Python.h>


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
 * Create a full POM_SENSOR_POS
 */
int create_pom_sensor_pos( int blender_date, 
							POM_SENSOR_POS* pos,			 
							const struct pom_position* robot, 
							const struct pom_position* sensor);

/*
 * Expect a list of sick_struct*
 */
int post_sick_poster(	POSTER_ID id,
						POM_SENSOR_POS* pom_sensor_pos,
						char* sick_data);

int finalize ( POSTER_ID id );
