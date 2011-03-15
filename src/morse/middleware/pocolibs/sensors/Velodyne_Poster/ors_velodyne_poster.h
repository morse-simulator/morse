#include <posterLib.h>
#include <velodyneClient.h>
#include <velodyneStruct.h>

/*
 * Structure to hold the pom data for an object
 */
struct pom_position {
	double yaw, pitch, roll;
	double x, y, z;
};


/*
 * Expect the name of the poster
 */
POSTER_ID init_data (char*	poster_name, int* ok);

/*
 * Expect a Structure of type velodyne3DImage
 */
int post_velodyne_poster(	POSTER_ID id,
                        int pom_date,
						const struct pom_position* robot,
						const struct pom_position* sensor,
                        int packets,
                        int array_length,
                        double* coordinate_array);


int finalize ( POSTER_ID id );
