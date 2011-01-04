#include <posterLib.h>

struct pom_position {
    double yaw, pitch, roll;
    double x, y, z;
};


POSTER_ID init_data ( const char* poster_name, const char* reference_frame, 
					  float confidence, int* ok);
//int post_data( POSTER_ID id, double x, double y, double z,
							//double yaw, double pitch, double roll);
int post_data( POSTER_ID id, const struct pom_position* robot, 
                             const struct pom_position* sensor);
int finalize ( POSTER_ID id );
