#include <posterLib.h>

POSTER_ID init_data ( const char* poster_name, const char* reference_frame, 
					  float confidence, int* ok);
int post_data( POSTER_ID id, double x, double y, double z,
							double yaw, double pitch, double roll);
int finalize ( POSTER_ID id );
