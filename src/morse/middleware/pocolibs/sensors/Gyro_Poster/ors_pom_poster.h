#include <posterLib.h>
#include <pom/pomStruct.h>

POSTER_ID init_data ( char* poster_name );
int post_data( POSTER_ID id, double x, double y, double z, double yaw, double pitch, double roll, int date );
int finalize ( POSTER_ID id );
