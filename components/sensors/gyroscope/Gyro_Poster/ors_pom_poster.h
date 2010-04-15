#include <posterLib.h>

void* init_data ( char* poster_name );
int post_data( void* id, double x, double y, double z, double yaw, double pitch, double roll, int date );
int finalize ( void* id );
