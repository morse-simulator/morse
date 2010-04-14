#include <posterLib.h>
#include <Python.h>
#include <viam/viamStruct.h>

void* init_data (char*  poster_name);
int post_viam_data( void* id, double x, double y, double z, double yaw, double pitch, double roll )
int finalize ( void* id );
