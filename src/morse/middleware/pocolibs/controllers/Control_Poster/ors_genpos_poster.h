#include <posterLib.h>
#include <Python.h>
#include <genPos/genPosStruct.h>

void* locate_poster (char*	poster_name);
//PyObject* read_genPos_data( void* id, float v, float w );
GENPOS_CART_SPEED read_genPos_data( void* id );
int finalize ( void* id );
