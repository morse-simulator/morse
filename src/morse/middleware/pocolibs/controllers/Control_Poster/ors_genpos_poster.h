#include <posterLib.h>
#include <Python.h>
#include <genPos/genPosStruct.h>

POSTER_ID locate_poster (char* poster_name, int* ok);
//PyObject* read_genPos_data( POSTER_ID id, float v, float w );
GENPOS_CART_SPEED read_genPos_data( POSTER_ID id );
int finalize ( POSTER_ID id );
