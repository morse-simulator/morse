#include <posterLib.h>
#include <genPos/genPosStruct.h>

POSTER_ID locate_poster (char* poster_name, int* ok);
GENPOS_CART_SPEED read_genPos_data( POSTER_ID id );
int finalize ( POSTER_ID id );
