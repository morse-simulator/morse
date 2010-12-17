#include <posterLib.h>
#include <genBasic/genBasicStruct.h>
#include <genManip/genManipStruct.h>
#include <gbM/gbStruct.h>
#include <lwr/lwrStruct.h>

POSTER_ID locate_poster (char* poster_name, int* ok);
Gb_q7 read_lwr_data( POSTER_ID id );
int finalize ( POSTER_ID id );
