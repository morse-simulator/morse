#include <posterLib.h>
#include <pom/pomStruct.h>

POSTER_ID locate_poster (char* poster_name, int* ok);
POM_EULER read_platine_data( POSTER_ID id );
int finalize ( POSTER_ID id );
