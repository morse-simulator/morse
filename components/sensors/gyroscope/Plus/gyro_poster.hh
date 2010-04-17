#include <posterLib.h>
#include <pom/pomStruct.h>
#include <poster_exception.hh>
#include <poster_locker.hh>
#include <poster_write.hh>

PosterWriter<POM_POS> init_data ( char* poster_name );
int post_data( PosterWriter<POM_POS> writer, double x, double y, double z, double yaw, double pitch, double roll, int date );
int finalize ( PosterWriter<POM_POS> writer );
