#include <posterLib.h>

/*
 * Expect the name of the poster
 */
POSTER_ID init_data (const char*	poster_name, int* ok);
int post_platine_posture(POSTER_ID id, double pan, double tilt);
int finalize ( POSTER_ID id );
