#include <posterLib.h>

/*
 * Expect the name of the poster
 */
POSTER_ID init_data (char*	poster_name, int* ok);

/*
 * Expect an array of 49 floats describing the human posture
 * See http://www.openrobots.org/morse/doc/user/sensors/human_posture.html
 */
int post_human_poster(POSTER_ID id, double dof[40]);

int finalize ( POSTER_ID id );
