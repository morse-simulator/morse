#include <posterLib.h>

/*
 * Expect the name of the poster
 */
POSTER_ID init_data (const char*	poster_name, int* ok);
int post_target_poster(POSTER_ID id, int has_target, double x, double y);
int finalize ( POSTER_ID id );
