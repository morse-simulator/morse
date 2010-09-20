#include <posterLib.h>
#include <mhpConst.h>
#include <mhpStruct.h>

/*
 * Expect the name of the poster
 */
POSTER_ID init_data (char*	poster_name, int* ok);

/*
 * Expect a Structure of type MHP_HUMAN_CONFIGURATION
 */
int post_human_poster(POSTER_ID id, MHP_HUMAN_CONFIGURATION* human_data);

MHP_HUMAN_CONFIGURATION generate_human_struct();

void set_dof(MHP_HUMAN_CONFIGURATION* human_data, double dof[40]);


int finalize ( POSTER_ID id );
