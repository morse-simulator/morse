#include <posterLib.h>
#include <vimanConst.h>
#include <vimanStruct.h>

typedef enum matrixRef {CameraRef, RobotRef, WorldRef} matrixRef;

/*
 * Expect the name of the poster
 */
POSTER_ID init_data (char*	poster_name, int* ok);

/*
 * Expect a Structure of type VimanObjectArray
 */
int post_viman_poster(	POSTER_ID id, VimanObjectArray viman_data);


VimanObjectArray create_viman_struct(char** scene_object_list, int list_length);

int write_matrix (VimanObjectArray viman_data, int index, matrixRef type,
	double nx, double ny, double nz,
	double ox, double oy, double oz,
	double ax, double ay, double az,
	double px, double py, double pz);

int finalize ( POSTER_ID id );
