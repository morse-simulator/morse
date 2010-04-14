#include <posterLib.h>
#include <Python.h>
#include <pom/pomStruct.h>
#include <viam/viamStruct.h>
#include <viam/viamStruct.h>
#include <viam/viamtypes.h>

void* init_data (char*  poster_name);

POM_SENSOR_POS create_pom_sensor_pos( int blender_date,
						double robot_x,
						double robot_y,
						double robot_z,
						double robot_yaw,
						double robot_pitch,
						double robot_roll,
						double sensor_x,
						double sensor_y,
						double sensor_z,
						double sensor_yaw,
						double sensor_pitch,
						double sensor_roll);

viam_cameracalibration_t create_calibration(void);

int post_viam_poster(	void* id,
						char* camera_name,
						int blender_date,
						double robot_x,
						double robot_y,
						double robot_z,
						double robot_yaw,
						double robot_pitch,
						double robot_roll,
						double sensor_x,
						double sensor_y,
						double sensor_z,
						double sensor_yaw,
						double sensor_pitch,
						double sensor_roll,
						unsigned long tacq_sec,
						unsigned long tacq_usec,
						int cam_channels,
						int cam_depth,
						int cam_width,
						int cam_height,
						unsigned char* image_data);

int finalize ( void* id );
