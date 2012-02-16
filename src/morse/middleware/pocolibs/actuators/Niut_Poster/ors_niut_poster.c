#include "ors_niut_poster.h"

// Return the data structure to Python.
// It will understand, since the definition of the structure is
//  in the files included in SWIG
GEN_POINT_3D read_niut_joint_position (PosterHandler* handler, int joint_index, int *ok)
{
    NIUT_HUMAN_LIST list;
    //NIUT_USER_STR user;
    //NIUT_SKELETON_STR skeleton;
    //NIUT_JOINT_STR joint;
    GEN_POINT_3D joint_position;

    read_poster(handler, ok, &list, sizeof(NIUT_HUMAN_LIST));

    // Extract the data for the specified joint of the first human in the list
    //user = list.users[0];
    //skeleton = user.skeleton;
    //joint = skeleton.joint[joint_index];
    //joint_position = joint.position;
    joint_position = list.users[0].skeleton.joint[joint_index].position;

	return (joint_position);
}
