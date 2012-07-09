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

    static int previous_user = 0;

    read_poster(handler, ok, &list, sizeof(NIUT_HUMAN_LIST));

    for (int i=0; i<16; i++)
        // Use the first active user tracked
        if (list.users[i].state == NIUT_TRACKING)
        {
            // Extract the data for the specified joint of the first human in the list
            //user = list.users[i];
            //skeleton = user.skeleton;
            //joint = skeleton.joint[joint_index];
            //joint_position = joint.position;
            joint_position = list.users[i].skeleton.joint[joint_index].position;
            if (i != previous_user)
            {
                previous_user = i;
                printf ("Niut reader: Now tracking user %d\n", i+1);
            }
        }

	return (joint_position);
}
