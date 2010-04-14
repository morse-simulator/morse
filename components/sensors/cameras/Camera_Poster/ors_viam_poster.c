#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "ors_viam_poster.h"

//#include <posterLib.h>


void* locate_poster (char*	poster_name)
{
	void* id;

	STATUS s = posterFind (poster_name, &id);
	if (s == ERROR)
	{
		char buf[1024];
		h2getErrMsg(errnoGet(), buf, sizeof(buf));
		printf ("Unable to locate the GENPOS poster : %s\n",buf);
		return (NULL);
	}
	/*
	else
		printf ("INIT ID = %p (pointer)   %d(integer)\n", id);
	*/

	return (id);
}


/*
//int read_genPos_data( void* id, float *v, float *w )
PyObject* read_genPos_data( void* id, float v, float w )
{
	GENPOS_CART_SPEED local_genPos;
	int offset = 0;

	posterRead (id, offset, &local_genPos, sizeof(GENPOS_CART_SPEED));

	// Read the variables we need for the speed
	v = local_genPos.v;
	w = local_genPos.w;

	printf ("DATA READ FROM POSTER:");
	printf ("\tv = %.4f", v);
	printf ("\tw = %.4f\n", w);

	PyObject *tuple, *list;

	// tuple = Py_BuildValue("(iis)", 1, 2, "three");
	// list = Py_BuildValue("[iis]", 1, 2, "three");
	tuple = Py_BuildValue("(ii)", v, w);


	PyObject* py_v = PyInt_FromLong(v);
	PyObject* py_w = PyInt_FromLong(w);

	PyObject* resTuple = PyTuple_New(2);

	PyTuple_SetItem(resTuple, 0, py_v);
	PyTuple_SetItem(resTuple, 1, py_w);

	return (resTuple);
}
*/


//int read_genPos_data( void* id, float *v, float *w )
GENPOS_CART_SPEED read_genPos_data( void* id )
{
	GENPOS_CART_SPEED local_genPos;
	int offset = 0;
	float v;
	float w;

	posterRead (id, offset, &local_genPos, sizeof(GENPOS_CART_SPEED));

	// Read the variables we need for the speed
	v = local_genPos.v;
	w = local_genPos.w;

	// printf ("DATA READ FROM POSTER:");
	// printf ("\tv = %.4f", v);
	// printf ("\tw = %.4f\n", w);

	return (local_genPos);
}


int finalize (void* id)
{
	posterDelete(id);

	return 0;
}
