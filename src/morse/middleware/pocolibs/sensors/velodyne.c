#include <Python.h>
#include "structmember.h"

#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdbool.h>
#include <pthread.h>

#include <pom/pomStruct.h>
#include <velodyne/velodyneClient.h>
#include <t3d/t3d.h>
#include <posterLib.h>

typedef struct {
    PyObject_HEAD
	size_t height, width, size, nb_rot;
	POSTER_ID id;
	velodyne3DImage img;

	size_t current_rot;
} Velodyne ;

typedef struct {
	PyObject_HEAD
	size_t nb_pts;
	int pom_tag;
	double x_rob, y_rob, z_rob, yaw_rob, pitch_rob, roll_rob;
	double x_cam, y_cam, z_cam, yaw_cam, pitch_cam, roll_cam;
} VelodyneSimu;

	
static int
Velodyne_init(Velodyne *self, PyObject *args, PyObject *kwds)
{
    PyObject *images;
	char* poster_name;

    if (!PyArg_ParseTuple(args, "snnn", &poster_name, &(self->width), &(self->height), &(self->nb_rot)))
		return -1;

	self->size = self->width * self->height;
	if (self->size * self->nb_rot > VELODYNE_3D_IMAGE_WIDTH * VELODYNE_3D_IMAGE_HEIGHT) {
		PyErr_Format(PyExc_ValueError, "Input image is too large (%zd) for internal buffer (%zd)\n",
				self->size * self->nb_rot,
				VELODYNE_3D_IMAGE_HEIGHT * VELODYNE_3D_IMAGE_WIDTH);
		return -1;
	}

	size_t poster_size = sizeof(velodyne3DImage);

	STATUS s = posterCreate (poster_name, poster_size, &self->id);
	if (s == ERROR)
	{
		char buf[1024];
		h2getErrMsg(errnoGet(), buf, sizeof(buf));
		PyErr_Format(PyExc_RuntimeError,
				"Unable to create the %s poster : %s\n", poster_name, buf);
		return -1;
	}


	printf("Succesfully created poster %s of size %zd\n", poster_name, poster_size); 


	self->img.width = VELODYNE_3D_IMAGE_WIDTH;
	self->img.maxScanWidth = VELODYNE_3D_IMAGE_WIDTH;
	self->img.height = (int)(ceil)(self->size * self->nb_rot / VELODYNE_3D_IMAGE_WIDTH);

	self->current_rot = 0;

    return 0;
}


static PyObject*
Velodyne_post(Velodyne* self, PyObject* args)
{
	VelodyneSimu* info;
	PyObject* data;
	Py_buffer buf;
	size_t i;

    if (!PyArg_ParseTuple(args, "OO", &info, &data))
        return NULL;


	velodyne3DImage* im3d = &self->img;

	PyObject_GetBuffer(data, &buf, PyBUF_SIMPLE);

	float* p = (float*)buf.buf;

	if (self->current_rot == 0) {
		// clean the whole point array
		memset(im3d->points, 0, sizeof(im3d->points));

		// store the position
		POM_EULER* local_stm_euler = & (im3d->position.sensorToMain.euler);
		local_stm_euler->yaw = info->yaw_cam;
		local_stm_euler->pitch = info->pitch_cam;
		local_stm_euler->roll = info->roll_cam;

		local_stm_euler->x = info->x_cam;
		local_stm_euler->y = info->y_cam;
		local_stm_euler->z = info->z_cam;

		// Fill in the Main to Origin
		POM_EULER* local_mto_euler = &(im3d->position.mainToOrigin.euler);
		local_mto_euler->yaw = info->yaw_rob;
		local_mto_euler->pitch = info->pitch_rob;
		local_mto_euler->roll = info->roll_rob;

		local_mto_euler->x = info->x_rob;
		local_mto_euler->y = info->y_rob;
		local_mto_euler->z = info->z_rob;

		memcpy( &im3d->position.mainToBase.euler, 
				&im3d->position.mainToOrigin.euler, sizeof(POM_EULER));

		im3d->position.date = info->pom_tag;

		// fill the poster
		velodyne3DPoint* points = im3d->points;

		for (i = 0; i < info->nb_pts; i++) 
		{
			points->coordinates[0] = *p;
			points->coordinates[1] = *(p+1);
			points->coordinates[2] = *(p+2);
			if (points->coordinates[0] == 0.0 &&
				points->coordinates[1] == 0.0 &&
				points->coordinates[2] == 0.0)

				points->status = VELODYNE_BAD_3DPOINT;
			else
				points->status = VELODYNE_GOOD_3DPOINT;
			points++;
			p += 3;
		}

		for (;i < self->size; i++, points++) 
			points->status = VELODYNE_BAD_3DPOINT;
	} else {
		// fill the poster, but we need to transform the position of each point
		// in the frame of the  first scan
		T3D sensor2ToMain2;
		T3D main2ToOrigin;
		T3D sensor2ToOrigin;
		T3D sensor1ToMain1;
		T3D main1ToOrigin;
		T3D sensor1ToOrigin;
		T3D originToSensor1;
		T3D sensor2ToSensor1;

		t3dInit(& sensor2ToMain2, T3D_BRYAN, T3D_ALLOW_CONVERSION);
		t3dInit(& main2ToOrigin, T3D_BRYAN, T3D_ALLOW_CONVERSION);
		t3dInit(& sensor2ToOrigin, T3D_BRYAN, T3D_ALLOW_CONVERSION);
		t3dInit(& sensor1ToMain1, T3D_BRYAN, T3D_ALLOW_CONVERSION);
		t3dInit(& main1ToOrigin, T3D_BRYAN, T3D_ALLOW_CONVERSION);
		t3dInit(& sensor1ToOrigin, T3D_BRYAN, T3D_ALLOW_CONVERSION);
		t3dInit(& originToSensor1, T3D_BRYAN, T3D_ALLOW_CONVERSION);
		t3dInit(& sensor2ToSensor1, T3D_BRYAN, T3D_ALLOW_CONVERSION);

		sensor2ToMain2.euler.euler[0] = info->yaw_cam;
		sensor2ToMain2.euler.euler[1] = info->pitch_cam;
		sensor2ToMain2.euler.euler[2] = info->roll_cam;
		sensor2ToMain2.euler.euler[3] = info->x_cam;
		sensor2ToMain2.euler.euler[4] = info->y_cam;
		sensor2ToMain2.euler.euler[5] = info->z_cam;

		main2ToOrigin.euler.euler[0] = info->yaw_rob;
		main2ToOrigin.euler.euler[1] = info->pitch_rob;
		main2ToOrigin.euler.euler[2] = info->roll_rob;
		main2ToOrigin.euler.euler[3] = info->x_rob;
		main2ToOrigin.euler.euler[4] = info->y_rob;
		main2ToOrigin.euler.euler[5] = info->z_rob;

		main1ToOrigin.euler.euler[0] = self->img.position.mainToOrigin.euler.yaw;
		main1ToOrigin.euler.euler[1] = self->img.position.mainToOrigin.euler.pitch;
		main1ToOrigin.euler.euler[2] = self->img.position.mainToOrigin.euler.roll;
		main1ToOrigin.euler.euler[3] = self->img.position.mainToOrigin.euler.x;
		main1ToOrigin.euler.euler[4] = self->img.position.mainToOrigin.euler.y;
		main1ToOrigin.euler.euler[5] = self->img.position.mainToOrigin.euler.z;

		sensor1ToMain1.euler.euler[0] = self->img.position.sensorToMain.euler.yaw;
		sensor1ToMain1.euler.euler[1] = self->img.position.sensorToMain.euler.pitch;
		sensor1ToMain1.euler.euler[2] = self->img.position.sensorToMain.euler.roll;
		sensor1ToMain1.euler.euler[3] = self->img.position.sensorToMain.euler.x;
		sensor1ToMain1.euler.euler[4] = self->img.position.sensorToMain.euler.y;
		sensor1ToMain1.euler.euler[5] = self->img.position.sensorToMain.euler.z;

		t3dCompIn (&sensor2ToOrigin, &sensor2ToMain2, &main2ToOrigin);
		t3dCompIn (&sensor1ToOrigin, &sensor1ToMain1, &main1ToOrigin);
		t3dInvertIn (&originToSensor1, &sensor1ToOrigin);
		t3dCompIn (&sensor2ToSensor1, &sensor2ToOrigin, &originToSensor1);

		if (!t3dOk (&sensor2ToSensor1)) {
			fprintf (stderr, "Failed to compute transformation\n");
			Py_RETURN_NONE;
		}

		velodyne3DPoint* points = im3d->points + self->current_rot * self->size;
		for (i = 0; i < info->nb_pts; i++) 
		{
			double *c;
			double x = *p; 
			double y = *(p+1);
			double z = *(p+2);
			if (x == 0.0 && y == 0.0 && z == 0.0) {
				points->status = VELODYNE_BAD_3DPOINT;
			} else {
				points->status = VELODYNE_GOOD_3DPOINT;
				c = sensor2ToSensor1.matrix.matrix;
				// matrix mulplication inlined
				points->coordinates[0]
					= (float) (x * *c + y * *(c + 1) + z * *(c + 2) + *(c + 3));
				c += 4;
				points->coordinates[1]
					= (float) (x * *c + y * *(c + 1) + z * *(c + 2) + *(c + 3));
				c += 4;
				points->coordinates[2]
					= (float) (x * *c + y * *(c + 1) + z * *(c + 2) + *(c + 3));
			}
			points++;
			p += 3;
		}

		for (;i < self->size; i++, points++) 
			points->status = VELODYNE_BAD_3DPOINT;

	}
	PyBuffer_Release(&buf);

	self->current_rot = (self->current_rot + 1) % self->nb_rot;

	// we have do one turn, just fill the poster
	
	if (self->current_rot == 0) {
		velodyne3DImage* p_im3d  = posterAddr(self->id);
		posterTake(self->id, POSTER_WRITE);
		memcpy(p_im3d, im3d, sizeof(*im3d));
		posterGive(self->id);
	}

	Py_RETURN_NONE;
}

static PyObject*
Velodyne_finalize(Velodyne* self)
{
	posterDelete(self->id);

	Py_RETURN_NONE;
}


static PyMethodDef Velodyne_methods[] = {
    {"post", (PyCFunction)Velodyne_post, METH_VARARGS,
     "Post an image in the poster"
    },
	{"finalize", (PyCFunction)Velodyne_finalize, METH_NOARGS,
	 "Cleanup ressource for velodyne poster"},
    {NULL}  /* Sentinel */
};

static PyTypeObject VelodyneType = {
    PyVarObject_HEAD_INIT(NULL, 0)
    "velodyne.Velodyne",             /* tp_name */
    sizeof(Velodyne),             /* tp_basicsize */
    0,                         /* tp_itemsize */
    0, 							/* tp_dealloc */
    0,                         /* tp_print */
    0,                         /* tp_getattr */
    0,                         /* tp_setattr */
    0,                         /* tp_reserved */
    0,                         /* tp_repr */
    0,                         /* tp_as_number */
    0,                         /* tp_as_sequence */
    0,                         /* tp_as_mapping */
    0,                         /* tp_hash  */
    0,                         /* tp_call */
    0,                         /* tp_str */
    0,                         /* tp_getattro */
    0,                         /* tp_setattro */
    0,                         /* tp_as_buffer */
    Py_TPFLAGS_DEFAULT, 		/* tp_flags */
    "Velodyne wrapper objects",     /* tp_doc */
    0,                         /* tp_traverse */
    0,                         /* tp_clear */
    0,                         /* tp_richcompare */
    0,                         /* tp_weaklistoffset */
    0,                         /* tp_iter */
    0,                         /* tp_iternext */
    Velodyne_methods,             /* tp_methods */
    0,				            /* tp_members */
    0,                         /* tp_getset */
    0,                         /* tp_base */
    0,                         /* tp_dict */
    0,                         /* tp_descr_get */
    0,                         /* tp_descr_set */
    0,                         /* tp_dictoffset */
    (initproc)Velodyne_init,      /* tp_init */
    0,                         /* tp_alloc */
    0,                 /* tp_new */
};

static PyMemberDef VelodyneSimu_members[] = {
	{"x_rob", T_DOUBLE, offsetof(VelodyneSimu, x_rob), 0, "x_rob"},
	{"y_rob", T_DOUBLE, offsetof(VelodyneSimu, y_rob), 0, "y_rob"},
	{"z_rob", T_DOUBLE, offsetof(VelodyneSimu, z_rob), 0, "z_rob"},
	{"yaw_rob", T_DOUBLE, offsetof(VelodyneSimu, yaw_rob), 0, "yaw_rob"},
	{"pitch_rob", T_DOUBLE, offsetof(VelodyneSimu, pitch_rob), 0, "pitch_rob"},
	{"roll_rob", T_DOUBLE, offsetof(VelodyneSimu, roll_rob), 0, "roll_rob"},
	{"x_cam", T_DOUBLE, offsetof(VelodyneSimu, x_cam), 0, "x_cam"},
	{"y_cam", T_DOUBLE, offsetof(VelodyneSimu, y_cam), 0, "y_cam"},
	{"z_cam", T_DOUBLE, offsetof(VelodyneSimu, z_cam), 0, "z_cam"},
	{"yaw_cam", T_DOUBLE, offsetof(VelodyneSimu, yaw_cam), 0, "yaw_cam"},
	{"pitch_cam", T_DOUBLE, offsetof(VelodyneSimu, pitch_cam), 0, "pitch_cam"},
	{"roll_cam", T_DOUBLE, offsetof(VelodyneSimu, roll_cam), 0, "roll_cam"},
	{"nb_pts", T_PYSSIZET, offsetof(VelodyneSimu, nb_pts), 0, "nb_pts"},
	{"pom_tag", T_INT, offsetof(VelodyneSimu, pom_tag), 0, "pom_tag"},
    {NULL}  /* Sentinel */
};

static PyMethodDef VelodyneSimu_methods[] = {
    {NULL}  /* Sentinel */
};

static int
VelodyneSimu_init(VelodyneSimu *self, PyObject *args, PyObject *kwds)
{
	return 0;
}

static PyTypeObject VelodyneSimuType = {
    PyVarObject_HEAD_INIT(NULL, 0)
    "velodyne.VelodyneSimu",      /* tp_name */
    sizeof(VelodyneSimu),     /* tp_basicsize */
    0,                         /* tp_itemsize */
    0, 							/* tp_dealloc */
    0,                         /* tp_print */
    0,                         /* tp_getattr */
    0,                         /* tp_setattr */
    0,                         /* tp_reserved */
    0,                         /* tp_repr */
    0,                         /* tp_as_number */
    0,                         /* tp_as_sequence */
    0,                         /* tp_as_mapping */
    0,                         /* tp_hash  */
    0,                         /* tp_call */
    0,                         /* tp_str */
    0,                         /* tp_getattro */
    0,                         /* tp_setattro */
    0,                         /* tp_as_buffer */
    Py_TPFLAGS_DEFAULT,		   /* tp_flags */
    "Velodyne simu image wrapper objects",    /* tp_doc */
    0,                         /* tp_traverse */
    0,                         /* tp_clear */
    0,                         /* tp_richcompare */
    0,                         /* tp_weaklistoffset */
    0,                         /* tp_iter */
    0,                         /* tp_iternext */
    VelodyneSimu_methods,             /* tp_methods */
    VelodyneSimu_members,             /* tp_members */
    0,                         /* tp_getset */
    0,                         /* tp_base */
    0,                         /* tp_dict */
    0,                         /* tp_descr_get */
    0,                         /* tp_descr_set */
    0,                         /* tp_dictoffset */
    0,						   /* tp_init */
    0,                         /* tp_alloc */
    0,                 /* tp_new */
};



static PyModuleDef velodynemodule = {
    PyModuleDef_HEAD_INIT,
    "velodyne",
    "Example module that creates an extension type.",
    -1,
    NULL, NULL, NULL, NULL, NULL
};

PyMODINIT_FUNC
PyInit_velodyne(void)
{
    PyObject* m;

	VelodyneType.tp_new = PyType_GenericNew;
	VelodyneSimuType.tp_new = PyType_GenericNew;

    if (PyType_Ready(&VelodyneType) < 0)
        return NULL;
	if (PyType_Ready(&VelodyneSimuType) < 0)
		return NULL;

    m = PyModule_Create(&velodynemodule);
    if (m == NULL)
        return NULL;

    Py_INCREF(&VelodyneType);
    PyModule_AddObject(m, "Velodyne", (PyObject *)&VelodyneType);

	Py_INCREF(&VelodyneSimuType);
    PyModule_AddObject(m, "VelodyneSimu", (PyObject *)&VelodyneSimuType);

    return m;
}
