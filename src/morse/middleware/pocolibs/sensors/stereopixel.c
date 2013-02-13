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
#include <stereopixel/stereopixelClient.h>
#include <posterLib.h>

#define BLENDER_HORIZONTAL_APERTURE 32.0

typedef struct {
    PyObject_HEAD
	size_t height, width;
	POSTER_ID id;
} Stereopixel;

typedef struct {
	PyObject_HEAD
	size_t width, height;
	int pom_tag;
	double x_rob, y_rob, z_rob, yaw_rob, pitch_rob, roll_rob;
	double x_cam, y_cam, z_cam, yaw_cam, pitch_cam, roll_cam;
} StereopixelSimu;

	

static int
Stereopixel_init(Stereopixel *self, PyObject *args, PyObject *kwds)
{
    PyObject *images;
	char* poster_name;

    if (!PyArg_ParseTuple(args, "snn", &poster_name, &(self->width), &(self->height)))
		return -1;

	size_t poster_size = sizeof(Spix3DImage) + 
		self->width * self->height * sizeof(Spix3DPixel);

	STATUS s = posterCreate (poster_name, poster_size, &self->id);
	if (s == ERROR)
	{
		char buf[1024];
		h2getErrMsg(errnoGet(), buf, sizeof(buf));
		printf ("Unable to create the %s poster : %s\n",poster_name, buf);
		return -1;
	}

	printf("Succesfully created poster %s of size %zd\n", poster_name, poster_size); 

	Spix3DImage* img = posterAddr(self->id);

	posterTake(self->id, POSTER_WRITE);

	img->width = self->width;
	img->height = self->height;

	// dummy matrix
	memset(img->intrinseque_rectified, 0, sizeof(img->intrinseque_rectified));

	posterGive(self->id);

    return 0;
}


static PyObject*
Stereopixel_post(Stereopixel* self, PyObject* args)
{
	StereopixelSimu* info;
	PyObject* data;
	Py_buffer buf;
	size_t i;

    if (!PyArg_ParseTuple(args, "OO", &info, &data))
        return NULL;

	assert(info->height == self->height);
	assert(info->width ==  self->width);


	Spix3DImage* im3d  = posterAddr(self->id);
	Spix3DPixel* pixels = im3d->pixel;

	posterTake(self->id, POSTER_WRITE);

	POM_EULER* local_stm_euler = & (im3d->imageLeftPos.sensorToMain.euler);
	local_stm_euler->yaw = info->yaw_cam;
	local_stm_euler->pitch = info->pitch_cam;
	local_stm_euler->roll = info->roll_cam;

	local_stm_euler->x = info->x_cam;
	local_stm_euler->y = info->y_cam;
	local_stm_euler->z = info->z_cam;

	// Fill in the Main to Origin
	POM_EULER* local_mto_euler = &(im3d->imageLeftPos.mainToOrigin.euler);
	local_mto_euler->yaw = info->yaw_rob;
	local_mto_euler->pitch = info->pitch_rob;
	local_mto_euler->roll = info->roll_rob;

	local_mto_euler->x = info->x_rob;
	local_mto_euler->y = info->y_rob;
	local_mto_euler->z = info->z_rob;

	memcpy( &im3d->imageLeftPos.mainToBase.euler, 
			&im3d->imageLeftPos.mainToOrigin.euler, sizeof(POM_EULER));

	im3d->imageLeftPos.date = info->pom_tag;
	im3d->imageTicks = info->pom_tag;

	PyObject_GetBuffer(data, &buf, PyBUF_SIMPLE);
	memcpy(&im3d->imageRightPos, &im3d->imageLeftPos, sizeof(POM_SENSOR_POS));

	float* p = (float*)buf.buf;

	for (i = 0; i < self->width * self->height; i++) 
	{
		pixels->x = *p;
		pixels->y = *(p+1);
		pixels->z = *(p+2);
		if (pixels->x == 0.0 &&
			pixels->y == 0.0 &&
			pixels->z == 0.0)

			pixels->good = 0;
		else
			pixels->good = 1;

		pixels++;
		p += 3;
	}

	PyBuffer_Release(&buf);
	posterGive(self->id);

	Py_RETURN_NONE;
}

static PyObject*
Stereopixel_finalize(Stereopixel* self)
{
	posterDelete(self->id);

	Py_RETURN_NONE;
}


static PyMethodDef Stereopixel_methods[] = {
    {"post", (PyCFunction)Stereopixel_post, METH_VARARGS,
     "Post an image in the poster"
    },
	{"finalize", (PyCFunction)Stereopixel_finalize, METH_NOARGS,
	 "Cleanup ressource for stereopixel poster"},
    {NULL}  /* Sentinel */
};

static PyTypeObject StereopixelType = {
    PyVarObject_HEAD_INIT(NULL, 0)
    "stereopixel.Stereopixel",             /* tp_name */
    sizeof(Stereopixel),             /* tp_basicsize */
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
    "Stereopixel wrapper objects",     /* tp_doc */
    0,                         /* tp_traverse */
    0,                         /* tp_clear */
    0,                         /* tp_richcompare */
    0,                         /* tp_weaklistoffset */
    0,                         /* tp_iter */
    0,                         /* tp_iternext */
    Stereopixel_methods,             /* tp_methods */
    0,				            /* tp_members */
    0,                         /* tp_getset */
    0,                         /* tp_base */
    0,                         /* tp_dict */
    0,                         /* tp_descr_get */
    0,                         /* tp_descr_set */
    0,                         /* tp_dictoffset */
    (initproc)Stereopixel_init,      /* tp_init */
    0,                         /* tp_alloc */
    0,                 /* tp_new */
};

static PyMemberDef StereopixelSimu_members[] = {
	{"x_rob", T_DOUBLE, offsetof(StereopixelSimu, x_rob), 0, "x_rob"},
	{"y_rob", T_DOUBLE, offsetof(StereopixelSimu, y_rob), 0, "y_rob"},
	{"z_rob", T_DOUBLE, offsetof(StereopixelSimu, z_rob), 0, "z_rob"},
	{"yaw_rob", T_DOUBLE, offsetof(StereopixelSimu, yaw_rob), 0, "yaw_rob"},
	{"pitch_rob", T_DOUBLE, offsetof(StereopixelSimu, pitch_rob), 0, "pitch_rob"},
	{"roll_rob", T_DOUBLE, offsetof(StereopixelSimu, roll_rob), 0, "roll_rob"},
	{"x_cam", T_DOUBLE, offsetof(StereopixelSimu, x_cam), 0, "x_cam"},
	{"y_cam", T_DOUBLE, offsetof(StereopixelSimu, y_cam), 0, "y_cam"},
	{"z_cam", T_DOUBLE, offsetof(StereopixelSimu, z_cam), 0, "z_cam"},
	{"yaw_cam", T_DOUBLE, offsetof(StereopixelSimu, yaw_cam), 0, "yaw_cam"},
	{"pitch_cam", T_DOUBLE, offsetof(StereopixelSimu, pitch_cam), 0, "pitch_cam"},
	{"roll_cam", T_DOUBLE, offsetof(StereopixelSimu, roll_cam), 0, "roll_cam"},
	{"width", T_PYSSIZET, offsetof(StereopixelSimu, width), 0, "width"},
	{"height", T_PYSSIZET, offsetof(StereopixelSimu, height), 0, "height"},
	{"pom_tag", T_INT, offsetof(StereopixelSimu, pom_tag), 0, "pom_tag"},
    {NULL}  /* Sentinel */
};

static PyMethodDef StereopixelSimu_methods[] = {
    {NULL}  /* Sentinel */
};

static int
StereopixelSimu_init(StereopixelSimu *self, PyObject *args, PyObject *kwds)
{
	return 0;
}

static PyTypeObject StereopixelSimuType = {
    PyVarObject_HEAD_INIT(NULL, 0)
    "stereopixel.StereopixelSimu",      /* tp_name */
    sizeof(StereopixelSimu),     /* tp_basicsize */
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
    "Stereopixel simu image wrapper objects",    /* tp_doc */
    0,                         /* tp_traverse */
    0,                         /* tp_clear */
    0,                         /* tp_richcompare */
    0,                         /* tp_weaklistoffset */
    0,                         /* tp_iter */
    0,                         /* tp_iternext */
    StereopixelSimu_methods,             /* tp_methods */
    StereopixelSimu_members,             /* tp_members */
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



static PyModuleDef stereopixelmodule = {
    PyModuleDef_HEAD_INIT,
    "stereopixel",
    "Example module that creates an extension type.",
    -1,
    NULL, NULL, NULL, NULL, NULL
};

PyMODINIT_FUNC
PyInit_stereopixel(void)
{
    PyObject* m;

	StereopixelType.tp_new = PyType_GenericNew;
	StereopixelSimuType.tp_new = PyType_GenericNew;

    if (PyType_Ready(&StereopixelType) < 0)
        return NULL;
	if (PyType_Ready(&StereopixelSimuType) < 0)
		return NULL;

    m = PyModule_Create(&stereopixelmodule);
    if (m == NULL)
        return NULL;

    Py_INCREF(&StereopixelType);
    PyModule_AddObject(m, "Stereopixel", (PyObject *)&StereopixelType);

	Py_INCREF(&StereopixelSimuType);
    PyModule_AddObject(m, "StereopixelSimu", (PyObject *)&StereopixelSimuType);

    return m;
}
