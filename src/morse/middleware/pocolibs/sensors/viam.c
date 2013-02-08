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
#include <viam/viamStruct.h>
#include <viam/viamtypes.h>
#include <posterLib.h>

#define BLENDER_HORIZONTAL_APERTURE 32.0

struct img {
	const char* name;
	double focal;
	size_t width, height;
	char * data;
};

struct internal_args {
	POSTER_ID id;
	size_t nb_images;
	struct img* imgs;
};

typedef struct {
    PyObject_HEAD
	struct internal_args args;
} Viam;

typedef struct {
	PyObject_HEAD
	double x, y, z, yaw, pitch, roll;
} ViamPos;

typedef struct {
	PyObject_HEAD
	size_t width, height;
	int pom_tag;
	double x, y, z, yaw, pitch, roll;
	unsigned long tacq_sec, tacq_usec;
	bool flipped;
} ViamSimuImage;

	
void
create_bank_calibration(viam_bankcalibration_t* bank_calib, size_t nb_images, 
		double baseline, double pixel_size)
{
	switch (nb_images) {
		case 1:
			bank_calib->type = VIAM_CAL_MONO;
			break;
		case 2:
			bank_calib->type = VIAM_CAL_STEREO;
			bank_calib->baseline = baseline;
			// probably unused
			bank_calib->pbaseline = baseline * pixel_size ;
			break;
		default:
			assert(false);
	}
}
/*
 * Build camera calibration, it must be completly revisited (in particular,
 * don't harcode size in it
 */
void
create_camera_calibration(viam_cameracalibration_t* local_calibration,
	   	const struct img* init)
{
	double identity[9] = {1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0 };
	double alpha_u = init->width * init->focal / BLENDER_HORIZONTAL_APERTURE;
	size_t i;

	memcpy(local_calibration->intrinsic, identity, sizeof(identity));
	memcpy(local_calibration->rectification, identity, sizeof(identity));
	memcpy(local_calibration->rotation, identity, sizeof(identity));

	local_calibration->intrinsic[2] = init->width / 2.0;
	local_calibration->intrinsic[5] = init->height / 2.0;
	local_calibration->intrinsic[0] = alpha_u;
	local_calibration->intrinsic[4] = alpha_u;

	memcpy(local_calibration->intrirect, local_calibration->intrinsic, sizeof(identity));

	for (i = 0; i < 5; i++)
		local_calibration->distortion[i] = 0.0;


	local_calibration->width = init->width;
	local_calibration->height = init->height;
}

static void 
fill_static_data(size_t i, size_t nb_images, ViamImageHeader* header, 
				 const struct img* init,
				 size_t* offset)
{
	strncpy ( header->name.id, init->name, VIAM_ID_MAX);
	header->name.id[VIAM_ID_MAX - 1]= '\0';

	create_camera_calibration(& header->calibration, init);

	header->nChannels = 1;
	header->depth = 8;
	header->width = init->width;
	header->height = init->height;
	header->widthStep = init->width * header->depth / 8 * header->nChannels; 
	header->imageSize = init->height * header->widthStep;
	header->dataOffset = 
		(nb_images - (i + 1)) * sizeof(ViamImageHeader) + *offset;
	*offset+= init->height * init->width;
}
	

static int
Viam_init(Viam *self, PyObject *args, PyObject *kwds)
{
	char* poster_name;
	char* bank_name;
	double baseline;
    PyObject *images;

    static char *kwlist[] = {"poster_name", "bank_name", "baseline", "images_spec", NULL};

    if (! PyArg_ParseTupleAndKeywords(args, kwds, "ssdO", kwlist,
                                      &poster_name, &bank_name, &baseline, &images))
        return -1;

	self->args.nb_images = PyList_Size(images);
	if (self->args.nb_images == 0)
		return -1;

	self->args.imgs = malloc(self->args.nb_images * sizeof(struct img));
	if (self->args.imgs == NULL)
		return -1;

	size_t poster_size = sizeof(ViamImageBank);
	size_t i;

	for (i = 0; i < self->args.nb_images; i++) {
		struct img* c_img = & self->args.imgs[i];
		const char* name;

		PyObject* o = PyList_GetItem(images, i);
		PyObject* p = PyDict_GetItemString(o, "name");
		name = PyBytes_AsString(PyUnicode_AsUTF8String(p));
		c_img->name = strdup(name);
		c_img->focal = PyFloat_AsDouble(PyDict_GetItemString(o, "focal"));
		c_img->width = PyLong_AsLongLong(PyDict_GetItemString(o, "width"));
		c_img->height = PyLong_AsLongLong(PyDict_GetItemString(o, "height"));

		poster_size+= sizeof(ViamImageHeader) + c_img->width * c_img->height;
	}

	STATUS s = posterCreate (poster_name, poster_size, &self->args.id);
	if (s == ERROR)
	{
		char buf[1024];
		h2getErrMsg(errnoGet(), buf, sizeof(buf));
		printf ("Unable to create the %s poster : %s\n",poster_name, buf);
		free(self->args.imgs);
		return -1;
	}

	printf("Succesfully created poster %s of size %zd\n", poster_name, poster_size); 

	ViamImageBank* bank  = posterAddr(self->args.id);

	posterTake(self->args.id, POSTER_WRITE);

	// Fill the poster with information that don't change
	strncpy ( bank->name.id, bank_name, VIAM_ID_MAX);
	bank->name.id[VIAM_ID_MAX - 1]= '\0';

	double alpha_u = self->args.imgs[0].width * self->args.imgs[0].focal / BLENDER_HORIZONTAL_APERTURE;
	create_bank_calibration(&bank->calibration, self->args.nb_images, baseline, alpha_u);
	bank->nImages = self->args.nb_images;

	size_t offset = 0;
	for (i = 0; i < self->args.nb_images; i++) 
		fill_static_data(i, self->args.nb_images, &bank->image[i], &self->args.imgs[i], &offset);

	posterGive(self->args.id);

    return 0;
}

/*
 * Fill a POM_SENSOR_POS on the base of information returned by the simulator
 * It is correct for the moment, but it is probably better to do it "in-place"
 */
int
create_pom_sensor_pos( int blender_date, 
					   POM_SENSOR_POS* pos,			 
					   const ViamPos* robot, 
					   const ViamSimuImage* sensor)
{
	// Fill in the Sensor to Main
	POM_EULER* local_stm_euler = & (pos->sensorToMain.euler);
	local_stm_euler->yaw = sensor->yaw;
	local_stm_euler->pitch = sensor->pitch;
	local_stm_euler->roll = sensor->roll;

	local_stm_euler->x = sensor->x;
	local_stm_euler->y = sensor->y;
	local_stm_euler->z = sensor->z;


	// Fill in the Main to Origin
	POM_EULER* local_mto_euler = &(pos->mainToOrigin.euler);
	local_mto_euler->yaw = robot->yaw;
	local_mto_euler->pitch = robot->pitch;
	local_mto_euler->roll = robot->roll;

	local_mto_euler->x = robot->x;
	local_mto_euler->y = robot->y;
	local_mto_euler->z = robot->z;

	memcpy( &pos->mainToBase.euler, &pos->mainToOrigin.euler, sizeof(POM_EULER));

	pos->date = blender_date;
	pos->pad = 0;

	return 0;
}

/*
 * Fill one image with the information computed by the simulator
 * Information in blender are stored in column major, we assume line major,
 * hence the transformation.
 *
 * We are filling too date tag, and position tag in this function
 */
static int
flip_and_fill_image(ViamImageHeader* image, const ViamPos* robot, 
								   const ViamSimuImage* img,
                                   char* image_data)
{
	assert(image->height == img->height);
	assert(image->width == img->width);

	image->tacq_sec = img->tacq_sec;
	image->tacq_usec = img->tacq_usec;
	create_pom_sensor_pos(img->pom_tag, &image->pos, robot, img);


	unsigned char* data = & image->data[image->dataOffset];
    size_t len = img->width * img->height;
	size_t i, j;
	for (j = 0 ; j < img->height; j++)
		for (i = 0 ; i <  img->width; i++)
	    {
            size_t index = (j * img->width + i) * 4;
            size_t index_ = ((img->height - 1 - j) * img->width  + i);

	    	unsigned char r = (unsigned char) image_data[index];
	    	unsigned char g = (unsigned char) image_data[index+1];
	    	unsigned char b = (unsigned char) image_data[index+2];

	    	// RGB[A] -> GREY
	    	data[index_] = 0.299*r + 0.587*g + 0.114*b;
	    }

	return 0;
}

static int
fill_image(ViamImageHeader* image, const ViamPos* robot, 
								   const ViamSimuImage* img,
                                   char* image_data)
{
	assert(image->height == img->height);
	assert(image->width == img->width);

	image->tacq_sec = img->tacq_sec;
	image->tacq_usec = img->tacq_usec;
	create_pom_sensor_pos(img->pom_tag, &image->pos, robot, img);


	unsigned char* data = & image->data[image->dataOffset];
    size_t len = img->width * img->height * 4;
	const char* p;
	
	for (p = image_data; p != image_data + len; p+= 4) {
		unsigned char r = (unsigned char) *p;
		unsigned char g = (unsigned char) *(p + 1);
		unsigned char b = (unsigned char) *(p + 2);

		*data++ = 0.299*r + 0.587*g + 0.114*b;
	}

	return 0;
}

static PyObject*
Viam_post(Viam* self, PyObject* args)
{
	PyObject* specs, *datas;
	PyObject* spec, *data;
	ViamPos* pos;
	size_t i;

    if (!PyArg_ParseTuple(args, "OOO", &pos, &specs, &datas))
        return NULL;

	size_t len1 = PyList_Size(specs);
	size_t len2 = PyList_Size(datas);
	assert(len1 == len2);
	assert(len1 == self->args.nb_images);

	ViamImageBank* bank  = posterAddr(self->args.id);
	posterTake(self->args.id, POSTER_WRITE);

	for (i = 0; i < self->args.nb_images; i++) {
		ViamSimuImage* img = (ViamSimuImage*) PyList_GetItem(specs, i);
		data = PyList_GetItem(datas, i);
		Py_buffer buf;
		int err = PyObject_GetBuffer(data, &buf, PyBUF_SIMPLE);
		assert (err == 0);
		if (!img->flipped)
			flip_and_fill_image(&bank->image[i], pos, img, buf.buf);
		else
			fill_image(&bank->image[i], pos, img, buf.buf);

		PyBuffer_Release(&buf);
	}

	posterGive(self->args.id);

	Py_RETURN_NONE;
}

static PyObject*
Viam_finalize(Viam* self)
{
	free(self->args.imgs);
	self->args.imgs = NULL;
	posterDelete(self->args.id);

	Py_RETURN_NONE;
}

static PyMemberDef Viam_members[] = {
/*
    {"first", T_OBJECT_EX, offsetof(Viam, first), 0,
     "first name"},
    {"last", T_OBJECT_EX, offsetof(Viam, last), 0,
     "last name"},
    {"number", T_INT, offsetof(Viam, number), 0,
     "noddy number"},
*/
    {NULL}  /* Sentinel */
};


static PyMethodDef Viam_methods[] = {
    {"post", (PyCFunction)Viam_post, METH_VARARGS,
     "Post an image in the poster"
    },
	{"finalize", (PyCFunction)Viam_finalize, METH_NOARGS,
	 "Cleanup ressource for viam poster"},
    {NULL}  /* Sentinel */
};

static PyTypeObject ViamType = {
    PyVarObject_HEAD_INIT(NULL, 0)
    "viam.Viam",             /* tp_name */
    sizeof(Viam),             /* tp_basicsize */
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
    "Viam wrapper objects",     /* tp_doc */
    0,                         /* tp_traverse */
    0,                         /* tp_clear */
    0,                         /* tp_richcompare */
    0,                         /* tp_weaklistoffset */
    0,                         /* tp_iter */
    0,                         /* tp_iternext */
    Viam_methods,             /* tp_methods */
    Viam_members,             /* tp_members */
    0,                         /* tp_getset */
    0,                         /* tp_base */
    0,                         /* tp_dict */
    0,                         /* tp_descr_get */
    0,                         /* tp_descr_set */
    0,                         /* tp_dictoffset */
    (initproc)Viam_init,      /* tp_init */
    0,                         /* tp_alloc */
    0,                 /* tp_new */
};

static PyMemberDef ViamSimuImage_members[] = {
	{"x", T_DOUBLE, offsetof(ViamSimuImage, x), 0, "x"},
	{"y", T_DOUBLE, offsetof(ViamSimuImage, y), 0, "y"},
	{"z", T_DOUBLE, offsetof(ViamSimuImage, z), 0, "z"},
	{"yaw", T_DOUBLE, offsetof(ViamSimuImage, yaw), 0, "yaw"},
	{"pitch", T_DOUBLE, offsetof(ViamSimuImage, pitch), 0, "pitch"},
	{"roll", T_DOUBLE, offsetof(ViamSimuImage, roll), 0, "roll"},
	{"width", T_PYSSIZET, offsetof(ViamSimuImage, width), 0, "width"},
	{"height", T_PYSSIZET, offsetof(ViamSimuImage, height), 0, "height"},
	{"pom_tag", T_INT, offsetof(ViamSimuImage, pom_tag), 0, "pom_tag"},
	{"tacq_sec", T_ULONG, offsetof(ViamSimuImage, tacq_sec), 0, "tacq_sec"},
	{"tacq_usec", T_ULONG, offsetof(ViamSimuImage, tacq_usec), 0, "tacq_usec"},
	{"flipped", T_BOOL, offsetof(ViamSimuImage, flipped), 0, "flipped"},
    {NULL}  /* Sentinel */
};

static PyMethodDef ViamSimuImage_methods[] = {
    {NULL}  /* Sentinel */
};

static int
ViamSimuImage_init(ViamSimuImage *self, PyObject *args, PyObject *kwds)
{
	return 0;
}

static PyTypeObject ViamSimuImageType = {
    PyVarObject_HEAD_INIT(NULL, 0)
    "viam.ViamSimuImage",      /* tp_name */
    sizeof(ViamSimuImage),     /* tp_basicsize */
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
    "Viam simu image wrapper objects",    /* tp_doc */
    0,                         /* tp_traverse */
    0,                         /* tp_clear */
    0,                         /* tp_richcompare */
    0,                         /* tp_weaklistoffset */
    0,                         /* tp_iter */
    0,                         /* tp_iternext */
    ViamSimuImage_methods,             /* tp_methods */
    ViamSimuImage_members,             /* tp_members */
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

static PyMemberDef ViamPos_members[] = {
	{"x", T_DOUBLE, offsetof(ViamPos, x), 0, "x"},
	{"y", T_DOUBLE, offsetof(ViamPos, y), 0, "y"},
	{"z", T_DOUBLE, offsetof(ViamPos, z), 0, "z"},
	{"yaw", T_DOUBLE, offsetof(ViamPos, yaw), 0, "yaw"},
	{"pitch", T_DOUBLE, offsetof(ViamPos, pitch), 0, "pitch"},
	{"roll", T_DOUBLE, offsetof(ViamPos, roll), 0, "roll"},
    {NULL}  /* Sentinel */
};


static PyTypeObject ViamPosType = {
    PyVarObject_HEAD_INIT(NULL, 0)
    "viam.ViamPos",      /* tp_name */
    sizeof(ViamPos),     /* tp_basicsize */
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
    "Viam pos wrapper objects",    /* tp_doc */
    0,                         /* tp_traverse */
    0,                         /* tp_clear */
    0,                         /* tp_richcompare */
    0,                         /* tp_weaklistoffset */
    0,                         /* tp_iter */
    0,                         /* tp_iternext */
    0,				          /* tp_methods */
    ViamPos_members,             /* tp_members */
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


static PyModuleDef viammodule = {
    PyModuleDef_HEAD_INIT,
    "viam",
    "Example module that creates an extension type.",
    -1,
    NULL, NULL, NULL, NULL, NULL
};

PyMODINIT_FUNC
PyInit_viam(void)
{
    PyObject* m;

	ViamType.tp_new = PyType_GenericNew;
	ViamSimuImageType.tp_new = PyType_GenericNew;
	ViamPosType.tp_new = PyType_GenericNew;

    if (PyType_Ready(&ViamType) < 0)
        return NULL;
	if (PyType_Ready(&ViamSimuImageType) < 0)
		return NULL;
	if (PyType_Ready(&ViamPosType) < 0)
		return NULL;

    m = PyModule_Create(&viammodule);
    if (m == NULL)
        return NULL;

    Py_INCREF(&ViamType);
    PyModule_AddObject(m, "Viam", (PyObject *)&ViamType);

	Py_INCREF(&ViamSimuImageType);
    PyModule_AddObject(m, "ViamSimuImage", (PyObject *)&ViamSimuImageType);

	Py_INCREF(&ViamPosType);
	PyModule_AddObject(m, "ViamPos", (PyObject*)&ViamPosType);
    return m;
}
