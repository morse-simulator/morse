#include <Python.h>
#include "structmember.h"
#include "GeomagnetismHeader.h"
#include "EGM9615.h"

typedef struct {
    PyObject_HEAD
    MAGtype_MagneticModel * MagneticModels[1];
	MAGtype_MagneticModel * TimedMagneticModel;
    MAGtype_Geoid Geoid;
    MAGtype_Ellipsoid Ellip;
} PyMagnetometer;

static void
Magnetometer_dealloc(PyMagnetometer* self)
{
    MAG_FreeMagneticModelMemory(self->TimedMagneticModel);
    MAG_FreeMagneticModelMemory(self->MagneticModels[0]);
    Py_TYPE(self)->tp_free((PyObject*)self);
}

static PyObject *
Magnetometer_new(PyTypeObject *type, PyObject *args, PyObject *kwds)
{
    PyMagnetometer *self;

    self = (PyMagnetometer *)type->tp_alloc(type, 0);
    if (self != NULL) {
    }

    return (PyObject *)self;
}

static int
Magnetometer_init(PyMagnetometer* self, PyObject* args)
{
    // Get the data as a Python object
	char* filename;
    if (!PyArg_ParseTuple(args, "s", &filename))
    {
        printf("Error while parsing Magnetometer parameters\n");
        return -1;
    }

    printf("Got %s\n", filename);

    if(!MAG_robustReadMagModels(filename, &self->MagneticModels, 1)) { return -1;}
    int nMax = self->MagneticModels[0]->nMax;
    int NumTerms = ((nMax + 1) * (nMax + 2) / 2);
    self->TimedMagneticModel = MAG_AllocateModelMemory(NumTerms); /* For storing the time modified WMM Model parameters */
    if(self->MagneticModels[0] == NULL || self->TimedMagneticModel == NULL)
		return -1;
    MAG_SetDefaults(&self->Ellip, &self->Geoid); /* Set default values and constants */

    /* Set EGM96 Geoid parameters */
    self->Geoid.GeoidHeightBuffer = GeoidHeightBuffer;
    self->Geoid.Geoid_Initialized = 1;
    /* Set EGM96 Geoid parameters END */

    return 0;
}

static PyObject*
Magnetometer_compute(PyMagnetometer* self, PyObject* args)
{
	double longitude, latitude, h, decimal_year;
    if (!PyArg_ParseTuple(args, "dddd", &longitude, &latitude, &h, &decimal_year))
    {
        printf("Error while parsing Magnetometer parameters\n");
        return NULL;
    }

    MAGtype_CoordGeodetic CoordGeodetic;
	CoordGeodetic.lambda = longitude;
	CoordGeodetic.phi = latitude;
	CoordGeodetic.HeightAboveEllipsoid = h;

    MAGtype_Date UserDate;
	UserDate.DecimalYear = decimal_year;

    MAGtype_GeoMagneticElements GeoMagneticElements, Errors;
    MAGtype_CoordSpherical CoordSpherical;

	MAG_GeodeticToSpherical(self->Ellip, CoordGeodetic, &CoordSpherical); /*Convert from geodetic to Spherical Equations: 17-18, WMM Technical report*/
	MAG_TimelyModifyMagneticModel(UserDate, self->MagneticModels[0], self->TimedMagneticModel); /* Time adjust the coefficients, Equation 19, WMM Technical report */
	MAG_Geomag(self->Ellip, CoordSpherical, CoordGeodetic, self->TimedMagneticModel, &GeoMagneticElements); /* Computes the geoMagnetic field elements and their time change*/
	return Py_BuildValue("(ddddddd)", GeoMagneticElements.Decl, GeoMagneticElements.Incl, 
						   GeoMagneticElements.F, GeoMagneticElements.H,
						   GeoMagneticElements.X, GeoMagneticElements.Y, GeoMagneticElements.Z);
}

static PyMethodDef Magnetometer_methods[] = {
    {"compute", (PyCFunction)Magnetometer_compute, METH_VARARGS,
     "Compute the magnetic field on a specific point"
    },
    {NULL}  /* Sentinel */
};

static PyTypeObject MagnetometerType = {
    PyVarObject_HEAD_INIT(NULL, 0)
    "magnetometer.Magnetometer", /* tp_name */
    sizeof(PyMagnetometer),     /* tp_basicsize */
    0,                         /* tp_itemsize */
    (destructor)Magnetometer_dealloc, /* tp_dealloc */
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
    Py_TPFLAGS_DEFAULT |
        Py_TPFLAGS_BASETYPE,   /* tp_flags */
    "Magnetometer objects",     /* tp_doc */
    0,                         /* tp_traverse */
    0,                         /* tp_clear */
    0,                         /* tp_richcompare */
    0,                         /* tp_weaklistoffset */
    0,                         /* tp_iter */
    0,                         /* tp_iternext */
    Magnetometer_methods,       /* tp_methods */
    0,							/* tp_members */
    0,                         /* tp_getset */
    0,                         /* tp_base */
    0,                         /* tp_dict */
    0,                         /* tp_descr_get */
    0,                         /* tp_descr_set */
    0,                         /* tp_dictoffset */
    (initproc)Magnetometer_init,/* tp_init */
    0,                         /* tp_alloc */
    Magnetometer_new,           /* tp_new */
};

static PyModuleDef _magnetometer_module = {
    PyModuleDef_HEAD_INIT,
    "_magnetometer",
    "magnetometer module wrap Magnetometer",
    -1,
    NULL, NULL, NULL, NULL, NULL
};

PyMODINIT_FUNC
PyInit__magnetometer(void)
{
    PyObject* m;

    if (PyType_Ready(&MagnetometerType) < 0)
        return NULL;

    m = PyModule_Create(&_magnetometer_module);
    if (m == NULL)
        return NULL;

    Py_INCREF(&MagnetometerType);
    PyModule_AddObject(m, "Magnetometer", (PyObject *)&MagnetometerType);
    return m;
}
