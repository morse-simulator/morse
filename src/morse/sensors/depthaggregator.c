#include <Python.h>
#include "structmember.h"


typedef struct {
    PyObject_HEAD
    // The buffer to return values
    float * points;
    int max_nb_points;
} PyAggregator;


static void
Aggregator_dealloc(PyAggregator* self)
{
    free(self->points);
    Py_TYPE(self)->tp_free((PyObject*)self);
}

static PyObject *
Aggregator_new(PyTypeObject *type, PyObject *args, PyObject *kwds)
{
    PyAggregator *self;

    self = (PyAggregator *)type->tp_alloc(type, 0);
    if (self != NULL) {
        // The buffer to return values
        self->points = NULL;
        self->max_nb_points = 0;
    }

    return (PyObject *)self;
}

static int
Aggregator_init(PyAggregator* self, PyObject* args)
{
    // Get the data as a Python object
    if (!PyArg_ParseTuple(args, "I", &self->max_nb_points))
    {
        printf("Error while parsing Aggregator parameters\n");
        return -1;
    }

    // Allocate the buffer to store the points
    self->points = malloc(self->max_nb_points * 3 * sizeof(float));

    return 0;
}

static PyObject* 
Aggregator_merge(PyAggregator* self, PyObject* args)
{
    PyObject * listObj;

    if (!PyArg_ParseTuple(args, "O!", &PyList_Type, &listObj))
        return NULL;

    int len = PyList_Size(listObj);
    int j = 0;

    struct timeval begin, end, res;
    gettimeofday(&begin, NULL);

    for (int i = 0; i < len; ++i)
    {
        float tx, ty, tz, rx, ry, rz;
        Py_buffer img_buffer;
        PyObject * obj = PyList_GetItem(listObj, i);
        PyObject * buf;
        if (!PyArg_ParseTuple(obj, "ffffffO", &tx, &ty, &tx, &rx, &ry, &rz, &buf))
            return NULL;

        PyObject_GetBuffer(buf, &img_buffer, PyBUF_SIMPLE);

        float cx = cos(rx);
        float cy = cos(ry);
        float cz = cos(rz);
        float sx = sin(rx);
        float sy = sin(ry);
        float sz = sin(rz);

        float* fbuffer = (float *) img_buffer.buf;
        int nb_points = img_buffer.len / (3 * sizeof(float));

        for (int k = 0; k < nb_points && j < 3*self->max_nb_points; ++k)
        {
            float x = *fbuffer++;
            float y = *fbuffer++;
            float z = *fbuffer++;

            self->points[j] = x * (cz * cy) + y * (sz * cx - cz * sy * sx) + z * (sz * sx + cz * sy * cx);
            self->points[j+1] = x * (-sz * cy) + y * (cz * cx + sz * sy * sx) + z * (cz * sx + sz * sy * cx);
            self->points[j+2] = x * -sy + y * (-cy * sx) + z * (cy * cx);

            j += 3;
        }
    }

    return PyMemoryView_FromMemory((char *)self->points, j*sizeof(float), PyBUF_READ);
}


static PyMemberDef Aggregator_members[] = {
    {"points", T_OBJECT_EX, offsetof(PyAggregator, points), 0,
     "PyAggregator points"},
    {"max_nb_points", T_INT, offsetof(PyAggregator, max_nb_points), 0,
     "PyAggregator max_nb_points"},
    {NULL}  /* Sentinel */
};

static PyMethodDef Aggregator_methods[] = {
    {"merge", (PyCFunction)Aggregator_merge, METH_VARARGS,
     "Convert a Z-Buffer image into an array of 3D points"
    },
    {NULL}  /* Sentinel */
};

static PyTypeObject AggregatorType = {
    PyVarObject_HEAD_INIT(NULL, 0)
    "depthaggregator.Aggregator", /* tp_name */
    sizeof(PyAggregator),     /* tp_basicsize */
    0,                         /* tp_itemsize */
    (destructor)Aggregator_dealloc, /* tp_dealloc */
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
    "Aggregator objects",     /* tp_doc */
    0,                         /* tp_traverse */
    0,                         /* tp_clear */
    0,                         /* tp_richcompare */
    0,                         /* tp_weaklistoffset */
    0,                         /* tp_iter */
    0,                         /* tp_iternext */
    Aggregator_methods,       /* tp_methods */
    Aggregator_members,       /* tp_members */
    0,                         /* tp_getset */
    0,                         /* tp_base */
    0,                         /* tp_dict */
    0,                         /* tp_descr_get */
    0,                         /* tp_descr_set */
    0,                         /* tp_dictoffset */
    (initproc)Aggregator_init,/* tp_init */
    0,                         /* tp_alloc */
    Aggregator_new,           /* tp_new */
};

static PyModuleDef depthaggregator_module = {
    PyModuleDef_HEAD_INIT,
    "depthaggregator",
    "zbufferto3d module wrap Aggregator",
    -1,
    NULL, NULL, NULL, NULL, NULL
};

PyMODINIT_FUNC
PyInit_depthaggregator(void)
{
    PyObject* m;

    if (PyType_Ready(&AggregatorType) < 0)
        return NULL;

    m = PyModule_Create(&depthaggregator_module);
    if (m == NULL)
        return NULL;

    Py_INCREF(&AggregatorType);
    PyModule_AddObject(m, "Aggregator", (PyObject *)&AggregatorType);
    return m;
}
