#include <Python.h>
#include "structmember.h"

// http://docs.python.org/3/extending/newtypes.html

typedef struct {
    PyObject_HEAD
    float near;
    float far;
    int width;
    int height;
    // The buffer to return values
    float * points;
    int points_size;
} PyZBufferToDepth;

static void
ZBufferToDepth_dealloc(PyZBufferToDepth* self)
{
    free(self->points);
    Py_TYPE(self)->tp_free((PyObject*)self);
}

static PyObject *
ZBufferToDepth_new(PyTypeObject *type, PyObject *args, PyObject *kwds)
{
    PyZBufferToDepth *self;

    self = (PyZBufferToDepth *)type->tp_alloc(type, 0);
    if (self != NULL) {
        self->near = 1.0;
        self->far = 20.0;
        self->width = 0;
        self->height = 0;
        // The buffer to return values
        self->points = NULL;
        self->points_size = 0;
    }

    return (PyObject *)self;
}

static int
ZBufferToDepth_init(PyZBufferToDepth* self, PyObject* args)
{
    // Get the data as a Python object
    if (!PyArg_ParseTuple(args, "ffII", &self->near, &self->far,
                          &self->width, &self->height))
    {
        printf("Error while parsing ZBufferToDepth parameters\n");
        return -1;
    }

    // Allocate the buffer to store the points
    self->points_size = self->width * self->height * sizeof(float);
    self->points = malloc(self->points_size);

    return 0;
}


// Here is where the real job is done
static PyObject*
ZBufferToDepth_recover(PyZBufferToDepth* self, PyObject* args)
{
    Py_buffer img_buffer;

    int n_pixels, pixel;
    double z_n;
    float z_b;
    float test;
    float * fbuffer;

    // Read the incomming data as a buffer. It is originally a bgl.Buffer object
    if (!PyArg_ParseTuple(args, "w*", &img_buffer))
        return NULL;

    fbuffer = (float *) img_buffer.buf;
    n_pixels = img_buffer.len / sizeof(float);

    for (pixel=0; pixel<n_pixels; pixel++)
    {
        z_b = fbuffer[pixel];

        z_n = 2.0 * z_b - 1.0;
        // Store the z coordinate in the buffer
        self->points[pixel] = 2.0 * self->near * self->far / (self->far + self->near - z_n * (self->far - self->near));
    }

    // release the Python buffers
    PyBuffer_Release(&img_buffer);

    return PyMemoryView_FromMemory((char *)self->points, self->points_size, PyBUF_READ);
}



static PyMemberDef ZBufferToDepth_members[] = {
    {"near", T_FLOAT, offsetof(PyZBufferToDepth, near), 0,
     "PyZBufferToDepth near"},
    {"far", T_FLOAT, offsetof(PyZBufferToDepth, far), 0,
     "PyZBufferToDepth far"},
    {"width", T_INT, offsetof(PyZBufferToDepth, width), 0,
     "PyZBufferToDepth width"},
    {"height", T_INT, offsetof(PyZBufferToDepth, height), 0,
     "PyZBufferToDepth height"},
    {"points", T_OBJECT_EX, offsetof(PyZBufferToDepth, points), 0,
     "PyZBufferToDepth points"},
    {"points_size", T_INT, offsetof(PyZBufferToDepth, points_size), 0,
     "PyZBufferToDepth points_size"},
    {NULL}  /* Sentinel */
};

static PyMethodDef ZBufferToDepth_methods[] = {
    {"recover", (PyCFunction)ZBufferToDepth_recover, METH_VARARGS,
     "Convert the depth pixels of a Z-Buffer image in meters"
    },
    {NULL}  /* Sentinel */
};

static PyTypeObject ZBufferToDepthType = {
    PyVarObject_HEAD_INIT(NULL, 0)
    "zbuffertodepth.ZBufferToDepth", /* tp_name */
    sizeof(PyZBufferToDepth),     /* tp_basicsize */
    0,                         /* tp_itemsize */
    (destructor)ZBufferToDepth_dealloc, /* tp_dealloc */
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
    "ZBufferToDepth objects",     /* tp_doc */
    0,                         /* tp_traverse */
    0,                         /* tp_clear */
    0,                         /* tp_richcompare */
    0,                         /* tp_weaklistoffset */
    0,                         /* tp_iter */
    0,                         /* tp_iternext */
    ZBufferToDepth_methods,       /* tp_methods */
    ZBufferToDepth_members,       /* tp_members */
    0,                         /* tp_getset */
    0,                         /* tp_base */
    0,                         /* tp_dict */
    0,                         /* tp_descr_get */
    0,                         /* tp_descr_set */
    0,                         /* tp_dictoffset */
    (initproc)ZBufferToDepth_init,/* tp_init */
    0,                         /* tp_alloc */
    ZBufferToDepth_new,           /* tp_new */
};

static PyModuleDef zbuffertodepth_module = {
    PyModuleDef_HEAD_INIT,
    "zbuffertodepth",
    "zbuffertodepth module wrap ZBufferToDepth",
    -1,
    NULL, NULL, NULL, NULL, NULL
};

PyMODINIT_FUNC
PyInit_zbuffertodepth(void)
{
    PyObject* m;

    if (PyType_Ready(&ZBufferToDepthType) < 0)
        return NULL;

    m = PyModule_Create(&zbuffertodepth_module);
    if (m == NULL)
        return NULL;

    Py_INCREF(&ZBufferToDepthType);
    PyModule_AddObject(m, "ZBufferToDepth", (PyObject *)&ZBufferToDepthType);
    return m;
}
