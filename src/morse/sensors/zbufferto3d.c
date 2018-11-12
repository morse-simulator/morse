#include <Python.h>
#include <stdbool.h>
#include "structmember.h"

// http://docs.python.org/3/extending/newtypes.html

typedef struct {
    PyObject_HEAD
    float alpha_u;
    float alpha_v;
    float near;
    float far;
    int width;
    int height;
    int u_0;
    int v_0;
    float* u_factor;
    float* v_factor;
    // the list of lines we want to process
    int * to_keep;
    int to_keep_size;
    // The buffer to return values
    float * points;
    int points_size;
    bool keep_resolution;
} PyZBufferTo3D;

static void
ZBufferTo3D_dealloc(PyZBufferTo3D* self)
{
    free(self->u_factor);
    free(self->v_factor);
    free(self->to_keep);
    free(self->points);
    Py_TYPE(self)->tp_free((PyObject*)self);
}

static PyObject *
ZBufferTo3D_new(PyTypeObject *type, PyObject *args, PyObject *kwds)
{
    PyZBufferTo3D *self;

    self = (PyZBufferTo3D *)type->tp_alloc(type, 0);
    if (self != NULL) {
        self->alpha_u = 0.0;
        self->alpha_v = 0.0;
        self->near = 1.0;
        self->far = 20.0;
        self->width = 0;
        self->height = 0;
        self->u_0;
        self->v_0;
        self->u_factor = NULL;
        self->v_factor = NULL;
        // keep part
        self->to_keep = NULL;
        self->to_keep_size = 0;
        // The buffer to return values
        self->points = NULL;
        self->points_size = 0;
        // Keep points (including over Far)
        self->keep_resolution = false;
    }

    return (PyObject *)self;
}

static int
ZBufferTo3D_init(PyZBufferTo3D* self, PyObject* args)
{
    PyObject * listObj;

    // Get the data as a Python object
    if (!PyArg_ParseTuple(args, "ffffIIbO!", &self->alpha_u, &self->alpha_v,
                          &self->near, &self->far, &self->width, &self->height,
                          &self->keep_resolution, &PyList_Type, &listObj))
    {
        printf("Error while parsing ZBufferTo3D parameters\n");
        return -1;
    }

    self->u_0 = self->width / 2;
    self->v_0 = self->height / 2;

    self->u_factor = malloc(self->width * sizeof(float));
    self->v_factor = malloc(self->height * sizeof(float));

    for (int u = 0; u < self->width; ++u)
        self->u_factor[u] = (float)(u - self->u_0) / self->alpha_u;
    for (int v = 0; v < self->height; ++v)
        self->v_factor[v] = (float)(v - self->v_0) / self->alpha_v;

    // Allocate the buffer to store the points
    self->points_size = self->width * self->height * 3 * sizeof(float);
    self->points = malloc(self->points_size);

    int len = PyList_Size(listObj);

    if (len > 0)
    {
        self->to_keep = malloc(sizeof(int) * len);
        self->to_keep_size = len;

        int* to_keep = self->to_keep;

        for (int i = 0; i < len; ++i, to_keep++)
        {
            *to_keep = PyLong_AsLong(PyList_GetItem(listObj, i));
        }
    }

    return 0;
}

// Here is where the real job is done
static PyObject*
ZBufferTo3D_recover(PyZBufferTo3D* self, PyObject* args)
{
    Py_buffer img_buffer;

    // Read the incomming data as a buffer. It is originally a bgl.Buffer object
    if (!PyArg_ParseTuple(args, "w*", &img_buffer))
        return NULL;

    // check that there is no division by 0
    if (self->width == 0)
        return NULL;

    int* to_keep = self->to_keep;
    int* to_keep_end = self->to_keep + self->to_keep_size;

    int valid_points = 0;
    // The image we receive is stored as a single array of floats.
    // Pixel 0, 0 in the data is located at the bottom left, according to
    // the OpenGL conventions.
    // We need to convert this frame of reference to (u, v), starting at the
    // top left
    // We process the image line by line.

    float A = 2.0 * self->near * self->far;
    float B = self->far + self->near;
    float C = self->far - self->near;

    float* points = self->points;

    while (to_keep < to_keep_end)
    {
        int v = self->height - *to_keep;
        float v_factor = self->v_factor[v];
        float* u_factor = self->u_factor;
        float* buf = (float*)img_buffer.buf + *to_keep * self->width;

        for (int u = 0; u < self->width; ++u, ++buf, ++u_factor)
        {
            float x, y, z_e;
            float z_b = *buf;

            if (z_b >= 1.0)
            {
                if(!self->keep_resolution)
                    continue; // nothing seen within the far clipping

                // Add a dummy point at 0,0,0 in order to keep the sensor resolution in the PointCloud
                x = 0.;
                y = 0.;
                z_e = 0.;
            }else{
              float z_n = 2.0 * z_b - 1.0;
              z_e = A / (B - z_n * C);

              // Use the intrinsic matrix of the camera view to get the 3D coordinates
              // corresponding to each pixel, with respect to the camera
              x = z_e * *u_factor;
              y = z_e * v_factor;
            }

            // Store the x, y, z coordinates in the buffer
            *points = x; ++points;
            *points = y; ++points;
            *points = z_e; ++points;
            ++valid_points;
        }
        ++to_keep;
    }

    return PyMemoryView_FromMemory((char *)self->points, valid_points*3*sizeof(float), PyBUF_READ);
}



static PyMemberDef ZBufferTo3D_members[] = {
    {"alpha_u", T_FLOAT, offsetof(PyZBufferTo3D, alpha_u), 0,
     "PyZBufferTo3D alpha_u"},
    {"alpha_v", T_FLOAT, offsetof(PyZBufferTo3D, alpha_v), 0,
     "PyZBufferTo3D alpha_v"},
    {"near", T_FLOAT, offsetof(PyZBufferTo3D, near), 0,
     "PyZBufferTo3D near"},
    {"far", T_FLOAT, offsetof(PyZBufferTo3D, far), 0,
     "PyZBufferTo3D far"},
    {"width", T_INT, offsetof(PyZBufferTo3D, width), 0,
     "PyZBufferTo3D width"},
    {"height", T_INT, offsetof(PyZBufferTo3D, height), 0,
     "PyZBufferTo3D height"},
    {"u_0", T_INT, offsetof(PyZBufferTo3D, u_0), 0,
     "PyZBufferTo3D u_0"},
    {"v_0", T_INT, offsetof(PyZBufferTo3D, v_0), 0,
     "PyZBufferTo3D v_0"},
    {"points", T_OBJECT_EX, offsetof(PyZBufferTo3D, points), 0,
     "PyZBufferTo3D points"},
    {"points_size", T_INT, offsetof(PyZBufferTo3D, points_size), 0,
     "PyZBufferTo3D points_size"},
    {"keep_resolution", T_BOOL, offsetof(PyZBufferTo3D, keep_resolution), 0,
     "PyZBufferTo3D keep_resolution"},
    {NULL}  /* Sentinel */
};

static PyMethodDef ZBufferTo3D_methods[] = {
    {"recover", (PyCFunction)ZBufferTo3D_recover, METH_VARARGS,
     "Convert a Z-Buffer image into an array of 3D points"
    },
    {NULL}  /* Sentinel */
};

static PyTypeObject ZBufferTo3DType = {
    PyVarObject_HEAD_INIT(NULL, 0)
    "zbufferto3d.ZBufferTo3D", /* tp_name */
    sizeof(PyZBufferTo3D),     /* tp_basicsize */
    0,                         /* tp_itemsize */
    (destructor)ZBufferTo3D_dealloc, /* tp_dealloc */
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
    "ZBufferTo3D objects",     /* tp_doc */
    0,                         /* tp_traverse */
    0,                         /* tp_clear */
    0,                         /* tp_richcompare */
    0,                         /* tp_weaklistoffset */
    0,                         /* tp_iter */
    0,                         /* tp_iternext */
    ZBufferTo3D_methods,       /* tp_methods */
    ZBufferTo3D_members,       /* tp_members */
    0,                         /* tp_getset */
    0,                         /* tp_base */
    0,                         /* tp_dict */
    0,                         /* tp_descr_get */
    0,                         /* tp_descr_set */
    0,                         /* tp_dictoffset */
    (initproc)ZBufferTo3D_init,/* tp_init */
    0,                         /* tp_alloc */
    ZBufferTo3D_new,           /* tp_new */
};

static PyModuleDef zbufferto3d_module = {
    PyModuleDef_HEAD_INIT,
    "zbufferto3d",
    "zbufferto3d module wrap ZBufferTo3D",
    -1,
    NULL, NULL, NULL, NULL, NULL
};

PyMODINIT_FUNC
PyInit_zbufferto3d(void)
{
    PyObject* m;

    if (PyType_Ready(&ZBufferTo3DType) < 0)
        return NULL;

    m = PyModule_Create(&zbufferto3d_module);
    if (m == NULL)
        return NULL;

    Py_INCREF(&ZBufferTo3DType);
    PyModule_AddObject(m, "ZBufferTo3D", (PyObject *)&ZBufferTo3DType);
    return m;
}
