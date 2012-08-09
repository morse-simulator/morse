#include <Python.h>

static float alpha_u = 0.0;
static float alpha_v = 0.0;
static float near = 1.0;
static float far = 20.0;
static int width = 0;
static int height = 0;
static int u_0, v_0;


// The list to return values
static PyObject* point_list;


static PyObject* init_image_parameters(PyObject* self, PyObject* args)
{
    // Get the data as a Python object
    if (!PyArg_ParseTuple(args, "ffffII", &alpha_u, &alpha_v, &near, &far, &width, &height));
    {
        printf ("Warning: zBuffTo3Dmodule.init_image_parameters does not completely like its parameters\n");
        printf ("Got this: %f, %f, %f, %f, %d, %d\n. Should be able to continue.\n", alpha_u, alpha_v, near, far, width, height);
        //return NULL;
    }

    u_0 = width / 2;
    v_0 = height / 2;
    printf("u_0 = %d, v_0 = %d\n", u_0, v_0);

    return Py_BuildValue("i", 1);
}


static PyObject* recover_3d_point(PyObject* self, PyObject* args)
{
    Py_buffer img_buffer;

    int i;
    int pixel;
    float z_b, z_n, z_e;
    float x, y;
    int u, v;

    // Read the incomming data as a buffer. It is originally an bgl.Buffer object
    if (!PyArg_ParseTuple(args, "w*", &img_buffer))
        return NULL;

    // check that there is no division by 0
    if (width == 0)
        return NULL;

    // Create a new list to return with the 3D data
    point_list = PyList_New(width*height);

    for (i=0; i<img_buffer.len; i+=4)
    {
        pixel = i / 4;
        memcpy (&z_b, &(img_buffer.buf)[i], sizeof(float));
        z_n = 2.0 * z_b - 1.0;
        z_e = 2.0 * near * far / (far + near - z_n * (far - near));

        // The image we receive is tored as a single array of floats.
        // Pixel 0, 0 in the data is located at the bottom left, according to
        // the OpenGL conventions.
        // We need to convert this frame of reference to (u, v), starting at the
        // top left
        u = pixel % width;
        v = height - (pixel / width);

        // Use the intrinsic matrix of the camera view to get the 3D coordinates
        // corresponding to each pixel, with respect to the camera
        x = z_e * (u - u_0) / alpha_u;
        y = z_e * (v - v_0) / alpha_v;

        //printf("PIXEL %d\n", pixel);
        //printf("\tz_b %f = z_n %f = z_e %f\n", z_b, z_n, z_e);
        //printf("\tu, v (%d, %d) = x, y (%f, %f)\n", u, v, x, y);

        // Store the x, y, z coordinates in the list
        PyList_SetItem(point_list, pixel, Py_BuildValue("[f,f,f]", x, y, z_e));
    }

    // release the Python buffers
    PyBuffer_Release(&img_buffer);

    return point_list;
}


static PyObject* release_memory(PyObject* self, PyObject* args)
{

    return Py_BuildValue("i", 1);
}


static PyMethodDef zBuffTo3D_methods[] = {
    {"init_image_parameters", init_image_parameters, METH_VARARGS, "Save the parameters of the projection frustum"},
    {"recover_3d_point", recover_3d_point, METH_VARARGS, "Convert a zBuffer image into an array of 3D points"},
    {"release_memory", release_memory, METH_VARARGS, "Free the memory used to store the data for the 3D points"},
    {NULL, NULL}
};


static struct PyModuleDef zBuffTo3Dmodule = {
    PyModuleDef_HEAD_INIT,
    "zBuffTo3D",
    NULL,
    -1,
    zBuffTo3D_methods,
    NULL,
    NULL,
    NULL,
    NULL
};

 
PyMODINIT_FUNC
PyInit_zBuffTo3D(void)
{
    return PyModule_Create(&zBuffTo3Dmodule);
}
