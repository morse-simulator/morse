#include <math.h>
#include <stdio.h>
#include <stdlib.h>

#include <Python.h>

/*
 * Generate random number between [ -1 ; 1 ]
 * More or less uniform distribution
 */
static inline double
generate_random()
{
	return 2.0 * ((double) rand()/RAND_MAX) - 1.0;
}

/* Use Polar form of the Box Muller transform */
static inline double
gaussian(double dev, double mean)
{
	double x1, x2, w, y1, y2;

	do {
		x1 = generate_random(dev);
		x2 = generate_random(dev);
		w = x1 * x1 + x2 * x2;
	} while ( w >= 1.0 );

	return dev * x1 * sqrt(-2.0 * log(w) / w) + mean;
}

/* Python wrapper for gaussian */
static PyObject* 
gaussian_gaussian(PyObject* self, PyObject* args)
{
	double dev, mean, res;
	if (!PyArg_ParseTuple(args, "dd", &dev, &mean))
		return NULL;
	res = gaussian(dev, mean);
	return Py_BuildValue("d", res);
}

/* Static method tables fro gaussian module */
static PyMethodDef GaussianMethods [] = {
	{"gaussian",  gaussian_gaussian, 2, "Compute a Gaussian value" },
	{NULL, NULL, 0, NULL}        /* Sentinel */
};

/* Init the module */
PyMODINIT_FUNC
initgaussian(void)
{
	srand(time(NULL));
	(void) Py_InitModule("gaussian", GaussianMethods);
}
