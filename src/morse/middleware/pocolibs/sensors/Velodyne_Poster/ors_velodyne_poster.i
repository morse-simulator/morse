%module ors_velodyne_poster
%include "typemaps.i"

%apply int* OUTPUT {int* ok};


%{
/* Include the header in the wrapper code */
#include "ors_velodyne_poster.h"
%}

%include "carrays.i"
%array_class(double, doubleArray);

/* Parse the header file to generate wrappers */
%include "velodyneClient.h"
%include "velodyneStruct.h"
%include "ors_velodyne_poster.h"
