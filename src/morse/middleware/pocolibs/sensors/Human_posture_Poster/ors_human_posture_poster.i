%module ors_human_posture_poster
%include "typemaps.i"

%{
/* Include the header in the wrapper code */
#include "ors_human_posture_poster.h"
%}

%include "carrays.i"
%array_class(double, doubleArray);

%apply int* OUTPUT {int* ok};


/* Parse the header file to generate wrappers */
%include "ors_human_posture_poster.h"
