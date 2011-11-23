%module ors_platine_posture_poster
%include "typemaps.i"

%apply int* OUTPUT {int* ok};


%{
/* Include the header in the wrapper code */
#include "ors_platine_posture_poster.h"
%}

/* Parse the header file to generate wrappers */
%include "ors_platine_posture_poster.h"
