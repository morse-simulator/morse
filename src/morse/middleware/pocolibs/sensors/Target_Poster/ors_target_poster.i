%module ors_target_poster
%include "typemaps.i"

%apply int* OUTPUT {int* ok};


%{
/* Include the header in the wrapper code */
#include "ors_target_poster.h"
%}

/* Parse the header file to generate wrappers */
%include "ors_target_poster.h"
