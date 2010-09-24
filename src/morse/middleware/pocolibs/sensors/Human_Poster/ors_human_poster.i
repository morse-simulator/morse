%module ors_human_poster
%include "typemaps.i"

%apply int* OUTPUT {int* ok};

%{
/* Include the header in the wrapper code */
#include <genBasic/genBasicStruct.h>
#include <genManip/genManipStruct.h>
#include "ors_human_poster.h"
%}

/* Parse the header file to generate wrappers */
%include "ors_human_poster.h"
