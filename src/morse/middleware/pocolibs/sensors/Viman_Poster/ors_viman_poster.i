%module ors_viman_poster
%include "typemaps.i"

%apply int* OUTPUT {int* ok};


%{
/* Include the header in the wrapper code */
#include "ors_viman_poster.h"
%}

%include "carrays.i"
/*
%array_class(struct simu_image_init, imageInitArray);
%array_class(struct simu_image, imageArray);
*/

/* Parse the header file to generate wrappers */
%include "vimanConst.h"
%include "vimanStruct.h"
%include "ors_viman_poster.h"
