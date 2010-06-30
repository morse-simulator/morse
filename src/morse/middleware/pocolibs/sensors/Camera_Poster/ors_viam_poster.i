%module ors_viam_poster
%include "typemaps.i"

%apply int* OUTPUT {int* ok};


%{
/* Include the header in the wrapper code */
#include "ors_viam_poster.h"
%}

%include "carrays.i"
%array_class(struct simu_image_init, imageInitArray);
%array_class(struct simu_image, imageArray);

/* Parse the header file to generate wrappers */
%include "ors_viam_poster.h"
