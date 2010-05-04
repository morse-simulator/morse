%module ors_sick_poster
%include "typemaps.i"
/* Apply a typemap that preserves the length of a binary string.
   It also gets the length and passes it as an argument
%apply (char *STRING, int LENGTH) { (unsigned char *image_string, int length) };
*/



%{
/* Include the header in the wrapper code */
#include "ors_sick_poster.h"
%}

%include "carrays.i"
/*
%array_class(struct simu_image_init, imageInitArray);
%array_class(struct simu_image, imageArray);
*/

/* Parse the header file to generate wrappers */
%include "ors_sick_poster.h"
