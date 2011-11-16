%module ors_platine_poster
%include "typemaps.i"
/* Apply a typemap that preserves the length of a binary string.
   It also gets the length and passes it as an argument */
%apply (char *STRING, int LENGTH) { (unsigned char *image_string, int length) };
%apply int* OUTPUT {int* ok};

/*
%include "carrays.i"
%array_class(unsigned int, uintArray);
*/

%{
/* Include the header in the wrapper code */
#include "ors_platine_poster.h"
%}

/* Parse the header file to generate wrappers */
%include "pom/pomStruct.h"
%include "platine/platineStruct.h"
%include "../PosterHandler.h"
%include "ors_platine_poster.h"
