%module ors_poster
%include "typemaps.i"
/* Apply a typemap that preserves the length of a binary string.
   It also gets the length and passes it as an argument
%apply (char *STRING, int LENGTH) { (unsigned char *image_string, int length) };
*/



%{
/* Include the header in the wrapper code */
#include "ors_poster.h"
%}

%include "carrays.i"

/* Parse the header file to generate wrappers */
%include "ors_poster.h"
