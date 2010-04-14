%module ors_viam_poster
%include "typemaps.i"
/* Apply a typemap that preserves the length of a binary string.
   It also gets the length and passes it as an argument */
%apply (char *STRING, int LENGTH) { (unsigned char *image_string, int length) };

/*
%include "carrays.i"
%array_class(unsigned int, uintArray);
*/

%{
/* Include the header in the wrapper code */
#include "ors_viam_poster.h"
#include <pom/pomStruct.h>
#include <viam/viamStruct.h>
#include <viam/viamtypes.h>
%}

/* Parse the header file to generate wrappers */
%include "ors_viam_poster.h"
%include "/home/gechever/openrobots/include/pom/pomStruct.h"
%include "/home/gechever/openrobots/include/viam/viamStruct.h"
%include "/home/gechever/openrobots/include/viam/viamtypes.h"
