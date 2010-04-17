%module gyro_poster
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
#include "gyro_poster.hh"
%}

/* Parse the header file to generate wrappers */
%include "gyro_poster.hh"
%include "poster_write.hh"
%template(pomPoster) PosterWriter<POM_POS>;
