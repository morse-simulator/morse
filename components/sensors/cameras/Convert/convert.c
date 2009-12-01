#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "convert.h"


unsigned char*	image_array;


int init_array (int length)
{
	image_array = (unsigned char*) malloc (sizeof (unsigned char) * (length * 3/4));
	if (image_array == NULL)
	{
		printf ("C module ERROR: Could not allocate memory\n");
		return 1;
	}

	return 0;
}

unsigned char* convert_image( unsigned char* image_string, int length )
{
	int i, j=0;
	int C_length = strlen (image_string);

	// Go through the image string one RGBA pixel at a time
	for (i=0; i<length; i+=4)
	{
		// Get the R,G,B components of a pixel
		image_array[j++] = (unsigned char) image_string[i];
		image_array[j++] = (unsigned char) image_string[i+1];
		image_array[j++] = (unsigned char) image_string[i+2];
	}

	/*
	// Print the results
	printf ("C unsigned - Converted string of length %d (local %d)\n", length, C_length);
	printf ("\tFrom: '%s'\n", image_string);
	printf ("\tTo  : [", image_string);
	for (i=0; i<j; i++)
		printf ("%d, ", image_array[i]);
	printf ("]\n");
	*/

	return image_array;
}
