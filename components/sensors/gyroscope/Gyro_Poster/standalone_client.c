#include <stdio.h>
#include <stdlib.h>
#include <string.h>
//#include "ors_genpos_poster.h"

#include <genPos/genPosStruct.h>
#include <posterLib.h>


POSTER_ID init_data (char*	poster_name)
{
	POSTER_ID id;

	STATUS s = posterCreate (poster_name, sizeof(GENPOS_CART_SPEED), &id);
	if (s == ERROR)
	{
		char buf[1024];
		h2getErrMsg(errnoGet(), buf, sizeof(buf));
		printf ("Unable to create the GENPOS poster : %s\n",buf);
		return (NULL);
	}
	/*
	else
		printf ("INIT ID = %p (pointer)   %d(integer)\n", id);
	*/

	return (id);
}

int write_genPos_data( POSTER_ID id, double v, double w )
{
	GENPOS_CART_SPEED local_genPos;
	int offset = 0;

	// Store the variables into the structure
	local_genPos.v = v;
	local_genPos.w = w;

	posterWrite (id, offset, &local_genPos, sizeof(GENPOS_CART_SPEED));

	printf ("DATA WRITEN TO POSTER:");
	printf ("\tv = %.4f", v);
	printf ("\tw = %.4f\n", w);

	return 0;
}


int finalize (POSTER_ID id)
{
	posterDelete(id);

	return 0;
}


int main (void)
{
	POSTER_ID id;
	char* poster_name = "CLIENT_GENPOS_POSTER";
	char key;
	double v = 0.0;
	double w = 0.0;

	id = init_data(poster_name);

	printf ("Poster client to drive the robot\n");
	printf ("Use the keys: w, a, s, d\n");
	printf ("Exit by pressing q\n");

	// Read the keyboard until 'q' is pressed
	do
	{
		key = getchar();
		if (key == 'w')
			v += 0.02;
		else if (key == 's')
			v -= 0.02;
		else if (key == 'a')
			w += 0.02;
		else if (key == 'd')
			w -= 0.02;

		write_genPos_data(id, v, w);

	} while (key != 'q');

	finalize(id);
}
