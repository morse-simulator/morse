#include "ors_poster.h"

POSTER_ID 
new_poster(char* portName, unsigned int size)
{
	POSTER_ID id;
	posterCreate(portName, size, &id);
	return id;
}

int finalize (POSTER_ID id)
{
	posterDelete(id);

	return 0;
}
