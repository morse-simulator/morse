#include "ors_poster.h"

int finalize (POSTER_ID id)
{
	posterDelete(id);

	return 0;
}
