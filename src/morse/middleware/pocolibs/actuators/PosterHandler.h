#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdbool.h>

#include <posterLib.h>

typedef struct PosterHandler {
	char* poster_name;
	bool found;
	POSTER_ID id;
} PosterHandler;

static void find_poster(PosterHandler* handler)
{
	STATUS s = posterFind (handler->poster_name, &handler->id);
	if (s == OK) {
		printf("Located poster %s\n", handler->poster_name);
		handler->found = true;
	}
}

static PosterHandler* createPosterHandler(const char* poster_name)
{
	PosterHandler* res = malloc(sizeof(PosterHandler));
	res->poster_name = strdup(poster_name);
	res->found = false;
	find_poster(res);
	return res;
}

static void destroyPosterHandler(PosterHandler* handler)
{
	free(handler->poster_name);
	if (handler->found)
		posterDelete(handler->id);
	free(handler);
}

static void read_poster(PosterHandler* handler, int* ok, void* data, size_t size)
{
	if (!handler->found)
		find_poster(handler);

	if (!handler->found) { 
		*ok = 0; 
		return; 
	}

	size_t err = posterRead(handler->id, 0, data, size);
	if (err != size) {
		printf("Error when reading poster %s : expected %zd bytes, got %zd\n",
				handler->poster_name, size, err);
		*ok = 0;
	} else 
		*ok = 1;
}
