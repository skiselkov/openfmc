#include <stdio.h>

#include "airac.h"

int
main(int argc, char **argv)
{
	airport_t *arpt;

	if (argc != 2) {
		fprintf(stderr, "Missing navdata dir argument\n");
		return (1);
	}

	arpt = airport_open("LZIB", argv[1]);

	return (0);
}
