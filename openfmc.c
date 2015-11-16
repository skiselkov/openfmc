#include <stdio.h>

#include "airac.h"

int
main(int argc, char **argv)
{
	airport_t *arpt;

	if (argc != 3) {
		fprintf(stderr, "Missing navdata dir & airport argument\n");
		return (1);
	}

	arpt = airport_open(argv[2], argv[1]);
	if (arpt)
		airport_close(arpt);

	return (0);
}
