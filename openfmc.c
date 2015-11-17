#include <stdio.h>
#include <stdlib.h>

#include "airac.h"

int
main(int argc, char **argv)
{
	airport_t *arpt;
	airway_db_t *awydb;

	if (argc != 3) {
		fprintf(stderr, "Missing navdata dir & airport argument\n");
		return (1);
	}

	arpt = airport_open(argv[2], argv[1]);
	if (arpt) {
		char *desc = airport_dump(arpt);
		fputs(desc, stdout);
		free(desc);
		airport_close(arpt);
	}

	awydb = airway_db_open(argv[1]);
	if (awydb) {
		airway_db_close(awydb);
	}

	return (0);
}
