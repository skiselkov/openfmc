#include <stdio.h>
#include <stdlib.h>

#include "airac.h"

int
main(int argc, char **argv)
{
	airport_t	*arpt;
	airway_db_t	*awydb;
	waypoint_db_t	*wptdb;
	navaid_db_t	*navaiddb;
	size_t		num_waypoints = 0;

	if (argc != 3) {
		fprintf(stderr, "Missing navdata dir & airport argument\n");
		return (1);
	}

	for (int i = 0; i < 1000; i++) {
	arpt = airport_open(argv[2], argv[1]);
	if (arpt) {
#if 0
		char *desc = airport_dump(arpt);
		fputs(desc, stdout);
		free(desc);
#endif
		airport_close(arpt);
	}
	}
	wptdb = waypoint_db_open(argv[1]);
	if (wptdb) {
		num_waypoints = htbl_count(&wptdb->by_name);
#if 0
		char *desc = waypoint_db_dump(wptdb);
		fputs(desc, stdout);
		free(desc);
#endif
		waypoint_db_close(wptdb);
	}
	awydb = airway_db_open(argv[1], num_waypoints);
	if (awydb) {
#if 0
		char *desc = airway_db_dump(awydb, B_TRUE);
		fputs(desc, stdout);
		free(desc);
#endif
#if 0
		char *desc = airway_db_dump(awydb, B_FALSE);
		fputs(desc, stdout);
		free(desc);
#endif
		airway_db_close(awydb);
	}
	navaiddb = navaid_db_open(argv[1]);
	if (navaiddb) {
#if 0
		char *desc = navaid_db_dump(navaiddb);
		fputs(desc, stdout);
		free(desc);
#endif
		navaid_db_close(navaiddb);
	}

	return (0);
}
