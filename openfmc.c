#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

#include "airac.h"

static void
test_airac(const char *navdata_dir, const char *arpt_icao, const char *dump)
{
	airport_t	*arpt;
	airway_db_t	*awydb;
	waypoint_db_t	*wptdb;
	navaid_db_t	*navaiddb;
	size_t		num_waypoints = 0;

	arpt = airport_open(arpt_icao, navdata_dir);
	if (arpt) {
		if (strcmp(dump, "airport") == 0) {
			char *desc = airport_dump(arpt);
			fputs(desc, stdout);
			free(desc);
		}
		airport_close(arpt);
	}
	wptdb = waypoint_db_open(navdata_dir);
	if (wptdb) {
		num_waypoints = htbl_count(&wptdb->by_name);
		if (strcmp(dump, "wpt") == 0) {
			char *desc = waypoint_db_dump(wptdb);
			fputs(desc, stdout);
			free(desc);
		}
		waypoint_db_close(wptdb);
	}
	awydb = airway_db_open(navdata_dir, num_waypoints);
	if (awydb) {
		if (strcmp(dump, "awyname") == 0) {
			char *desc = airway_db_dump(awydb, B_TRUE);
			fputs(desc, stdout);
			free(desc);
		} else if (strcmp(dump, "awyfix") == 0) {
			char *desc = airway_db_dump(awydb, B_FALSE);
			fputs(desc, stdout);
			free(desc);
		}
		airway_db_close(awydb);
	}
	navaiddb = navaid_db_open(navdata_dir);
	if (navaiddb) {
		if (strcmp(dump, "navaid") == 0) {
			char *desc = navaid_db_dump(navaiddb);
			fputs(desc, stdout);
			free(desc);
		}
		navaid_db_close(navaiddb);
	}
}

int
main(int argc, char **argv)
{
	char opt;
	const char *dump = "";

	while ((opt = getopt(argc, argv, "d:")) != -1) {
		switch (opt) {
		case 'd':
			dump = optarg;
			break;
		case '?':
		default:
			fprintf(stderr, "Usage: %s [-d <airport|awyname|awyfix"
			    "|wpt|navaid] <navdata_dir> <arpt_icao>\n",
			    argv[0]);
			return (1);
		}
	}

	if (argc - optind != 2) {
		fprintf(stderr, "Missing navdata dir & airport argument\n");
		return (1);
	}

	for (int i = 0; i < ((strcmp(dump, "") == 0) ? 10 : 1); i++)
		test_airac(argv[optind], argv[optind + 1], dump);

	return (0);
}
