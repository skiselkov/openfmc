/*
 * CDDL HEADER START
 *
 * The contents of this file are subject to the terms of the
 * Common Development and Distribution License, Version 1.0 only
 * (the "License").  You may not use this file except in compliance
 * with the License.
 *
 * You can obtain a copy of the license at usr/src/OPENSOLARIS.LICENSE
 * or http://www.opensource.org/licenses/CDDL-1.0.
 * See the License for the specific language governing permissions
 * and limitations under the License.
 *
 * When distributing Covered Code, include this CDDL HEADER in each
 * file and include the License file at usr/src/OPENSOLARIS.LICENSE.
 * If applicable, add the following below this CDDL HEADER, with the
 * fields enclosed by brackets "[]" replaced with your own identifying
 * information: Portions Copyright [yyyy] [name of copyright owner]
 *
 * CDDL HEADER END
 */
/*
 * Copyright 2015 Saso Kiselkov. All rights reserved.
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <errno.h>

#include "helpers.h"
#include "airac.h"

static void
test_arpts(const char *navdata_dir)
{
	char		*line = NULL;
	size_t		linecap = 0;
	ssize_t		line_len;
	char		*arpt_fname;
	FILE		*arpt_fp;

	arpt_fname = malloc(strlen(navdata_dir) + 1 +
	    strlen("Airports.txt") + 1);
	sprintf(arpt_fname, "%s" PATHSEP "%s", navdata_dir, "Airports.txt");
	arpt_fp = fopen(arpt_fname, "r");
	if (arpt_fp == NULL) {
		fprintf(stderr, "Can't open %s: %s\n", arpt_fname,
		    strerror(errno));
		exit(EXIT_FAILURE);
	}
	while ((line_len = getline(&line, &linecap, arpt_fp)) != -1) {
		char		*comps[10];
		airport_t	*arpt;

		if (explode_line(line, ',', comps, 10) != 10 ||
		    strcmp(comps[0], "A") != 0)
			continue;

		arpt = airport_open(comps[1], navdata_dir);
		if (arpt) {
#if 0
			if (strcmp(dump, "airport") == 0) {
				char *desc = airport_dump(arpt);
				fputs(desc, stdout);
				free(desc);
			}
#endif
			airport_close(arpt);
		} else {
			exit(EXIT_FAILURE);
		}
	}
	free(arpt_fname);
	fclose(arpt_fp);
}

static void
test_airac(const char *navdata_dir, const char *dump)
{
	airway_db_t	*awydb;
	waypoint_db_t	*wptdb;
	navaid_db_t	*navaiddb;
	size_t		num_waypoints = 0;

	test_arpts(navdata_dir);

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
			    "|wpt|navaid] <navdata_dir>\n",
			    argv[0]);
			return (1);
		}
	}

	if (argc - optind != 1) {
		fprintf(stderr, "Missing navdata argument\n");
		return (1);
	}

	test_airac(argv[optind], dump);

	return (0);
}
