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

#include <png.h>

#include "helpers.h"
#include "airac.h"

void
test_arpts(const char *navdata_dir, const char *dump,
    const waypoint_db_t *wptdb, const navaid_db_t *navdb)
{
	char		*line = NULL;
	size_t		linecap = 0;
	ssize_t		line_len;
	char		*arpt_fname;
	FILE		*arpt_fp;

	if (strlen(dump) == 4) {
		airport_t *arpt = airport_open(dump, navdata_dir, wptdb, navdb);
		if (!arpt)
			exit(EXIT_FAILURE);
		char *desc = airport_dump(arpt);
		fputs(desc, stdout);
		free(desc);
		airport_close(arpt);
		return;
	} else if (strlen(dump) > 0)
		return;

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

		arpt = airport_open(comps[1], navdata_dir, wptdb, navdb);
		if (arpt)
		//	exit(EXIT_FAILURE);
			airport_close(arpt);
	}
	free(arpt_fname);
	fclose(arpt_fp);
}

void
test_airac(const char *navdata_dir, const char *dump)
{
	airway_db_t	*awydb;
	waypoint_db_t	*wptdb;
	navaid_db_t	*navdb;

	navdb = navaid_db_open(navdata_dir);
	wptdb = waypoint_db_open(navdata_dir);
	if (!wptdb)
		exit(EXIT_FAILURE);
	awydb = airway_db_open(navdata_dir, htbl_count(&wptdb->by_name));
	if (!navdb || !awydb)
		exit(EXIT_FAILURE);

	test_arpts(navdata_dir, dump, wptdb, navdb);
	if (strcmp(dump, "wpt") == 0) {
		char *desc = waypoint_db_dump(wptdb);
		fputs(desc, stdout);
		free(desc);
	}
	if (strcmp(dump, "awyname") == 0) {
		char *desc = airway_db_dump(awydb, B_TRUE);
		fputs(desc, stdout);
		free(desc);
	} else if (strcmp(dump, "awyfix") == 0) {
		char *desc = airway_db_dump(awydb, B_FALSE);
		fputs(desc, stdout);
		free(desc);
	}
	if (strcmp(dump, "navaid") == 0) {
		char *desc = navaid_db_dump(navdb);
		fputs(desc, stdout);
		free(desc);
	}

	airway_db_close(awydb);
	waypoint_db_close(wptdb);
	navaid_db_close(navdb);
}

void
test_lcc(double reflat, double stdpar1, double stdpar2)
{
#define	IMGH		1200
#define	IMGW		1200
#define	REFLON		0
#define	BPP		3
#define	FILL_POLY(x, y, r, g, b) \
	do { \
		if (x < 0 || x >= IMGW || y < 0 || y >= IMGH) \
			continue; \
		rows[(y)][BPP * (x)] = r; \
		rows[(y)][BPP * (x) + 1] = g; \
		rows[(y)][BPP * (x) + 2] = b; \
	} while (0)

	FILE		*fp;
	png_structp	png_ptr;
	png_infop	info_ptr;
	png_bytep	rows[IMGH];
	double		scale_factor, offset;

	lcc_t	lcc = lcc_init(reflat, REFLON, stdpar1, stdpar2);
	vect2_t	tip;

	for (int i = 0; i < IMGH; i++) {
		rows[i] = calloc(sizeof (png_byte) * IMGW * 3, 1);
	}

	{
		geo_pos2_t	min_lat = {-40, 0};
		geo_pos2_t	max_lat = {89.999999, 0};
		vect2_t		res_min = geo2lcc(min_lat, &lcc);
		vect2_t		res_max = geo2lcc(max_lat, &lcc);

		printf("min: %lf max: %lf\n", res_min.y, res_max.y);
		scale_factor = IMGH / (res_max.y - res_min.y) / 2;
		offset = res_min.y * scale_factor;
		printf("offset: %lf   scale_factor: %lf\n", offset,
		    scale_factor);
		tip = res_max;
	}

	for (double lat = -80; lat < 90.0;) {
		geo_pos2_t	maxw_pos = {lat, -90.0};
		vect2_t		maxw = geo2lcc(maxw_pos, &lcc);
		for (double lon = -160.0; lon <= 160.0;) {
			geo_pos2_t	pos = {lat, lon};
			vect2_t		res = geo2lcc(pos, &lcc);
			int		x, y;
			double scale_factor_x =
			    (res.y - tip.y) / (maxw.x - tip.x);

			x = (res.x * scale_factor_x * scale_factor) + IMGW / 2;

			y = IMGH - ((res.y * scale_factor - offset) +
			    (1 - cos(DEG_TO_RAD(lon))) * (tip.y - res.y) *
			    scale_factor);

			if (lat < 88.0)
				lon += 2.0;
			else if (lat < 89.5)
				lon += 5.0;
			else
				lon += 20.0;
			if (lat == reflat || lat + 1 == reflat) {
				for (int i = -1; i <= 1; i++) {
					for (int j = -1; j <= 1; j++) {
						FILL_POLY(x + i, y + j, 255,
						    0, 0);
					}
				}
			} else if (lat == stdpar1 || lat == stdpar2) {
				FILL_POLY(x, y, 0, 255, 0);
			} else {
				FILL_POLY(x, y, 255, 255, 255);
			}
		}
		if (lat < 88.0)
			lat += 2.0;
		else if (lat < 89.5)
			lat += 0.5;
		else
			lat += 0.1;
	}

	fp = fopen("test.png", "wb");
	VERIFY(fp != NULL);
	png_ptr = png_create_write_struct(PNG_LIBPNG_VER_STRING,
	    NULL, NULL, NULL);
	VERIFY(png_ptr != NULL);
	info_ptr = png_create_info_struct(png_ptr);
	VERIFY(info_ptr != NULL);

	VERIFY(setjmp(png_jmpbuf(png_ptr)) == 0);
	png_init_io(png_ptr, fp);
	png_set_IHDR(png_ptr, info_ptr, IMGW, IMGH, 8, PNG_COLOR_TYPE_RGB,
	    PNG_INTERLACE_NONE, PNG_COMPRESSION_TYPE_DEFAULT,
	    PNG_FILTER_TYPE_DEFAULT);
	png_write_info(png_ptr, info_ptr);
	png_write_image(png_ptr, rows);
	png_write_end(png_ptr, NULL);
	fclose(fp);
	png_free_data(png_ptr, info_ptr, PNG_FREE_ALL, -1);
	png_destroy_write_struct(&png_ptr, (png_infopp)NULL);

	for (int i = 0; i < IMGH; i++)
		free(rows[i]);
}

void
test_gc(void)
{
	printf("hdg: %lf\n", gc_point_hdg(GEO_POS2(0, 0), GEO_POS2(45, 90),
	    80));
}

int
main(int argc, char **argv)
{
	UNUSED(argc);
	UNUSED(argv);
/*
	char opt;
	const char *dump = "";

	while ((opt = getopt(argc, argv, "d:")) != -1) {
		switch (opt) {
		case 'd':
			dump = optarg;
			break;
		case '?':
		default:
			fprintf(stderr, "Usage: %s [-d <ICAO|awyname|awyfix"
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
*/
	test_gc();

	return (0);
}
