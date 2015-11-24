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
#include <cairo.h>

#include <png.h>

#include "helpers.h"
#include "airac.h"

#define	IMGH		1200
#define	IMGW		1200
#define	FONTSZ		16
#define	REFLON		0
#define	CHANNELS	4
#define	BPP		8
#define	FORMAT		PNG_COLOR_TYPE_RGBA
#define	SET_PIXEL(x, y, a, r, g, b) \
	do { \
		if ((x) >= 0 && (x) < IMGW && (y) >= 0 && (y) < IMGH) { \
			rows[(y)][CHANNELS * (x) + (CHANNELS - 4)] = (a); \
			rows[(y)][CHANNELS * (x) + (CHANNELS - 3)] = (r); \
			rows[(y)][CHANNELS * (x) + (CHANNELS - 2)] = (g); \
			rows[(y)][CHANNELS * (x) + (CHANNELS - 1)] = (b); \
		} \
	} while (0)

static void
prep_png_img(uint8_t *img, png_bytepp png_rows, size_t w, size_t h)
{
	for (size_t r = 0; r < h; r++) {
		png_rows[r] = &img[(r * w) * CHANNELS];
		for (size_t c = 0; c < w; c++) {
			img[(r * w + c) * CHANNELS + 3] = 0xff;
		}
	}
}

static void
xlate_png_byteorder(uint8_t *imgp, size_t w, size_t h)
{
	uint32_t *img = (uint32_t *)imgp;

	for (size_t i = 0; i < w * h; i++) {
		/* assumes little endian */
		img[i] = (img[i] & 0xff000000u) |
		    ((img[i] & 0x00ff0000) >> 16) |
		    (img[i] & 0x0000ff00) |
		    ((img[i] & 0x000000ff) << 16);
	}
}

static void
write_png_img(const char *filename, const png_bytepp rows, size_t w, size_t h)
{
	png_structp	png_ptr;
	png_infop	info_ptr;
	FILE *fp = fopen(filename, "wb");

	VERIFY(fp != NULL);
	png_ptr = png_create_write_struct(PNG_LIBPNG_VER_STRING,
	    NULL, NULL, NULL);
	VERIFY(png_ptr != NULL);
	info_ptr = png_create_info_struct(png_ptr);
	VERIFY(info_ptr != NULL);

	VERIFY(setjmp(png_jmpbuf(png_ptr)) == 0);
	png_init_io(png_ptr, fp);
	png_set_IHDR(png_ptr, info_ptr, w, h, BPP, PNG_COLOR_TYPE_RGBA,
	    PNG_INTERLACE_NONE, PNG_COMPRESSION_TYPE_DEFAULT,
	    PNG_FILTER_TYPE_DEFAULT);
	png_write_info(png_ptr, info_ptr);
	png_write_image(png_ptr, rows);
	png_write_end(png_ptr, NULL);
	fclose(fp);
	png_free_data(png_ptr, info_ptr, PNG_FREE_ALL, -1);
	png_destroy_write_struct(&png_ptr, (png_infopp)NULL);
}

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

vect2_t
test_fpp_xy(double lat, double lon, const fpp_t *fpp, double scale)
{
	vect2_t pos = geo2fpp(GEO_POS2(lat, lon), fpp);
	pos.x = pos.x * ((IMGW - 1) / (2.0 * scale)) + IMGW / 2;
	pos.y = IMGH - (pos.y * ((IMGH - 1) / (2.0 * scale)) + IMGH / 2);
	return (pos);
}

void
test_fpp(void)
{
	fpp_t			fpp;
	png_bytep		rows[IMGH];
	uint8_t			*img;
	cairo_surface_t		*surface;
	cairo_t			*cr;
	char			text[64];
	cairo_text_extents_t	ext;

#define	LAT0	0
#define	LAT1	90
#define	LATi	5
#define	LON0	-90
#define	LON1	90
#define	LONi	5
#define	SCALE	1

#define	PROJLAT	45
#define	PROJLON	45
#define	PROJROT	45

	fpp = ortho_fpp_init(GEO_POS2(PROJLAT, PROJLON), PROJROT);

	surface = cairo_image_surface_create(CAIRO_FORMAT_ARGB32, IMGW, IMGH);
	cr = cairo_create(surface);
	img = cairo_image_surface_get_data(surface);

	prep_png_img(img, rows, IMGW, IMGH);

	cairo_set_antialias(cr, CAIRO_ANTIALIAS_DEFAULT);
	cairo_set_source_rgb(cr, 1, 1, 1);
	cairo_set_line_width(cr, 2);
	for (int lat = LAT0; lat < LAT1; lat += LATi) {
		vect2_t p = test_fpp_xy(lat, LON0, &fpp, SCALE * EARTH_MSL);
		if (IS_NULL_VECT(p))
			continue;
		cairo_move_to(cr, p.x, p.y);
		for (int lon = LON0 + LONi; lon <= LON1; lon += LONi) {
			p = test_fpp_xy(lat, lon, &fpp, SCALE * EARTH_MSL);
			if (IS_NULL_VECT(p))
				continue;
			cairo_line_to(cr, p.x, p.y);
		}
		cairo_stroke(cr);
	}
	for (int lon = LON0; lon <= LON1; lon += LONi) {
		vect2_t p = test_fpp_xy(LAT1, lon, &fpp, SCALE * EARTH_MSL);
		if (IS_NULL_VECT(p))
			continue;
		cairo_move_to(cr, p.x, p.y);
		for (int lat = LAT1 - LATi; lat >= LAT0; lat -= LATi) {
			p = test_fpp_xy(lat, lon, &fpp, SCALE * EARTH_MSL);
			if (IS_NULL_VECT(p))
				continue;
			cairo_line_to(cr, p.x, p.y);
		}
		cairo_stroke(cr);
	}

	snprintf(text, sizeof (text), "lat: %d lon: %d rot: %d",
	    PROJLAT, PROJLON, PROJROT);
	cairo_set_font_size(cr, FONTSZ);

	cairo_set_source_rgb(cr, 0, 0, 0);
	cairo_move_to(cr, IMGW / 2 + FONTSZ / 2, IMGH / 2 - FONTSZ / 2);
	cairo_text_extents(cr, text, &ext);
	cairo_line_to(cr, IMGW / 2 + 1.5 * FONTSZ + ext.width,
	    IMGH / 2 - FONTSZ / 2);
	cairo_line_to(cr, IMGW / 2 + 1.5 * FONTSZ + ext.width,
	    IMGH / 2 - 1.5 * FONTSZ - ext.height);
	cairo_line_to(cr, IMGW / 2 + FONTSZ / 2,
	    IMGH / 2 - 1.5 * FONTSZ - ext.height);
	cairo_close_path(cr);
	cairo_fill(cr);

	cairo_set_source_rgb(cr, 1, 0, 0);
	cairo_move_to(cr, IMGW / 2 + FONTSZ, IMGH / 2 - FONTSZ);
	cairo_show_text(cr, text);

	cairo_set_antialias(cr, CAIRO_ANTIALIAS_NONE);
	cairo_set_source_rgb(cr, 1, 0, 0);
	cairo_set_line_width(cr, 1);
	cairo_move_to(cr, 0, IMGH / 2);
	cairo_line_to(cr, IMGW, IMGH / 2);
	cairo_stroke(cr);
	cairo_move_to(cr, IMGW / 2, 0);
	cairo_line_to(cr, IMGW / 2, IMGH);
	cairo_stroke(cr);

	xlate_png_byteorder(img, IMGW, IMGH);
	write_png_img("test.png", rows, IMGW, IMGH);

	cairo_destroy(cr);
}

void
test_lcc(double reflat, double stdpar1, double stdpar2)
{
	uint8_t		*img;
	png_bytep	rows[IMGH];
	double		scale_factor, offset;

	lcc_t	lcc = lcc_init(reflat, REFLON, stdpar1, stdpar2);
	vect2_t	tip;

	img = calloc(IMGW * IMGH * CHANNELS, 1);
	prep_png_img(img, rows, IMGW, IMGH);

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
						SET_PIXEL(x + i, y + j, 255,
						    255, 0, 0);
					}
				}
			} else if (lat == stdpar1 || lat == stdpar2) {
				SET_PIXEL(x, y, 255, 0, 255, 0);
			} else {
				SET_PIXEL(x, y, 255, 255, 255, 255);
			}
		}
		if (lat < 88.0)
			lat += 2.0;
		else if (lat < 89.5)
			lat += 0.5;
		else
			lat += 0.1;
	}

	xlate_png_byteorder(img, IMGW, IMGH);
	write_png_img("test.png", rows, IMGW, IMGH);

	free(img);
}

void
test_gc(void)
{
	printf("hdg: %lf\n", gc_point_hdg(GEO_POS2(0, 0), GEO_POS2(45, 90),
	    80));
}

void
test_geo_xlate(void)
{
	geo_xlate_t	xlate = geo_xlate_init(GEO_POS2(0, 90), 0);
	geo_pos2_t	pos = geo_xlate(GEO_POS2(0, 0), &xlate);
	printf("pos.lat: %lf  pos.lon: %lf\n", pos.lat, pos.lon);
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
	test_lcc(40, 30, 50);
*/
	test_fpp();
	test_geo_xlate();

	return (0);
}
