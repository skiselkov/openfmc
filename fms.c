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
#include <string.h>
#include <stdlib.h>
#include <xlocale.h>
#include <time.h>
#include <errno.h>
#include <regex.h>

#include "log.h"
#include "fms.h"

/*
 * Retrieves the time/date validity of a navigation database.
 *
 * @param navdata_dir Directory containing the database's "Airports.txt".
 * @param cyclep Will be filled with the AIRAC cycle of the DB.
 * @param fromp Will be filled with the DB's start of validity UNIXTIME.
 * @param top Will be filled with the DB's end of validity UNIXTIME.
 *
 * @return B_TRUE on successful read of the DB, B_FALSE on failure.
 */
static bool_t
navdata_get_valid(const char *navdata_dir, unsigned *cyclep, time_t *fromp,
    time_t *top)
{
	FILE		*arpt_fp;
	char		*arpt_fname;
	ssize_t		line_len = 0;
	size_t		line_cap = 0;
	char		*line = NULL;
	char		*comps[5];
	locale_t	loc = NULL;
	struct tm	tm_start, tm_end;

	/* The file containing the validity line is "Airports.txt" */
	arpt_fname = malloc(strlen(navdata_dir) +
	    strlen(PATHSEP "Airports.txt") + 1);
	sprintf(arpt_fname, "%s" PATHSEP "Airports.txt", navdata_dir);
	arpt_fp = fopen(arpt_fname, "r");
	if (arpt_fp == NULL) {
		openfmc_log(OPENFMC_LOG_ERR, "Can't open %s: %s", arpt_fname,
		    strerror(errno));
		goto errout;
	}

	/*
	 * Initialize global C locale to give us English month names and also
	 * pre-fill tm_start/tm_end with sensible values.
	 */
	loc = newlocale(LC_TIME_MASK, NULL, LC_GLOBAL_LOCALE);
	memset(&tm_start, 0, sizeof (tm_start));
	memset(&tm_end, 0, sizeof (tm_end));
	tm_start.tm_zone = "UTC";
	tm_end.tm_zone = "UTC";
	tm_end.tm_sec = 59;
	tm_end.tm_min = 59;
	tm_end.tm_hour = 23;

	if (loc == NULL) {
		openfmc_log(OPENFMC_LOG_ERR,
		    "Can't open locale C: %s", strerror(errno));
		goto errout;
	}

	/* Locate 'X' line at the start of the file. */
	while ((line_len = getline(&line, &line_cap, arpt_fp)) != -1) {
		unsigned cycle;

		strip_newline(line);
		if (explode_line(line, ',', comps, 5) != 5 ||
		    strcmp(comps[0], "X") != 0)
			continue;

		/* Check AIRAC cycle number */
		cycle = atoi(comps[1]);
		if (strlen(comps[1]) != 4 || cycle < 1 || cycle > 9913 ||
		    cycle % 100 > 13) {
			openfmc_log(OPENFMC_LOG_ERR, "Error validating AIRAC "
			    "cycle number: \"%s\" is malformed.", comps[1]);
			goto errout;
		}
		*cyclep = cycle;

		/* Check validity period format */
		if (strlen(comps[2]) != 13 ||
		    strptime_l(&comps[2][11], "%y", &tm_start, loc) == NULL ||
		    strptime_l(&comps[2][0], "%d%b", &tm_start, loc) == NULL ||
		    strptime_l(&comps[2][11], "%y", &tm_end, loc) == NULL ||
		    strptime_l(&comps[2][5], "%d%b", &tm_end, loc) == NULL) {
			openfmc_log(OPENFMC_LOG_ERR, "Error validating AIRAC "
			    "cycle date: \"%s\" is invalid.", comps[2]);
			goto errout;
		}
		/* If the months are disordered, roll over end year */
		if (tm_start.tm_mon > tm_end.tm_mon)
			tm_end.tm_year++;
		*fromp = timegm(&tm_start);
		*top = timegm(&tm_end);
		break;
	}
	if (line_len == -1)
		goto errout;

	fclose(arpt_fp);
	free(arpt_fname);
	free(line);
	freelocale(loc);

	return (B_TRUE);
errout:
	if (arpt_fp != NULL)
		fclose(arpt_fp);
	free(arpt_fname);
	free(line);
	if (loc != NULL)
		freelocale(loc);

	return (B_FALSE);
}

static bool_t
fms_alloc_regex(fms_t *fms)
{
	int err;

#define	COMPILE_REGEX(preg, pattern) \
	do { \
		ASSERT((preg) == NULL); \
		(preg) = malloc(sizeof (regex_t)); \
		VERIFY((preg) != NULL); \
		if ((err = regcomp((preg), (pattern), REG_EXTENDED)) != 0) { \
			openfmc_log(OPENFMC_LOG_ERR, "Error compiling regex " \
			    "\"%s\": compile error %d", (pattern), err); \
			free((preg)); \
			return (B_FALSE); \
		} \
	} while (0)

	COMPILE_REGEX(fms->regex.wptname, "^([A-Z]{1,5})$");
	COMPILE_REGEX(fms->regex.navaid_blw100, "^([A-Z]{1,3})([0-9]{2})$");
	COMPILE_REGEX(fms->regex.navaid_abv100, "^([0-9]{2})([A-Z]{1,3})$");
	/* 5010N, 50N10 */
	COMPILE_REGEX(fms->regex.geo_nw_blw100, "^([0-9]{2})([0-9]{2})N$");
	COMPILE_REGEX(fms->regex.geo_nw_abv100, "^([0-9]{2})N([0-9]{2})$");
	/* 5010E, 50E10 */
	COMPILE_REGEX(fms->regex.geo_ne_blw100, "^([0-9]{2})([0-9]{2})E$");
	COMPILE_REGEX(fms->regex.geo_ne_abv100, "^([0-9]{2})E([0-9]{2})$");
	/* 5010W, 50W10 */
	COMPILE_REGEX(fms->regex.geo_sw_blw100, "^([0-9]{2})([0-9]{2})W$");
	COMPILE_REGEX(fms->regex.geo_sw_abv100, "^([0-9]{2})W([0-9]{2})$");
	/* 5010S, 50S10 */
	COMPILE_REGEX(fms->regex.geo_se_blw100, "^([0-9]{2})([0-9]{2})S$");
	COMPILE_REGEX(fms->regex.geo_se_abv100, "^([0-9]{2})S([0-9]{2})$");

	/* SEA330/10 */
	COMPILE_REGEX(fms->regex.radial_dme,
	    "^([A-Z]{1,5})([0-9]{3})/([0-9]{1,3})$");
	/* SEA330/OLM020 */
	COMPILE_REGEX(fms->regex.radial_isect,
	    "^([A-Z]{1,5})([0-9]{3})/([A-Z]{1,5})([0-9]{3})$");

	/* VAMPS/25, ELN/-30 */
	COMPILE_REGEX(fms->regex.along_trk, "^([A-Z]{1,5})/([-]?[0-9]{1,3})$");

#undef	COMPILE_REGEX

	return (B_TRUE);
}

static void
fms_free_regex(fms_t *fms)
{
#define	DESTROY_REGEX(preg) \
	do { \
		if ((preg) != NULL) { \
			regfree((preg)); \
			free((preg)); \
			(preg) = NULL; \
		} \
	} while (0)

	DESTROY_REGEX(fms->regex.wptname);
	DESTROY_REGEX(fms->regex.navaid_blw100);
	DESTROY_REGEX(fms->regex.navaid_abv100);

	DESTROY_REGEX(fms->regex.geo_nw_blw100);
	DESTROY_REGEX(fms->regex.geo_nw_abv100);
	DESTROY_REGEX(fms->regex.geo_ne_blw100);
	DESTROY_REGEX(fms->regex.geo_ne_abv100);
	DESTROY_REGEX(fms->regex.geo_sw_blw100);
	DESTROY_REGEX(fms->regex.geo_sw_abv100);
	DESTROY_REGEX(fms->regex.geo_se_blw100);
	DESTROY_REGEX(fms->regex.geo_se_abv100);

	DESTROY_REGEX(fms->regex.radial_dme);
	DESTROY_REGEX(fms->regex.radial_isect);

	DESTROY_REGEX(fms->regex.along_trk);

#undef	DESTROY_REGEX
}

fms_t *
fms_new(const char *navdata_dir, const char *wmm_file)
{
	fms_t *fms = calloc(sizeof (*fms), 1);

	fms->navdb = fms_navdb_open(navdata_dir, wmm_file);
	if (!fms->navdb)
		goto errout;
	if (!fms_alloc_regex(fms))
		goto errout;

	return (fms);
errout:
	fms_destroy(fms);
	return (NULL);
}

void
fms_destroy(fms_t *fms)
{
	if (fms->navdb)
		fms_navdb_close(fms->navdb);
	fms_free_regex(fms);
	free(fms);
}

/*
 * Opens and constructs a navigational database + the world magnetic model.
 * Argument should be self-explanatory.
 *
 * @return The database on success, NULL on failure.
 */
fms_navdb_t *
fms_navdb_open(const char *navdata_dir, const char *wmm_file)
{
	time_t t = time(NULL);
	struct tm now;
	fms_navdb_t *navdb;

	localtime_r(&t, &now);

	navdb = calloc(sizeof (*navdb), 1);
	if (!navdata_get_valid(navdata_dir, &navdb->airac_cycle,
	    &navdb->valid_from, &navdb->valid_to))
		goto errout;
	navdb->navdata_dir = strdup(navdata_dir);
	navdb->wmm_file = strdup(wmm_file);
	if (navdb->navdata_dir == NULL || navdb->wmm_file == NULL)
		goto errout;
	navdb->navaiddb = navaid_db_open(navdata_dir);
	navdb->wptdb = waypoint_db_open(navdata_dir);
	if (navdb->navaiddb == NULL || navdb->wptdb == NULL)
		goto errout;
	navdb->awydb = airway_db_open(navdata_dir,
	    htbl_count(&navdb->wptdb->by_name));
	if (navdb->awydb == NULL)
		goto errout;

	navdb->wmm = wmm_open(wmm_file, 1900.0 + now.tm_year +
	    (now.tm_yday / 365.0));
	if (navdb->wmm == NULL)
		goto errout;

	return (navdb);
errout:
	fms_navdb_close(navdb);
	return (NULL);
}

/*
 * Closes a navigational database opened with navdb_open and frees
 * all of its resources.
 */
void
fms_navdb_close(fms_navdb_t *navdb)
{
	free(navdb->navdata_dir);
	if (navdb->awydb != NULL)
		airway_db_close(navdb->awydb);
	if (navdb->wptdb != NULL)
		waypoint_db_close(navdb->wptdb);
	if (navdb->navaiddb != NULL)
		navaid_db_close(navdb->navaiddb);
	free(navdb->wmm_file);
	if (navdb->wmm)
		wmm_close(navdb->wmm);
	free(navdb);
}

/*
 * Checks if the navigational database is current (i.e. its validity
 * period falls within the current time).
 */
bool_t
navdb_is_current(const fms_navdb_t *navdb)
{
	time_t now = time(NULL);
	return (navdb->valid_from <= now && now <= navdb->valid_to);
}

bool_t
navdata_is_current(const char *navdata_dir)
{
	time_t now = time(NULL);
	unsigned cycle;
	time_t from, to;

	return (navdata_get_valid(navdata_dir, &cycle, &from, &to) &&
	    from <= now && now <= to);
}

static size_t
fms_fix_lookup(const char *fixname, const fms_t *fms)
{
	/* TODO */
}

fix_t *
fms_wpt_name_decode(const char *name, fms_t *fms, size_t *num)
{
	regmatch_t pmatch[4];
	int n;

	if ((n = regexec(fms->regex.wptname, name, 1, pmatch, 0)) == 0) {
		char fixname[NAV_NAME_LEN];
	}
}
