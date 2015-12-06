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

static fix_t *geofix(double lat, double lon, const char *namefmt, ...)
    PRINTF_ATTR(3);

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

		strip_space(line);
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

	/* "DOT", "ALPHA"; 1 match (wptname) */
	COMPILE_REGEX(fms->regex.wptname, "^([A-Z0-9]{1,5})$");
	/* 5010N, 50N10; 2 matches (lat, lon) */
	COMPILE_REGEX(fms->regex.geo_nw_blw100, "^([0-9]{2})([0-9]{2})N$");
	COMPILE_REGEX(fms->regex.geo_nw_abv100, "^([0-9]{2})N([0-9]{2})$");
	/* 5010E, 50E10; 2 matches (lat, lon) */
	COMPILE_REGEX(fms->regex.geo_ne_blw100, "^([0-9]{2})([0-9]{2})E$");
	COMPILE_REGEX(fms->regex.geo_ne_abv100, "^([0-9]{2})E([0-9]{2})$");
	/* 5010W, 50W10; 2 matches (lat, lon) */
	COMPILE_REGEX(fms->regex.geo_sw_blw100, "^([0-9]{2})([0-9]{2})W$");
	COMPILE_REGEX(fms->regex.geo_sw_abv100, "^([0-9]{2})W([0-9]{2})$");
	/* 5010S, 50S10; 2 matches (lat, lon) */
	COMPILE_REGEX(fms->regex.geo_se_blw100, "^([0-9]{2})([0-9]{2})S$");
	COMPILE_REGEX(fms->regex.geo_se_abv100, "^([0-9]{2})S([0-9]{2})$");
	/* N47W008; 4 matches ('N|S', lat, 'W|E', lon) */
	COMPILE_REGEX(fms->regex.geo_long,
	    "^([NS])([0-9]{2,2})([WE])([0-9]{3,3})$");
	/* N4715.4W00803.4; 6 matches ('N|S', lat, min.0, 'W|E', lon, min.0) */
	COMPILE_REGEX(fms->regex.geo_detailed,
	    "^([NS])([0-9]{2})([0-9]{2}\\.[0-9])([WE])([0-9]{3})"
	    "([0-9]{2}\\.[0-9])$");
	/* W060-10; 3 matches ('N|S|W|E', lat/lon, lat/lon-increment) */
	COMPILE_REGEX(fms->regex.geo_report,
	    "^([NSEW])([0-9]{2,3})-([0-9]{1,2})$");
	/* SEA330/10; 3 matches (wptname, radial, DME) */
	COMPILE_REGEX(fms->regex.radial_dme,
	    "^([A-Z]{1,5})([0-9]{3})/([0-9]{1,3})$");
	/* SEA330/OLM020; 4 matches (wptname1, radial1, wptname2, radial2) */
	COMPILE_REGEX(fms->regex.radial_isect,
	    "^([A-Z]{1,5})([0-9]{3})/([A-Z]{1,5})([0-9]{3})$");
	/* VAMPS/25, ELN/-30; 2 matches (wptname, [-]DME) */
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

	DESTROY_REGEX(fms->regex.geo_nw_blw100);
	DESTROY_REGEX(fms->regex.geo_nw_abv100);
	DESTROY_REGEX(fms->regex.geo_ne_blw100);
	DESTROY_REGEX(fms->regex.geo_ne_abv100);
	DESTROY_REGEX(fms->regex.geo_sw_blw100);
	DESTROY_REGEX(fms->regex.geo_sw_abv100);
	DESTROY_REGEX(fms->regex.geo_se_blw100);
	DESTROY_REGEX(fms->regex.geo_se_abv100);

	DESTROY_REGEX(fms->regex.geo_long);
	DESTROY_REGEX(fms->regex.geo_detailed);
	DESTROY_REGEX(fms->regex.geo_report);

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

static fix_t *
fms_lookup_fix_by_name(const char *fixname, const fms_t *fms, size_t *num_fixes)
{
	char name[NAV_NAME_LEN];
	fix_t *fixes = NULL;
	size_t i = 0, n = 0;
	const list_t *list;

	memset(name, 0, sizeof (name));
	strlcpy(name, fixname, sizeof (name));

	list = htbl_lookup_multi(&fms->navdb->wptdb->by_name, name);
	if (list != NULL) {
		n += list_count(list);
		fixes = realloc(fixes, n * sizeof (*fixes));
		for (const void *mv = list_head(list); mv != NULL;
		    mv = list_next(list, mv)) {
			fix_t *fix = HTBL_VALUE_MULTI(mv);
			memcpy(&fixes[i], fix, sizeof (*fixes));
			i++;
		}
	}

	list = htbl_lookup_multi(&fms->navdb->navaiddb->by_id, name);
	if (list != NULL) {
		n += list_count(list);
		fixes = realloc(fixes, n * sizeof (*fixes));
		for (const void *mv = list_head(list); mv != NULL;
		    mv = list_next(list, mv)) {
			navaid_t *navaid = HTBL_VALUE_MULTI(mv);
			memcpy(fixes[i].name, navaid->ID, sizeof (navaid->ID));
			memcpy(fixes[i].icao_country_code,
			    navaid->icao_country_code,
			    sizeof (navaid->icao_country_code));
			fixes[i].pos = GEO3_TO_GEO2(navaid->pos);
			i++;
		}
	}

	ASSERT(i == n);
	*num_fixes = n;

	return (fixes);
}

static fix_t *
geofix(double lat, double lon, const char *namefmt, ...)
{
	fix_t *fix = calloc(sizeof (*fix), 1);
	va_list ap;
	int n;

	va_start(ap, namefmt);
	n = vsnprintf(fix->name, sizeof (fix->name), namefmt, ap);
	ASSERT(n > 0 && (unsigned)n < sizeof (fix->name));
	va_end(ap);
	fix->pos.lat = lat;
	fix->pos.lon = lon;
	return (fix);
}

#define	GET_MATCH(str, i, param, minlen, maxlen) \
	do { \
		CTASSERT(i < MAX_MATCHES); \
		regoff_t len = pmatch[i].rm_eo - pmatch[i].rm_so; \
		ASSERT(len >= minlen && len <= maxlen); \
		memcpy(param, &str[pmatch[i].rm_so], len); \
		param[len] = 0; \
	} while (0)

static bool_t
parse_latlon(const char *str, const regex_t *regex, int *latdeg,
    int *londeg)
{
#define	MAX_MATCHES	3
	regmatch_t	pmatch[MAX_MATCHES];
	char		latstr[3], lonstr[3];

	if (regexec(regex, str, MAX_MATCHES, pmatch, 0) != 0)
		return (B_FALSE);
	GET_MATCH(str, 1, latstr, 2, 2);
	GET_MATCH(str, 2, lonstr, 2, 2);
	*latdeg = atoi(latstr);
	*londeg = atoi(lonstr);

	return (B_TRUE);
#undef	MAX_MATCHES
}

/*
 * Decodes a waypoint name as entered by the user and looks up the fixes
 * it can describe.
 *
 * @param name Waypoint name as entered by the user. This must conform to
 *	one of the following formats (parsing order is as listed here):
 *	1) A 5-character geodetic coordinate combo, one of:
 *		a) "5010N" = N50 W010
 *		b) "50N10" = N50 W110
 *		c) "5010E" = N50 E010
 *		d) "50E10" = N50 E110
 *		e) "5010W" = S50 W010
 *		f) "50W10" = S50 W110
 *		g) "5010S" = S50 E010
 *		h) "50S10" = S50 E110
 *	2) A 7-character geodetic coordinate rounded to the nearest degree
 *	   (e.g. "N47W008").
 *	3) A 15-character geodetic coordinate down to fractional minutes
 *	   (e.g. "N4715.4W00803.4").
 *	4) A simple waypoint name of 1-5 alphanumeric upper case characters
 *	   (e.g. "ALPHA", "KM95S").
 *	5) TODO: reporting waypoints.
 *	5) A 9 to 12 character waypoint/radial/DME combo (e.g. "SEA330/10").
 *	6) A 9 to 17 character waypoint1/radial1/waypoint2/radial2 combo
 *	   (e.g. "SEA330/OLM020").
 *	7) TODO: along track waypoints.
 */
fix_t *
fms_wpt_name_decode(const char *name, fms_t *fms, size_t *num_fixes,
    bool_t *is_wpt_seq)
{
#define	MAX_MATCHES	7
	regmatch_t pmatch[MAX_MATCHES];
	int latdeg, londeg;

	*is_wpt_seq = B_FALSE;

	if (parse_latlon(name, fms->regex.geo_nw_blw100, &latdeg, &londeg)) {
		*num_fixes = 1;
		return (geofix(latdeg, -londeg, "%s", name));
	}
	if (parse_latlon(name, fms->regex.geo_nw_abv100, &latdeg, &londeg)) {
		*num_fixes = 1;
		return (geofix(latdeg, -londeg - 100, "%s", name));
	}
	if (parse_latlon(name, fms->regex.geo_ne_blw100, &latdeg, &londeg)) {
		*num_fixes = 1;
		return (geofix(latdeg, londeg, "%s", name));
	}
	if (parse_latlon(name, fms->regex.geo_ne_abv100, &latdeg, &londeg)) {
		*num_fixes = 1;
		return (geofix(latdeg, londeg + 100, "%s", name));
	}
	if (parse_latlon(name, fms->regex.geo_sw_blw100, &latdeg, &londeg)) {
		*num_fixes = 1;
		return (geofix(-latdeg, -londeg, "%s", name));
	}
	if (parse_latlon(name, fms->regex.geo_sw_abv100, &latdeg, &londeg)) {
		*num_fixes = 1;
		return (geofix(-latdeg, -londeg - 100, "%s", name));
	}
	if (parse_latlon(name, fms->regex.geo_se_blw100, &latdeg, &londeg)) {
		*num_fixes = 1;
		return (geofix(-latdeg, londeg, "%s", name));
	}
	if (parse_latlon(name, fms->regex.geo_se_abv100, &latdeg, &londeg)) {
		*num_fixes = 1;
		return (geofix(-latdeg, londeg + 100, "%s", name));
	}

	if (regexec(fms->regex.geo_long, name, 5, pmatch, 0) == 0) {
		char NS[2], latstr[3], EW[2], lonstr[4];
		double lat, lon;

		GET_MATCH(name, 1, NS, 1, 1);
		GET_MATCH(name, 2, latstr, 2, 2);
		GET_MATCH(name, 3, EW, 1, 1);
		GET_MATCH(name, 4, lonstr, 3, 3);
		lat = (NS[0] == 'N' ? atof(latstr) : -atof(latstr));
		lon = (EW[0] == 'E' ? atof(lonstr) : -atof(lonstr));

		*num_fixes = 1;
		return (geofix(lat, lon, "%s", name));
	}

	if (regexec(fms->regex.geo_detailed, name, 7, pmatch, 0) == 0) {
		char NS[2], latstr[3], latmin[5], EW[2], lonstr[4], lonmin[5];
		double lat, lon;

		GET_MATCH(name, 1, NS, 1, 1);
		GET_MATCH(name, 2, latstr, 2, 2);
		GET_MATCH(name, 3, latmin, 4, 4);
		GET_MATCH(name, 4, EW, 1, 1);
		GET_MATCH(name, 5, lonstr, 3, 3);
		GET_MATCH(name, 6, lonmin, 4, 4);
		if (NS[0] == 'N')
			lat = atof(latstr) + (atof(latmin) / 0.6);
		else
			lat = -atof(latstr) - (atof(latmin) / 0.6);
		if (EW[0] == 'E')
			lon = atof(lonstr) + (atof(lonmin) / 0.6);
		else
			lon = -atof(lonstr) - (atof(lonmin) / 0.6);

		*num_fixes = 1;
		return (geofix(lat, lon, "%s%s%s%s", NS, latstr, EW, lonstr));
	}

	/*
	 * This needs to come after geo pos fix scanning, because it covers
	 * them as well, but not all geographical fixes are in our wptdb.
	 */
	if (regexec(fms->regex.wptname, name, 2, pmatch, 0) == 0)
		return (fms_lookup_fix_by_name(name, fms, num_fixes));

	if (regexec(fms->regex.radial_dme, name, 4, pmatch, 0) == 0) {
		char wptname[6], radialstr[4], diststr[4];
		fix_t *fixes;
		size_t num;
		unsigned radial, dist;

		GET_MATCH(name, 1, wptname, 1, 5);
		GET_MATCH(name, 2, radialstr, 3, 3);
		GET_MATCH(name, 3, diststr, 1, 3);
		radial = atoi(radialstr);
		dist = atoi(diststr);
		if (!is_valid_hdg(radial) || dist == 0)
			goto errout;

		fixes = fms_lookup_fix_by_name(wptname, fms, &num);
		if (fixes == NULL)
			goto errout;

		/* go through all results, displace & rewrite their names */
		for (size_t i = 0; i < num; i++) {
			VERIFY(snprintf(fixes[i].name, sizeof (fixes[i].name),
			    "%s%02d", wptname, fms->wpt_seq_num) <
			    (long)sizeof (fixes[i].name));
			fixes[i].pos = geo_displace_mag(&wgs84, fms->navdb->wmm,
			    fixes[i].pos, radial, dist);
		}
		fms->wpt_seq_num++;

		*num_fixes = num;
		return (fixes);
	}

	if (regexec(fms->regex.radial_isect, name, 5, pmatch, 0) == 0) {
		char wpt1name[6], radial1str[4], wpt2name[6], radial2str[4];
		unsigned radial1, radial2;
		fix_t *tmp_fixes1, *tmp_fixes2;
		size_t num_fixes1, num_fixes2, num = 0;
		fix_t *fixes;

		GET_MATCH(name, 1, wpt1name, 1, 5);
		GET_MATCH(name, 2, radial1str, 3, 3);
		GET_MATCH(name, 3, wpt2name, 1, 5);
		GET_MATCH(name, 4, radial2str, 3, 3);
		radial1 = atoi(radial1str);
		radial2 = atoi(radial2str);
		if (!is_valid_hdg(radial1) || !is_valid_hdg(radial2) ||
		    radial1 == radial2)
			goto errout;

		tmp_fixes1 = fms_lookup_fix_by_name(wpt1name, fms, &num_fixes1);
		if (tmp_fixes1 == NULL)
			goto errout;
		tmp_fixes2 = fms_lookup_fix_by_name(wpt2name, fms, &num_fixes2);
		if (tmp_fixes2 == NULL) {
			free (tmp_fixes1);
			goto errout;
		}

#define	WPT_ISECT_MAXRNG	1000000
		fixes = calloc(sizeof (*fixes), num_fixes1 * num_fixes2);
		for (size_t i = 0; i < num_fixes1; i++) {
			vect3_t pos1_v = geo2ecef(GEO2_TO_GEO3(
			    tmp_fixes1[i].pos, 0), &wgs84);
			for (size_t j = 0; j < num_fixes2; j++) {
				vect3_t pos2_v = geo2ecef(GEO2_TO_GEO3(
				    tmp_fixes2[j].pos, 0), &wgs84);

				if (vect3_abs(vect3_sub(pos2_v, pos1_v)) >
				    WPT_ISECT_MAXRNG)
					continue;
				fixes[num].pos = geo_mag_radial_isect(&wgs84,
				    fms->navdb->wmm, tmp_fixes1[i].pos, radial1,
				    tmp_fixes2[j].pos, radial2);
				VERIFY(snprintf(fixes[num].name,
				    sizeof (fixes[num].name), "%s%02d",
				    wpt1name, fms->wpt_seq_num) <
				    (long)sizeof (fixes[num].name));
				num++;
			}
		}

		free(tmp_fixes1);
		free(tmp_fixes2);

		if (num == 0) {
			free(fixes);
			goto errout;
		}
		fms->wpt_seq_num++;

		*num_fixes = num;
		return (fixes);
	}
errout:
	*num_fixes = 0;
	return (NULL);
#undef	MAX_MATCHES
}
