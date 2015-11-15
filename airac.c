#include <stdlib.h>
#include <string.h>
#include <assert.h>
#include <stdio.h>
#include <math.h>

#include "airac.h"
#include "helpers.h"

/* Maximum allowable runway length */
#define	MAX_RWY_LEN	100000

/* Maximum allowable glidepath angle */
#define	GP_MAX_ANGLE	10.0

static int
parse_arpt_line(char *line, airport_t *arpt)
{
	size_t num_comps;
	char **comps = explode_line(line, ",", &num_comps);

	/* Check this is an airport line and it's the one we're looking for */
	if (num_comps != 10 || strcmp(comps[0], "A") != 0 ||
	    strcmp(comps[1], arpt->icao) != 0) {
		goto errout;
	}
	(void) strlcpy(arpt->name, comps[2], sizeof (arpt->name));
	if (!geo_pos_3d_from_str(comps[3], comps[4], comps[5],
	    &arpt->refpt_pos))
		goto errout;
	if (/* transition altitude must be uint && a valid altitude */
	    sscanf(comps[6], "%u", &arpt->TA) != 1 || !is_valid_alt(arpt->TA) ||
	    /* transition level must be uint && a valid altitude */
	    sscanf(comps[7], "%u", &arpt->TL) != 1 || !is_valid_alt(arpt->TL) ||
	    /* longest rwy must be non-zero uint && <= MAX_RWY_LEN */
	    sscanf(comps[8], "%u", &arpt->longest_rwy) != 1 ||
	    arpt->longest_rwy == 0 || arpt->longest_rwy > MAX_RWY_LEN)
		goto errout;

	free(comps);
	return (1);
errout:
	free(comps);
	return (0);
}

static int
parse_rwy_line(char *line, runway_t *rwy)
{
	size_t num_comps;
	char **comps = explode_line(line, ",", &num_comps);

	if (strcmp(comps[0], "R") != 1)
		goto errout;
	(void) strlcpy(rwy->rwy_ID, comps[1], sizeof (rwy->rwy_ID));
	if (/* hdg must be uint && valid magnetic heading */
	    sscanf(comps[2], "%u", &rwy->hdg) != 1 ||
	    !is_valid_hdg(rwy->hdg) ||
	    /* length must be non-zero uint && <= MAX_RWY_LEN */
	    sscanf(comps[3], "%u", &rwy->length) != 1 || rwy->length == 0 ||
	    rwy->length > MAX_RWY_LEN ||
	    /* width must be non-zero uint */
	    sscanf(comps[4], "%u", &rwy->width) != 1 || rwy->width == 0 ||
	    /* loc_avail must be 0 or 1 */
	    sscanf(comps[5], "%d", &rwy->loc_avail) != 1 ||
	    (rwy->loc_avail != 0 && rwy->loc_avail != 1) ||
	    /* loc_freq must be real && if loc_avail, valid LOC frequency */
	    sscanf(comps[6], "%lf", &rwy->loc_freq) != 1 ||
	    (rwy->loc_avail && !is_valid_loc_freq(rwy->loc_freq)) ||
	    /* loc_fcrs must be real && if loc_avail, valid magnetic heading */
	    sscanf(comps[7], "%u", &rwy->loc_fcrs) != 1 ||
	    (rwy->loc_avail && !is_valid_hdg(rwy->loc_fcrs)) ||
	    /* threshold must have valid position */
	    !geo_pos_3d_from_str(comps[8], comps[9], comps[10],
	    &rwy->thr_pos) ||
	    /* GP angle must be between 0.0 and GP_MAX_ANGLE */
	    sscanf(comps[11], "%lf", &rwy->gp_angle) != 1 ||
	    rwy->gp_angle < 0.0 || rwy->gp_angle > GP_MAX_ANGLE)
		goto errout;

	free(comps);
	return (1);
errout:
	free(comps);
	return (0);
}

airport_t *
airport_parse(const char *arpt_icao, const char *navdata_dir)
{
	airport_t	*arpt;
	FILE		*arpt_fp;
	char		*arpt_fname;
	ssize_t		line_len = 0, line_cap = 0;
	char		*line = NULL;
	int		done = 0;

	arpt = calloc(sizeof (*arpt), 1);
	assert(strlen(arpt_icao) == 4);
	strcpy(arpt->icao, arpt_icao);

	/* Open Airports.txt and locate the starting line */
	arpt_fname = malloc(strlen(navdata_dir) + strlen("/Airports.txt") + 1);
	arpt_fname[0] = 0;
	(void) strcat(arpt_fname, navdata_dir);
	(void) strcat(arpt_fname, "/Airports.txt");
	arpt_fp = fopen(arpt_fname, "r");
	if (arpt_fp == NULL)
		goto errout;

	/* Locate airport start line & parse it */
	while ((line_len = getline(&line, (size_t *)&line_cap, arpt_fp))
	    != -1 && !done) {
		strip_newline(line, line_len);
		if (parse_arpt_line(line, arpt)) {
			done = 1;
			break;
		}
	}
	if (done == 0) {
		/* error reading/locating airport */
		goto errout;
	}

	/* airport found, read non-empty runway lines */
	while ((line_len = getline(&line, (size_t *)&line_cap, arpt_fp)) > 0) {
		strip_newline(line, line_len);
		if (strlen(line) == 0)
			break;
		arpt->num_rwys++;
		arpt->rwys = realloc(arpt->rwys, sizeof (*arpt->rwys) *
		    arpt->num_rwys);
		memset(&arpt->rwys[arpt->num_rwys - 1], 0, sizeof (runway_t));
		if (!parse_rwy_line(line, &arpt->rwys[arpt->num_rwys - 1]))
			goto errout;
	}

	/* airport must have at least one runway */
	if (arpt->num_rwys == 0)
		goto errout;

	free(arpt_fname);
	(void) fclose(arpt_fp);

	return (arpt);

errout:
	if (arpt)
		airport_free(arpt);
	free(arpt_fname);
	if (arpt_fp)
		(void) fclose(arpt_fp);
	free(line);
	return (NULL);
}

/*
 * Locates and returns a runway structure at an airport based on runway ID.
 */
const runway_t *
airport_find_rwy_by_ID(const airport_t *arpt, const char *rwy_ID)
{
	for (unsigned i = 0; i < arpt->num_rwys; i++) {
		if (strcmp(arpt->rwys[i].rwy_ID, rwy_ID) == 0)
			return (&arpt->rwys[i]);
	}
	return (NULL);
}

/*
 * Returns the position of a gate at an airport based on gate ID.
 */
geo_pos_2d_t
airport_find_gate_pos(const airport_t *arpt, const char *gate_ID)
{
	for (unsigned i = 0; i < arpt->num_gates; i++) {
		if (strcmp(arpt->gates[i].name, gate_ID) == 0)
			return arpt->gates[i].pos;
	}
	return (null_2d_pos);
}
