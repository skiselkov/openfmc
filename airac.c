/*
 * CDDL HEADER START
 *
 * The contents of this file are subject to the terms of the
 * Common Development and Distribution License, Version 1.0 only
 * (the "License").  You may not use this file except in compliance
 * with the License.
 *
 * You can obtain a copy of the license in the file COPYING
 * or http://www.opensource.org/licenses/CDDL-1.0.
 * See the License for the specific language governing permissions
 * and limitations under the License.
 *
 * When distributing Covered Code, include this CDDL HEADER in each
 * file and include the License file COPYING.
 * If applicable, add the following below this CDDL HEADER, with the
 * fields enclosed by brackets "[]" replaced with your own identifying
 * information: Portions Copyright [yyyy] [name of copyright owner]
 *
 * CDDL HEADER END
 */
/*
 * Copyright 2015 Saso Kiselkov. All rights reserved.
 */

#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include <math.h>
#include <errno.h>

#include "geom.h"
#include "airac.h"
#include "helpers.h"
#include "log.h"

/* Maximum allowable runway length & width (in feet) */
#define	MAX_RWY_LEN	250000
/* Maximum allowable runway glidepath angle */
#define	GP_MAX_ANGLE	10.0

/* Allocation limits */
#define	MAX_PROC_SEGS	100
#define	MAX_AWY_SEGS	1000
#define	MAX_NUM_AWYS	100000
#define	MAX_NUM_WPTS	1000000
#define	MAX_NUM_NAVAIDS	1000000

/*
 * Copies src to dst (for which sizeof must give its real size) and checks
 * for overflow.
 */
#define	STRLCPY_CHECK(dst, src) \
	(strlcpy(dst, src, sizeof (dst)) <= sizeof (dst))
#define	STRLCPY_CHECK_ERROUT(dst, src) \
	do { \
		if (!STRLCPY_CHECK(dst, src)) { \
			openfmc_log(OPENFMC_LOG_ERR, "Parsing error: input " \
			    "string too long."); \
			goto errout; \
		} \
	} while (0)

const wpt_t null_wpt = {
	.name = "\000\000\000\000\000\000\000",
	.icao_country_code = "\000\000",
	.pos = NULL_GEO_POS2
};

typedef struct {
	char	**result;
	size_t	*result_sz;
	char	scratch[NAV_NAME_LEN];
} db_dump_info_t;

static const char *navaid_type_name(navaid_type_t t)
{
	ASSERT(t <= NAVAID_TYPE_ANY);
	switch (t) {
	case NAVAID_TYPE_VOR:
		return "VOR";
	case NAVAID_TYPE_VORDME:
		return "VORDME";
	case NAVAID_TYPE_LOC:
		return "LOC";
	case NAVAID_TYPE_LOCDME:
		return "LOCDME";
	case NAVAID_TYPE_NDB:
		return "NDB";
	case NAVAID_TYPE_TACAN:
		return "TACAN";
	case NAVAID_TYPE_UNKNOWN:
		return "UNKNOWN";
	case NAVAID_TYPE_ANY:
		return "(any)";
	case NAVAID_TYPE_ANY_VOR:
		return "(VOR/VORDME)";
	case NAVAID_TYPE_ANY_LOC:
		return "(LOC/LOCDME)";
	default:
		return "(non-standard combo)";
	}
}

/* The order in this array must follow navproc_type_t */
static const char *navproc_type_to_str[NAVPROC_TYPES] = {
	"SID", "SIDCM", "SIDTR",
	"STAR", "STARCM", "STARTR",
	"FINALTR", "FINAL"
};

static void dump_AF_seg(char **result, size_t *result_sz,
    const navproc_seg_t *seg);
static void dump_CA_seg(char **result, size_t *result_sz,
    const navproc_seg_t *seg);
static void dump_CA_seg(char **result, size_t *result_sz,
    const navproc_seg_t *seg);
static void dump_CD_seg(char **result, size_t *result_sz,
    const navproc_seg_t *seg);
static void dump_CF_seg(char **result, size_t *result_sz,
    const navproc_seg_t *seg);
static void dump_CI_CR_seg(char **result, size_t *result_sz,
    const navproc_seg_t *seg);
static void dump_DF_TF_seg(char **result, size_t *result_sz,
    const navproc_seg_t *seg);
static void dump_FA_seg(char **result, size_t *result_sz,
    const navproc_seg_t *seg);
static void dump_FC_seg(char **result, size_t *result_sz,
    const navproc_seg_t *seg);
static void dump_FD_seg(char **result, size_t *result_sz,
    const navproc_seg_t *seg);
static void dump_FM_seg(char **result, size_t *result_sz,
    const navproc_seg_t *seg);
static void dump_HA_HF_HM_seg(char **result, size_t *result_sz,
    const navproc_seg_t *seg);
static void dump_IF_seg(char **result, size_t *result_sz,
    const navproc_seg_t *seg);
static void dump_PI_seg(char **result, size_t *result_sz,
    const navproc_seg_t *seg);
static void dump_RF_seg(char **result, size_t *result_sz,
    const navproc_seg_t *seg);
static void dump_VA_seg(char **result, size_t *result_sz,
    const navproc_seg_t *seg);
static void dump_VD_seg(char **result, size_t *result_sz,
    const navproc_seg_t *seg);
static void dump_VI_VM_VR_seg(char **result, size_t *result_sz,
    const navproc_seg_t *seg);

typedef void (*navproc_seg_dump_func_t)(char **, size_t *,
    const navproc_seg_t *);
/* The order in this array must follow navproc_seg_type_t */
static navproc_seg_dump_func_t navproc_seg_dump_funcs[NAVPROC_SEG_TYPES] = {
	dump_AF_seg,		/* NAVPROC_SEG_TYPE_ARC_TO_FIX */
	dump_CA_seg,		/* NAVPROC_SEG_TYPE_CRS_TO_ALT */
	dump_CD_seg,		/* NAVPROC_SEG_TYPE_CRS_TO_DME */
	dump_CF_seg,		/* NAVPROC_SEG_TYPE_CRS_TO_FIX */
	dump_CI_CR_seg,		/* NAVPROC_SEG_TYPE_CRS_TO_INTCP */
	dump_CI_CR_seg,		/* NAVPROC_SEG_TYPE_CRS_TO_RADIAL */
	dump_DF_TF_seg,		/* NAVPROC_SEG_TYPE_DIR_TO_FIX */
	dump_FA_seg,		/* NAVPROC_SEG_TYPE_FIX_TO_ALT */
	dump_FC_seg,		/* NAVPROC_SEG_TYPE_FIX_TO_DIST */
	dump_FD_seg,		/* NAVPROC_SEG_TYPE_FIX_TO_DME */
	dump_FM_seg,		/* NAVPROC_SEG_TYPE_FIX_TO_MANUAL */
	dump_HA_HF_HM_seg,	/* NAVPROC_SEG_TYPE_HOLD_TO_ALT */
	dump_HA_HF_HM_seg,	/* NAVPROC_SEG_TYPE_HOLD_TO_FIX */
	dump_HA_HF_HM_seg,	/* NAVPROC_SEG_TYPE_HOLD_TO_MANUAL */
	dump_IF_seg,		/* NAVPROC_SEG_TYPE_INIT_FIX */
	dump_PI_seg,		/* NAVPROC_SEG_TYPE_PROC_TURN */
	dump_RF_seg,		/* NAVPROC_SEG_TYPE_RADIUS_ARC_TO_FIX */
	dump_DF_TF_seg,		/* NAVPROC_SEG_TYPE_TRK_TO_FIX */
	dump_VA_seg,		/* NAVPROC_SEG_TYPE_HDG_TO_ALT */
	dump_VD_seg,		/* NAVPROC_SEG_TYPE_HDG_TO_DME */
	dump_VI_VM_VR_seg,	/* NAVPROC_SEG_TYPE_HDG_TO_INTCP */
	dump_VI_VM_VR_seg,	/* NAVPROC_SEG_TYPE_HDG_TO_MANUAL */
	dump_VI_VM_VR_seg	/* NAVPROC_SEG_TYPE_HDG_TO_RADIAL */
};

/* The order in this array must follow navproc_final_t */
static const char *navproc_final_types_to_str[NAVPROC_FINAL_TYPES] = {
	"ILS",	"VOR",	"NDB",	"RNAV",	"LDA"
};

static inline bool_t
is_valid_turn(int turn)
{
	return (turn >= TURN_ANY && turn <= TURN_RIGHT);
}

static inline const char *
dump_turn(turn_t turn)
{
	switch (turn) {
	case TURN_ANY:
		return "any";
	case TURN_LEFT:
		return "left";
	case TURN_RIGHT:
		return "right";
	default:
		assert(0);
	}
}

/*
 * Parses one airway line starting with 'A,' from ATS.txt.
 */
static bool_t
parse_airway_line(const char *line, airway_t *awy, const char *filename,
    size_t line_num)
{
	char	line_copy[128];
	char	*comps[3];

	(void) strlcpy(line_copy, line, sizeof (line_copy));
	if (explode_line(line_copy, ',', comps, 3) != 3) {
		openfmc_log(OPENFMC_LOG_ERR, "%s:%lu: error parsing airway "
		    "line: invalid number of columns, wanted 3.", filename,
		    line_num);
		return (B_FALSE);
	}
	if (strcmp(comps[0], "A") != 0) {
		openfmc_log(OPENFMC_LOG_ERR, "%s:%lu: error parsing airway "
		    "line: wanted line type 'A', got '%s'.", filename,
		    line_num, comps[0]);
		return (B_FALSE);
	}
	if (strlen(comps[1]) > sizeof (awy->name) - 1) {
		openfmc_log(OPENFMC_LOG_ERR, "%s:%lu: error parsing airway "
		    "line: airway name '%s' too long (max allowed %lu chars).",
		    filename, line_num, comps[1], sizeof (awy->name) - 1);
		return (B_FALSE);
	}
	(void) strlcpy(awy->name, comps[1], sizeof (awy->name));
	awy->num_segs = atoi(comps[2]);
	if (awy->num_segs == 0 || awy->num_segs > MAX_AWY_SEGS) {
		openfmc_log(OPENFMC_LOG_ERR, "%s:%lu: error parsing airway "
		    "line: invalid number of segments \"%s\".", filename,
		    line_num, comps[2]);
		return (B_FALSE);
	}

	return (B_TRUE);
}

static bool_t
parse_airway_seg_line(const char *line, airway_seg_t *seg)
{
	char	line_copy[128];
	char	*comps[10];

	STRLCPY_CHECK_ERROUT(line_copy, line);
	if (explode_line(line_copy, ',', comps, 10) != 10) {
		openfmc_log(OPENFMC_LOG_ERR, "Error parsing airway segment: "
		    "invalid number of cols, wanted 10.");
		goto errout;
	}
	if (strcmp(comps[0], "S") != 0) {
		openfmc_log(OPENFMC_LOG_ERR, "Error parsing airway segment: "
		    "wanted line type 'S', got '%s'.", comps[0]);
		goto errout;
	}
	STRLCPY_CHECK_ERROUT(seg->endpt[0].name, comps[1]);
	STRLCPY_CHECK_ERROUT(seg->endpt[1].name, comps[4]);
	if (!geo_pos2_from_str(comps[2], comps[3], &seg->endpt[0].pos) ||
	    !geo_pos2_from_str(comps[5], comps[6], &seg->endpt[1].pos)) {
		openfmc_log(OPENFMC_LOG_ERR, "Error parsing airway segment: "
		    "segment wpt positions invalid.");
		goto errout;
	}

	return (B_TRUE);
errout:
	openfmc_log(OPENFMC_LOG_ERR, "Offending line was: \"%s\".", line);
	return (B_FALSE);
}

static bool_t
parse_airway_segs(FILE *fp, airway_t *awy, const char *filename,
    size_t *line_num)
{
	char	*line = NULL;
	ssize_t	line_len = 0;
	size_t	line_cap = 0;
	size_t	nsegs;

	ASSERT(awy->segs == NULL);
	awy->segs = calloc(sizeof (airway_seg_t), awy->num_segs);

	for (nsegs = 0; nsegs < awy->num_segs &&
	    (line_len = parser_get_next_line(fp, &line, &line_cap,
	    line_num)) != -1; nsegs++) {
		if (!parse_airway_seg_line(line, &awy->segs[nsegs]))
			goto errout;

		/* Check that adjacent airway segments are connected */
		if (nsegs > 0 && memcmp(&awy->segs[nsegs - 1].endpt[1],
		    &awy->segs[nsegs].endpt[0], sizeof (wpt_t)) != 0) {
			openfmc_log(OPENFMC_LOG_ERR, "%s:%lu: error parsing "
			    "airway \"%s\": segment #%lu (wpt %s) and #%lu "
			    "(wpt %s) aren't connected.", filename, *line_num,
			    awy->name, nsegs - 1,
			    awy->segs[nsegs - 1].endpt[1].name, nsegs,
			    awy->segs[nsegs].endpt[0].name);
			goto errout;
		}
	}
	if (nsegs != awy->num_segs) {
		openfmc_log(OPENFMC_LOG_ERR, "%s:%lu: error parsing airway "
		    "\"%s\": expected %u segments, but only %lu 'S' lines "
		    "followed.", filename, *line_num, awy->name,
		    awy->num_segs, nsegs);
		goto errout;
	}

	free(line);
	return (B_TRUE);
errout:
	free(line);
	free(awy->segs);
	awy->segs = NULL;

	return (B_FALSE);
}

static void
airway_free(airway_t *awy)
{
	free(awy->segs);
	free(awy);
}

airway_db_t *
airway_db_open(const char *navdata_dir, size_t num_waypoints)
{
	airway_db_t	*db = NULL;
	FILE		*ats_fp = NULL;
	char		*ats_fname = NULL;
	ssize_t		line_len = 0;
	size_t		line_cap = 0, line_num = 0;
	char		*line = NULL;
	uint64_t	num_airways = 0;
	airway_t	*awy = NULL;

	/* Open ATS.txt */
	ats_fname = malloc(strlen(navdata_dir) + strlen(PATHSEP "ATS.txt") + 1);
	sprintf(ats_fname, "%s" PATHSEP "ATS.txt", navdata_dir);
	ats_fp = fopen(ats_fname, "r");
	if (ats_fp == NULL) {
		openfmc_log(OPENFMC_LOG_ERR, "Can't open %s: %s", ats_fname,
		    strerror(errno));
		goto errout;
	}

	/* Count number of lines starting with "A," to size up hash table */
	while ((line_len = parser_get_next_line(ats_fp, &line, &line_cap,
	    &line_num)) != -1) {
		if (line_len > 3 && line[0] == 'A' && line[1] == ',')
			num_airways++;
	}
	rewind(ats_fp);
	line_num = 0;
	if (num_airways == 0 || num_airways > MAX_NUM_AWYS) {
		openfmc_log(OPENFMC_LOG_ERR, "Error parsing %s: invalid "
		    "number of airways found: %llu", ats_fname, num_airways);
		goto errout;
	}

	db = calloc(sizeof (*db), 1);
	if (!db)
		goto errout;
	htbl_create(&db->by_awy_name, num_airways, NAV_NAME_LEN, B_TRUE);
	htbl_create(&db->by_fix_name, num_waypoints, NAV_NAME_LEN, B_TRUE);

	while ((line_len = parser_get_next_line(ats_fp, &line, &line_cap,
	    &line_num)) != -1) {
		if (line_len == 0)
			continue;
		awy = calloc(sizeof (*awy), 1);
		if (!awy)
			goto errout;
		if (!parse_airway_line(line, awy, ats_fname, line_num) ||
		    !parse_airway_segs(ats_fp, awy, ats_fname, &line_num)) {
			goto errout;
		}
		htbl_set(&db->by_awy_name, awy->name, awy);
		for (size_t i = 0; i < awy->num_segs; i++)
			htbl_set(&db->by_fix_name, awy->segs[i].endpt[0].name,
			    awy);
		if (awy->num_segs > 0) {
			htbl_set(&db->by_fix_name,
			    awy->segs[awy->num_segs - 1].endpt[1].name, awy);
		}
		awy = NULL;
	}

	if (ats_fp)
		fclose(ats_fp);
	free(ats_fname);
	if (awy)
		airway_free(awy);
	return (db);
errout:
	if (db)
		airway_db_close(db);
	if (ats_fp)
		fclose(ats_fp);
	free(ats_fname);
	if (awy)
		airway_free(awy);
	return (NULL);
}

void
airway_db_close(airway_db_t *db)
{
	htbl_empty(&db->by_awy_name,
	    (void (*)(void *, void *))airway_free, NULL);
	htbl_empty(&db->by_fix_name, NULL, NULL);
	htbl_destroy(&db->by_awy_name);
	htbl_destroy(&db->by_fix_name);
	free(db);
}

static void
airway_db_dump_awy(const void *k, const airway_t *awy, db_dump_info_t *info)
{
	UNUSED(k);
	append_format(info->result, info->result_sz,
	    "  %s (%u):\n"
	    "    wpt 1        lat         lon     wpt 2        lat "
	    "        lon\n"
	    "    ----- ---------- -----------     ----- ---------- "
	    "-----------\n", awy->name, awy->num_segs);
	for (size_t i = 0; i < awy->num_segs; i++) {
		const airway_seg_t *seg = &awy->segs[i];
		append_format(info->result, info->result_sz,
		    "    %5s %10.6lf %11.6lf  -  %5s %10.6lf %11.6lf\n",
		    seg->endpt[0].name, seg->endpt[0].pos.lat,
		    seg->endpt[0].pos.lon, seg->endpt[1].name,
		    seg->endpt[1].pos.lat, seg->endpt[1].pos.lon);
	}
	append_format(info->result, info->result_sz, "\n");
}

static void
airway_db_dump_fix(const void *k, const airway_t *awy, db_dump_info_t *info)
{
	const char *key = k;

	if (strcmp(k, info->scratch) != 0) {
		append_format(info->result, info->result_sz,
		    "\n"
		    "  %s\n"
		    "    %s", key, awy->name);
		strcpy(info->scratch, k);
	} else {
		append_format(info->result, info->result_sz, " %s", awy->name);
	}
}

char *
airway_db_dump(const airway_db_t *db, bool_t by_awy_name)
{
	char		*result = NULL;
	size_t		result_sz = 0;
	db_dump_info_t	info = { .result = &result, .result_sz = &result_sz };

	info.scratch[0] = 0;
	if (by_awy_name) {
		append_format(&result, &result_sz, "Airways (%lu):\n",
		    htbl_count(&db->by_awy_name));
		htbl_foreach(&db->by_awy_name,
		    (void (*)(const void *, void *, void*))airway_db_dump_awy,
		    &info);
	} else {
		append_format(&result, &result_sz, "Fixes (%lu):\n",
		    htbl_count(&db->by_fix_name));
		htbl_foreach(&db->by_fix_name,
		    (void (*)(const void *, void *, void*))airway_db_dump_fix,
		    &info);
		append_format(&result, &result_sz, "\n");
	}
	return (result);
}

/*
 * Performs an airway DB lookup based on 3 lookup and returns an airway
 * matching:
 *	1) awyname: Airway name. This argument is mandatory. If the airway
 *	   doesn't exist, returns NULL.
 *	2) start_wpt: Starting section wpt. This is optional.
 *	   In addition to the airway name matching, this will also check
 *	   that the airway contains this exact wpt as the starting
 *	   wpt in one of its segments.
 *	3) end_wpt_name: Ending section wpt name. This is optional. In
 *	   addition to conditions #1 and #2 (if not NULL), also checks that
 *	   the section start wpt is followed by the appropriate airway
 *	   segment end wpt.
 * Using all three arguments you can choose the direction of bidirectional
 * airways (which appear in the database as two airway_t's with the same
 * name but wpt order reversed).
 */
const airway_t *
airway_db_lookup(const airway_db_t *db, const char *awyname,
    const wpt_t *start_wpt, const char *end_wpt_name, const wpt_t **endfixpp)
{
	const list_t *awy_list;

	/* null fixes will never match anything, so don't even bother */
	if ((start_wpt && IS_NULL_WPT(start_wpt)) ||
	    (end_wpt_name != NULL && *end_wpt_name == 0)) {
		if (endfixpp != NULL)
			*endfixpp = NULL;
		return (NULL);
	}

	ASSERT(awyname != NULL);
	awy_list = htbl_lookup_multi(&db->by_awy_name, awyname);
	if (awy_list == NULL) {
		if (endfixpp != NULL)
			*endfixpp = NULL;
		return (NULL);
	}
	for (const void *mv = list_head(awy_list); mv != NULL;
	    mv = list_next(awy_list, mv)) {
		const airway_t		*awy = HTBL_VALUE_MULTI(mv);
		unsigned		i = 0;

		ASSERT(awy != NULL);
		ASSERT(strcmp(awy->name, awyname) == 0);
		if (start_wpt != NULL) {
			/* Look for the start wpt */
			for (; i < awy->num_segs; i++) {
				if (WPT_EQ(&awy->segs[i].endpt[0], start_wpt))
					break;
			}
			if (i == awy->num_segs)
				continue;
		}
		if (end_wpt_name != NULL) {
			/* Look for the end wpt */
			for (; i < awy->num_segs; i++) {
				if (strcmp(awy->segs[i].endpt[1].name,
				    end_wpt_name) == 0)
					break;
			}
			if (i == awy->num_segs)
				continue;
			if (endfixpp)
				*endfixpp = &awy->segs[i].endpt[1];
		} else if (endfixpp) {
			*endfixpp = NULL;
		}
		/* Airway matches and start&end fixes follow each other */
		return (awy);
	}
	if (endfixpp != NULL)
		*endfixpp = NULL;

	return (NULL);
}

/*
 * Given a 1st airway name, 1st airway section start wpt name and a 2nd airway
 * name, looks for an airway segment end wpt following the start wpt on airway
 * #1 that is also an airway segment start wpt on airway #2.
 */
const wpt_t *
airway_db_lookup_awy_intersection(const airway_db_t *db, const char *awy1_name,
    const char *awy1_start_wpt_name, const char *awy2_name)
{
	const list_t *awy1_list;

	if (awy1_start_wpt_name != NULL && *awy1_start_wpt_name == 0)
		return (NULL);

	ASSERT(awy1_name != NULL);
	ASSERT(awy1_start_wpt_name != NULL);
	awy1_list = htbl_lookup_multi(&db->by_awy_name, awy1_name);
	if (awy1_list == NULL)
		return (NULL);
	for (const void *mv1 = list_head(awy1_list); mv1 != NULL;
	    mv1 = list_next(awy1_list, mv1)) {
		const airway_t		*awy1 = HTBL_VALUE_MULTI(mv1);
		unsigned		i = 0;

		ASSERT(awy1 != NULL);
		ASSERT(strcmp(awy1->name, awy1_name) == 0);
		/* Look for the start wpt */
		for (i = 0; i < awy1->num_segs; i++) {
			if (strcmp(awy1->segs[i].endpt[0].name,
			    awy1_start_wpt_name) == 0)
				break;
		}
		if (i == awy1->num_segs)
			continue;

		/*
		 * Now try and lookup airways with awy2_name and starting
		 * at the end fixes of segments following this segment.
		 */
		for (; i < awy1->num_segs; i++) {
			if (airway_db_lookup(db, awy2_name,
			    &awy1->segs[i].endpt[1], NULL, NULL) != NULL) {
				return (&awy1->segs[i].endpt[1]);
			}
		}
	}

	return (NULL);
}

/*
 * Checks if `wpt' is a starting wpt on any segment of airway `awy'.
 */
bool_t
airway_db_fiwpt_on_awy(const airway_db_t *db, const wpt_t *wpt,
    const char *awyname)
{
	const list_t *awy_list;

	ASSERT(wpt != NULL);
	ASSERT(awyname != NULL);
	awy_list = htbl_lookup_multi(&db->by_fix_name, wpt->name);
	if (awy_list == NULL)
		return (B_FALSE);
	for (const void *mv = list_head(awy_list); mv != NULL;
	    mv = list_next(awy_list, mv)) {
		airway_t *awy = HTBL_VALUE_MULTI(mv);

		if (strcmp(awy->name, awyname) != 0)
			continue;
		/* look for the exact start wpt (incl geo pos) */
		for (unsigned i = 0; i < awy->num_segs; i++) {
			if (memcmp(&awy->segs[i].endpt[0], wpt, sizeof (*wpt))
			    == 0)
				return (B_TRUE);
		}
	}
	return (B_FALSE);
}

static bool_t
parse_waypoint_line(const char *line, wpt_t *wpt)
{
	char	line_copy[128];
	char	*comps[4];

	STRLCPY_CHECK_ERROUT(line_copy, line);
	if (explode_line(line_copy, ',', comps, 4) != 4) {
		openfmc_log(OPENFMC_LOG_ERR, "Error parsing waypoint: "
		    "line contains invalid number of columns, wanted 4.");
		goto errout;
	}
	STRLCPY_CHECK_ERROUT(wpt->name, comps[0]);
	if (!geo_pos2_from_str(comps[1], comps[2], &wpt->pos)) {
		openfmc_log(OPENFMC_LOG_ERR, "Error parsing waypoint: "
		    "lat/lon position invalid.");
		goto errout;
	}
	STRLCPY_CHECK_ERROUT(wpt->icao_country_code, comps[3]);

	return (B_TRUE);
errout:
	openfmc_log(OPENFMC_LOG_ERR, "Offending line was: \"%s\".", line);
	return (B_FALSE);
}

waypoint_db_t *
waypoint_db_open(const char *navdata_dir)
{
	waypoint_db_t	*db = NULL;
	FILE		*wpts_fp = NULL;
	char		*wpts_fname = NULL;
	ssize_t		line_len = 0;
	size_t		line_cap = 0, line_num = 0;
	char		*line = NULL;
	uint64_t	num_wpts = 0;
	wpt_t		*wpt = NULL;

	/* Open Waypoints.txt */
	wpts_fname = malloc(strlen(navdata_dir) +
	    strlen(PATHSEP "Waypoints.txt") + 1);
	sprintf(wpts_fname, "%s" PATHSEP "Waypoints.txt", navdata_dir);
	wpts_fp = fopen(wpts_fname, "r");
	if (wpts_fp == NULL) {
		openfmc_log(OPENFMC_LOG_ERR, "Can't open %s: %s",
		    wpts_fname, strerror(errno));
		goto errout;
	}

	/*
	 * Count number of lines NOT starting with " " to size up hash
	 * table. We don't care about non-named (coordinate) waypoints, we
	 * can construct those on the fly.
	 */
	while ((line_len = parser_get_next_line(wpts_fp, &line, &line_cap,
	    &line_num)) != -1) {
		if (line_len > 1 && line[0] != ',')
			num_wpts++;
	}
	rewind(wpts_fp);
	line_num = 0;
	if (num_wpts == 0 || num_wpts > MAX_NUM_WPTS) {
		openfmc_log(OPENFMC_LOG_ERR, "Error parsing %s: invalid "
		    "number of waypoints found: %llu", wpts_fname, num_wpts);
		goto errout;
	}

	db = calloc(sizeof (*db), 1);
	if (!db)
		goto errout;
	htbl_create(&db->by_name, num_wpts, NAV_NAME_LEN, B_TRUE);

	while ((line_len = parser_get_next_line(wpts_fp, &line, &line_cap,
	    &line_num)) != -1) {
		if (line_len == 0 || *line == ',')
			continue;
		wpt = calloc(sizeof (*wpt), 1);
		if (!wpt)
			goto errout;
		if (!parse_waypoint_line(line, wpt))
			goto errout;
		htbl_set(&db->by_name, wpt->name, wpt);
		wpt = NULL;
	}

	if (wpts_fp)
		fclose(wpts_fp);
	free(wpts_fname);
	if (wpt)
		free(wpt);
	return (db);
errout:
	if (db)
		waypoint_db_close(db);
	if (wpts_fp)
		fclose(wpts_fp);
	free(wpts_fname);
	if (wpt)
		free(wpt);
	return (NULL);
}

void
waypoint_db_close(waypoint_db_t *db)
{
	htbl_empty(&db->by_name, (void (*)(void *, void *))free, NULL);
	htbl_destroy(&db->by_name);
	free(db);
}

static void
waypoint_db_dump_cb(const void *k, const wpt_t *wpt, db_dump_info_t *info)
{
	UNUSED(k);
	append_format(info->result, info->result_sz,
	    "  %5s %2s %10.6lf %11.6lf\n",
	    wpt->name, wpt->icao_country_code, wpt->pos.lat, wpt->pos.lon);
}

char *
waypoint_db_dump(const waypoint_db_t *db)
{
	char		*result = NULL;
	size_t		result_sz = 0;
	db_dump_info_t	info = { .result = &result, .result_sz = &result_sz };

	append_format(&result, &result_sz,
	    "Waypoints (%lu):\n"
	    "   name CC        lat         lon\n"
	    "  ----- -- ---------- -----------\n",
	    htbl_count(&db->by_name));
	htbl_foreach(&db->by_name,
	    (void (*)(const void *, void *, void*))waypoint_db_dump_cb,
	    &info);
	return (result);
}

static bool_t
parse_navaid_line(const char *line, navaid_t *navaid)
{
	char	line_copy[128];
	char	*comps[11];
	int	dme;
	double	freq;

	STRLCPY_CHECK_ERROUT(line_copy, line);
	if (explode_line(line_copy, ',', comps, 11) != 11) {
		openfmc_log(OPENFMC_LOG_ERR, "Error parsing navaids: "
		    "line contains invalid number of columns, wanted 11.");
		goto errout;
	}
	STRLCPY_CHECK_ERROUT(navaid->ID, comps[0]);
	/* ok to truncate the name */
	(void) strlcpy(navaid->name, comps[1], sizeof (navaid->name));
	freq = atof(comps[2]);
	dme = atoi(comps[4]);
	if (is_valid_ndb_freq(freq)) {
		navaid->type = NAVAID_TYPE_NDB;
		navaid->freq = freq * 1000;
	} else if (is_valid_vor_freq(freq)) {
		navaid->type = (dme ? NAVAID_TYPE_VORDME : NAVAID_TYPE_VOR);
		navaid->freq = freq * 1000000;
	} else if (is_valid_loc_freq(freq)) {
		navaid->type = dme ? NAVAID_TYPE_LOCDME : NAVAID_TYPE_LOC;
		navaid->freq = freq * 1000000;
	} else if (is_valid_tacan_freq(freq)) {
		navaid->type = NAVAID_TYPE_TACAN;
		navaid->freq = freq * 1000000;
	} else if (freq == 0.0 && strcmp(comps[2], "000.00") == 0) {
		navaid->type = NAVAID_TYPE_UNKNOWN;
	} else {
		openfmc_log(OPENFMC_LOG_ERR, "Error parsing navaid: "
		    "\"%s\" is not a valid VOR, LOC or NDB frequency.",
		    comps[2]);
		goto errout;
	}
	if (!geo_pos3_from_str(comps[6], comps[7], comps[8], &navaid->pos)) {
		openfmc_log(OPENFMC_LOG_ERR, "Error parsing navaid: "
		    "lat/lon/elev position invalid.");
		goto errout;
	}
	STRLCPY_CHECK_ERROUT(navaid->icao_country_code, comps[9]);

	return (B_TRUE);
errout:
	openfmc_log(OPENFMC_LOG_ERR, "Offending line was: \"%s\".", line);
	return (B_FALSE);
}

navaid_db_t *
navaid_db_open(const char *navdata_dir)
{
	navaid_db_t	*db = NULL;
	FILE		*navaids_fp = NULL;
	char		*navaids_fname = NULL;
	ssize_t		line_len = 0;
	size_t		line_cap = 0, line_num = 0;
	char		*line = NULL;
	uint64_t	num_navaids = 0;
	navaid_t	*navaid = NULL;

	/* Open Navaids.txt */
	navaids_fname = malloc(strlen(navdata_dir) +
	    strlen(PATHSEP "Navaids.txt") + 1);
	sprintf(navaids_fname, "%s" PATHSEP "Navaids.txt", navdata_dir);
	navaids_fp = fopen(navaids_fname, "r");
	if (navaids_fp == NULL) {
		openfmc_log(OPENFMC_LOG_ERR, "Can't open %s: %s",
		    navaids_fname, strerror(errno));
		goto errout;
	}

	while ((line_len = parser_get_next_line(navaids_fp, &line, &line_cap,
	    &line_num)) != -1) {
		if (line_len > 3)
			num_navaids++;
	}
	rewind(navaids_fp);
	line_num = 0;
	if (num_navaids == 0 || num_navaids > MAX_NUM_NAVAIDS) {
		openfmc_log(OPENFMC_LOG_ERR, "Error parsing %s: invalid "
		    "number of navaids found: %llu", navaids_fname,
		    num_navaids);
		goto errout;
	}

	db = calloc(sizeof (*db), 1);
	if (!db)
		goto errout;
	htbl_create(&db->by_id, num_navaids, NAV_NAME_LEN, B_TRUE);

	while ((line_len = parser_get_next_line(navaids_fp, &line, &line_cap,
	    &line_num)) != -1) {
		if (line_len == 0)
			continue;
		navaid = calloc(sizeof (*navaid), 1);
		if (!navaid)
			goto errout;
		if (!parse_navaid_line(line, navaid))
			goto errout;
		htbl_set(&db->by_id, navaid->ID, navaid);
		navaid = NULL;
	}

	if (navaids_fp)
		fclose(navaids_fp);
	free(navaids_fname);
	if (navaid)
		free(navaid);
	return (db);
errout:
	if (db)
		navaid_db_close(db);
	if (navaids_fp)
		fclose(navaids_fp);
	free(navaids_fname);
	if (navaid)
		free(navaid);
	return (NULL);
}

void
navaid_db_close(navaid_db_t *db)
{
	htbl_empty(&db->by_id, (void (*)(void *, void *))free, NULL);
	htbl_destroy(&db->by_id);
	free(db);
}

static void
navaid_db_dump_append(const void *k, const navaid_t *navaid,
    db_dump_info_t *info)
{
	UNUSED(k);
	append_format(info->result, info->result_sz,
	    "  %7s %4s %2s %15s %6.2lf %sHz %10.6lf %11.6lf %d\n",
	    navaid_type_name(navaid->type), navaid->ID,
	    navaid->icao_country_code, navaid->name,
	    navaid->freq / (navaid->type == NAVAID_TYPE_NDB ? 1000.0 :
	    1000000.0),
	    navaid->type == NAVAID_TYPE_NDB ? "k" : "M",
	    navaid->pos.lat, navaid->pos.lon, (int)navaid->pos.elev);
}

char *
navaid_db_dump(const navaid_db_t *db)
{
	char		*result = NULL;
	size_t		result_sz = 0;
	db_dump_info_t	info = {
	    .result = &result, .result_sz = &result_sz
	};

	append_format(&result, &result_sz,
	    "Navaids: (%lu)\n"
	    "     type name CC       long name       freq        lat "
	    "        lon  elev\n"
	    "  ------- ---- -- --------------- ---------- ---------- "
	    "----------- -----\n", htbl_count(&db->by_id));
	htbl_foreach(&db->by_id,
	    (void (*)(const void *, void *, void*))navaid_db_dump_append,
	    &info);
	return (result);
}

/*
 * Locates the nearest wpt named `name' to `refpt' in `db' and returns it
 * as a wpt for usage in procedure segments. This function allows searching
 * either for waypoint fixes or radio navaids - pass the appropriate *db
 * argument and set the other to NULL. Only one type of database can be
 * searched at a time. When searching for navaids, further navaid type
 * discrimination is possible by passing a navaid type mask. If any navaid
 * type is acceptable, pass NAVAID_TYPE_ANY.
 * If the object is found, its 2D geo position is returned, otherwise
 * NULL_GEO_POS2 is returned.
 */
static geo_pos2_t
find_nearest(const char *name, geo_pos3_t refpt, const waypoint_db_t *wptdb,
    const navaid_db_t *navdb, navaid_type_t type)
{
	char		name_padd[NAV_NAME_LEN];
	const list_t	*list = NULL;
	geo_pos2_t	result = NULL_GEO_POS2;
	vect3_t		refpt_v;
	bool_t		usewptdb = (wptdb != NULL);
	double		min_dist = EARTH_MSL;

	ASSERT((wptdb != NULL && navdb == NULL) ||
	    (wptdb == NULL && navdb != NULL));

	memset(name_padd, 0, sizeof (name_padd));
	(void) strlcpy(name_padd, name, sizeof (name_padd));

	if (usewptdb)
		list = htbl_lookup_multi(&wptdb->by_name, name_padd);
	else
		list = htbl_lookup_multi(&navdb->by_id, name_padd);

	if (list == NULL)
		return (result);

	refpt_v = geo2ecef(refpt, &wgs84);
	for (void *v = list_head(list); v != NULL; v = list_next(list, v)) {
		geo_pos3_t	pos;
		double		dist;

		if (usewptdb) {
			wpt_t *wpt = HTBL_VALUE_MULTI(v);
			/* use the airport refpt's elev as approximation */
			pos = GEO2_TO_GEO3(wpt->pos, refpt.elev);
		} else {
			navaid_t *navaid = HTBL_VALUE_MULTI(v);
			if (!(navaid->type & type))
				continue;
			pos = navaid->pos;
		}
		dist = vect3_abs(vect3_sub(refpt_v, geo2ecef(pos,
		    &wgs84)));
		if (dist < min_dist) {
			result = GEO3_TO_GEO2(pos);
			min_dist = dist;
		}
	}
	return (result);
}

static bool_t
proc_navaid_lookup(const char *name, wpt_t *wpt, const airport_t *arpt,
    const waypoint_db_t *wptdb, const navaid_db_t *navdb, navaid_type_t type)
{
	geo_pos2_t	fix_pos = NULL_GEO_POS2, navaid_pos = NULL_GEO_POS2;
	geo_pos2_t	pos;

	if (wptdb != NULL)
		fix_pos = find_nearest(name, arpt->refpt, wptdb, NULL, type);
	if (navdb != NULL)
		navaid_pos = find_nearest(name, arpt->refpt, NULL, navdb, type);

	if (IS_NULL_GEO_POS(fix_pos) && IS_NULL_GEO_POS(navaid_pos)) {
		openfmc_log(OPENFMC_LOG_ERR, "Error looking up wpt/navaid "
		    "\"%s\" for arpt %s procedure: no wpt/navaid of type %s "
		    "found.", name, arpt->icao, navaid_type_name(type));
		return (B_FALSE);
	} else if (!IS_NULL_GEO_POS(fix_pos) && IS_NULL_GEO_POS(navaid_pos)) {
		pos = fix_pos;
	} else if (IS_NULL_GEO_POS(fix_pos) && !IS_NULL_GEO_POS(navaid_pos)) {
		pos = navaid_pos;
	} else {
		/* Found both, resolve conflict, pick the closest one */
		vect3_t refpt_v = geo2ecef(arpt->refpt, &wgs84);
		vect3_t fix_v = geo2ecef(GEO2_TO_GEO3(fix_pos, 0),
		    &wgs84);
		vect3_t navaid_v = geo2ecef(GEO2_TO_GEO3(navaid_pos, 0),
		    &wgs84);
		vect3_t r2f = vect3_sub(refpt_v, fix_v);
		vect3_t r2n = vect3_sub(refpt_v, navaid_v);
		if (vect3_abs(r2f) < vect3_abs(r2n))
			pos = fix_pos;
		else
			pos = navaid_pos;
	}

	memset(wpt, 0, sizeof (*wpt));
	(void) strlcpy(wpt->name, name, sizeof (wpt->name));
	wpt->pos = pos;

	return (B_TRUE);
}

static bool_t
parse_arpt_line(const char *line, airport_t *arpt)
{
	char	line_copy[128];
	char	*comps[10];

	/* Check this is an airport line and it's the one we're looking for */
	STRLCPY_CHECK_ERROUT(line_copy, line);
	if (explode_line(line_copy, ',', comps, 10) != 10 ||
	    strcmp(comps[0], "A") != 0 || strcmp(comps[1], arpt->icao) != 0) {
		/*
		 * Don't log this error, this function is used to look for
		 * airport lines.
		 */
		goto errout;
	}

	STRLCPY_CHECK_ERROUT(arpt->name, comps[2]);
	if (!geo_pos3_from_str(comps[3], comps[4], comps[5], &arpt->refpt)) {
		openfmc_log(OPENFMC_LOG_ERR, "Error parsing initial airport "
		    "line: reference point coordinates invalid.");
		goto errout;
	}
	arpt->TA = atoi(comps[6]);
	arpt->TL = atoi(comps[7]);
	arpt->longest_rwy = atoi(comps[8]);
	arpt->true_hdg = !!atoi(comps[9]);
	if (!is_valid_alt(arpt->TA) || !is_valid_alt(arpt->TL) ||
	    arpt->longest_rwy == 0 || arpt->longest_rwy > MAX_RWY_LEN) {
		openfmc_log(OPENFMC_LOG_ERR, "Error parsing initial airport "
		    "line: TA, TL or longest runway parameters invalid.");
		goto errout;
	}

	return (B_TRUE);
errout:
	return (B_FALSE);
}

static bool_t
parse_rwy_line(const char *line, runway_t *rwy, airport_t *arpt)
{
	char	line_copy[128];
	char	*comps[15];
	double	loc_freq;

	/* Line must start with "R" keyword */
	STRLCPY_CHECK_ERROUT(line_copy, line);
	if (explode_line(line_copy, ',', comps, 15) != 15 ||
	    strcmp(comps[0], "R") != 0) {
		openfmc_log(OPENFMC_LOG_ERR, "Error parsing runway line: "
		    "runway doesn't start with 'R'.");
		goto errout;
	}

	if (!is_valid_rwy_ID(comps[1])) {
		openfmc_log(OPENFMC_LOG_ERR, "Error parsing runway line: "
		    "runway ID \"%s\" invalid.", comps[1]);
		goto errout;
	}
	(void) strcpy(rwy->ID, comps[1]);
	rwy->hdg = atoi(comps[2]);
	if (arpt->true_hdg && rwy->hdg > 360 && rwy->hdg <= 720) {
		/* Some airports on true hdgs declare runways > 360! WTF?! */
		rwy->hdg %= 360;
	}
	rwy->length = atoi(comps[3]);
	/* rwy width field is unreliable! */
	rwy->width = atoi(comps[4]);
	rwy->loc_avail = atoi(comps[5]);
	loc_freq = atof(comps[6]);
	rwy->loc_freq = loc_freq * 1000000;
	rwy->loc_fcrs = atoi(comps[7]);
	rwy->gp_angle = atof(comps[11]);
	rwy->arpt = arpt;
	if (!is_valid_hdg(rwy->hdg) ||
	    rwy->length == 0 || rwy->length > MAX_RWY_LEN ||
	    (rwy->loc_avail != 0 && rwy->loc_avail != 1) ||
	    (rwy->loc_avail && !is_valid_loc_freq(loc_freq)) ||
	    (rwy->loc_avail && !is_valid_hdg(rwy->loc_fcrs)) ||
	    !geo_pos3_from_str(comps[8], comps[9], comps[10],
	    &rwy->thr_pos) ||
	    rwy->gp_angle < 0.0 || rwy->gp_angle > GP_MAX_ANGLE) {
		openfmc_log(OPENFMC_LOG_ERR, "Error parsing runway line: "
		    "invalid parameters found.");
		goto errout;
	}

	return (B_TRUE);
errout:
	openfmc_log(OPENFMC_LOG_ERR, "Error parsing runway line \"%s\".", line);
	return (B_FALSE);
}

static bool_t
parse_sid_proc_line(const airport_t *arpt, char **comps, size_t num_comps,
    navproc_t *proc)
{
	if (num_comps != 4) {
		openfmc_log(OPENFMC_LOG_ERR, "Error parsing SID line: "
		    "incorrect number of columns.");
		return (B_FALSE);
	}
	STRLCPY_CHECK_ERROUT(proc->name, comps[1]);
	if (is_valid_rwy_ID(comps[2])) {
		proc->type = NAVPROC_TYPE_SID;
		proc->rwy = airport_find_rwy_by_ID(arpt, comps[2]);
		if (proc->rwy == NULL) {
			openfmc_log(OPENFMC_LOG_ERR, "Error parsing SID line: "
			    "runway \"%s\" not found in parent airport.",
			    comps[2]);
			return (B_FALSE);
		}
	} else if (strcmp(comps[2], "ALL") == 0) {
		proc->type = NAVPROC_TYPE_SID_COMMON;
	} else {
		proc->type = NAVPROC_TYPE_SID_TRANS;
		STRLCPY_CHECK_ERROUT(proc->tr_name, comps[2]);
	}
	return (B_TRUE);
errout:
	return (B_FALSE);
}

static bool_t
parse_star_proc_line(const airport_t *arpt, char **comps, size_t num_comps,
    navproc_t *proc)
{
	if (num_comps != 4) {
		openfmc_log(OPENFMC_LOG_ERR, "Error parsing STAR line: "
		    "incorrect number of columns.");
		return (B_FALSE);
	}
	STRLCPY_CHECK_ERROUT(proc->name, comps[1]);
	if (is_valid_rwy_ID(comps[2])) {
		proc->type = NAVPROC_TYPE_STAR;
		proc->rwy = airport_find_rwy_by_ID(arpt, comps[2]);
		if (proc->rwy == NULL) {
			openfmc_log(OPENFMC_LOG_ERR, "Error parsing STAR line: "
			    "runway \"%s\" not found in parent airport.",
			    comps[2]);
			return (B_FALSE);
		}
	} else if (strcmp(comps[2], "ALL") == 0) {
		proc->type = NAVPROC_TYPE_STAR_COMMON;
	} else {
		proc->type = NAVPROC_TYPE_STAR_TRANS;
		STRLCPY_CHECK_ERROUT(proc->tr_name, comps[2]);
	}
	return (B_TRUE);
errout:
	return (B_FALSE);
}

static bool_t
parse_apptr_proc_line(const airport_t *arpt, char **comps, size_t num_comps,
    navproc_t *proc)
{
	if (num_comps != 4) {
		openfmc_log(OPENFMC_LOG_ERR, "Error parsing APPTR line: "
		    "incorrect number of columns.");
		return (B_FALSE);
	}
	proc->type = NAVPROC_TYPE_FINAL_TRANS;
	STRLCPY_CHECK_ERROUT(proc->name, comps[1]);
	proc->rwy = airport_find_rwy_by_ID(arpt, comps[2]);
	if (proc->rwy == NULL) {
		openfmc_log(OPENFMC_LOG_ERR, "Error parsing APPTR line: "
		    "runway \"%s\" not found in parent airport.",
		    comps[2]);
		return (B_FALSE);
	}
	STRLCPY_CHECK_ERROUT(proc->tr_name, comps[3]);
	return (B_TRUE);
errout:	/* needed for STRLCPY_CHECK_ERROUT */
	return (B_FALSE);
}

static bool_t
parse_final_proc_line(const airport_t *arpt, char **comps, size_t num_comps,
    navproc_t *proc)
{
	if (num_comps != 5) {
		openfmc_log(OPENFMC_LOG_ERR, "Error parsing FINAL line: "
		    "incorrect number of columns.");
		return (B_FALSE);
	}
	proc->type = NAVPROC_TYPE_FINAL;
	if (!is_valid_rwy_ID(comps[2])) {
		openfmc_log(OPENFMC_LOG_ERR, "Error parsing FINAL line: "
		    "invalid runway ID \"%s\".", comps[2]);
		return (B_FALSE);
	}
	STRLCPY_CHECK_ERROUT(proc->name, comps[1]);
	proc->rwy = airport_find_rwy_by_ID(arpt, comps[2]);
	if (proc->rwy == NULL) {
		openfmc_log(OPENFMC_LOG_ERR, "Error parsing FINAL line: "
		    "runway \"%s\" not found in parent airport.",
		    comps[2]);
		return (B_FALSE);
	}
	if (strlen(comps[3]) != 1) {
		openfmc_log(OPENFMC_LOG_ERR, "Error parsing FINAL line: "
		    "invalid approach type code \"%s\".", comps[3]);
		return (B_FALSE);
	}
	switch (comps[3][0]) {
	case 'I':
		proc->final_type = NAVPROC_FINAL_ILS;
		break;
	case 'D':
		proc->final_type = NAVPROC_FINAL_VOR;
		break;
	case 'N':
		proc->final_type = NAVPROC_FINAL_NDB;
		break;
	case 'G':
		proc->final_type = NAVPROC_FINAL_RNAV;
		break;
	case 'C':
		proc->final_type = NAVPROC_FINAL_LDA;
		break;
	default:
		openfmc_log(OPENFMC_LOG_ERR, "Error parsing FINAL line: "
		    "invalid approach type code \"%s\".", comps[3]);
		return (B_FALSE);
	}
	proc->num_main_segs = atoi(comps[4]);
	if (proc->num_main_segs > MAX_PROC_SEGS) {
		openfmc_log(OPENFMC_LOG_ERR, "Error parsing FINAL line: "
		    "invalid number of main segments \"%s\".", comps[4]);
		return (B_FALSE);
	}
	return (B_TRUE);
errout:	/* needed for STRLCPY_CHECK_ERROUT */
	return (B_FALSE);
}

static bool_t
parse_alt_spd_term(char *comps[3], alt_lim_t *alt, spd_lim_t *spd)
{
	alt->type = atoi(comps[0]);
	switch (alt->type) {
	case ALT_LIM_NONE:
		break;
	case ALT_LIM_AT:
	case ALT_LIM_AT_OR_ABV:
	case ALT_LIM_AT_OR_BLW:
		alt->alt1 = atoi(comps[1]);
		if (!is_valid_alt(alt->alt1)) {
			openfmc_log(OPENFMC_LOG_ERR, "Error parsing altitude "
			    "limit: invalid altitude value \"%s\".", comps[1]);
			return (B_FALSE);
		}
		break;
	case ALT_LIM_BETWEEN:
		alt->alt1 = atoi(comps[1]);
		alt->alt2 = atoi(comps[2]);
		if (!is_valid_alt(alt->alt1) || !is_valid_alt(alt->alt2)) {
			openfmc_log(OPENFMC_LOG_ERR, "Error parsing altitude "
			    "limit: invalid altitude values \"%s,%s\".",
			    comps[1], comps[2]);
			return (B_FALSE);
		}
		break;
	default:
		openfmc_log(OPENFMC_LOG_ERR, "Error parsing altitude "
		    "limit: unknown constraint type \"%s\".", comps[0]);
		return (B_FALSE);
	}

	spd->type = atoi(comps[3]);
	switch (spd->type) {
	case SPD_LIM_NONE:
		break;
	case SPD_LIM_AT_OR_BLW:
		spd->spd1 = atoi(comps[4]);
		if (!is_valid_spd(spd->spd1)) {
			openfmc_log(OPENFMC_LOG_ERR, "Error parsing speed "
			    "limit: invalid speed value \"%s\".", comps[4]);
			return (B_FALSE);
		}
		break;
	default:
		openfmc_log(OPENFMC_LOG_ERR, "Error parsing speed "
		    "limit: unknown limit type \"%s\".", comps[3]);
		return (B_FALSE);
	}

	return (B_TRUE);
}

static bool_t
parse_proc_seg_wpt(char *comps[3], wpt_t *wpt)
{
	if (strlen(comps[0]) > sizeof (wpt->name) - 1)
		return (B_FALSE);
	if (!geo_pos2_from_str(comps[1], comps[2], &wpt->pos) ||
	    !STRLCPY_CHECK(wpt->name, comps[0]))
		return (B_FALSE);
	return (B_TRUE);
}

#define	CHECK_NUM_COMPS(n, seg_type) \
	do { \
		if (num_comps != (n)) { \
			openfmc_log(OPENFMC_LOG_ERR, "Error parsing %s " \
			    "leg definition line: invalid number of columns " \
			    "on line, wanted %d, got %lu.", #seg_type, n, \
			    num_comps); \
			return (B_FALSE); \
		} \
	} while (0)

static bool_t
parse_AF_seg(char **comps, size_t num_comps, navproc_seg_t *seg,
    const airport_t *arpt, const navaid_db_t *db)
{
	int dir;

	CHECK_NUM_COMPS(17, AF);
	seg->type = NAVPROC_SEG_TYPE_ARC_TO_FIX;
	dir = atoi(comps[4]);
	seg->leg_cmd.dme_arc.start_radial = atof(comps[8]);
	seg->leg_cmd.dme_arc.radius = atof(comps[7]);
	seg->leg_cmd.dme_arc.end_radial = atof(comps[6]);
	if ((dir != 1 && dir != 2) ||
	    !parse_proc_seg_wpt(&comps[1], &seg->term_cond.fix) ||
	    !is_valid_hdg(seg->leg_cmd.dme_arc.start_radial) ||
	    !is_valid_arc_radius(seg->leg_cmd.dme_arc.radius) ||
	    !is_valid_hdg(seg->leg_cmd.dme_arc.end_radial) ||
	    !parse_alt_spd_term(&comps[9], &seg->alt_lim, &seg->spd_lim) ||
	    !proc_navaid_lookup(comps[5], &seg->leg_cmd.dme_arc.navaid,
	    arpt, NULL, db, NAVAID_TYPE_ANY)) {
		return (B_FALSE);
	}
	seg->leg_cmd.dme_arc.cw = (dir == 2);

	return (B_TRUE);
}

static void
dump_alt_constr(const alt_lim_t *alt, char desc[32])
{
	switch (alt->type) {
	case ALT_LIM_NONE:
		desc[0] = 0;
		break;
	case ALT_LIM_AT:
		sprintf(desc, ",A==%u", alt->alt1);
		break;
	case ALT_LIM_AT_OR_ABV:
		sprintf(desc, ",A>=%u", alt->alt1);
		break;
	case ALT_LIM_AT_OR_BLW:
		sprintf(desc, ",A<=%u", alt->alt1);
		break;
	case ALT_LIM_BETWEEN:
		sprintf(desc, ",%u<=A<=%u", alt->alt2, alt->alt1);
		break;
	default:
		ASSERT(0);
	}
}

static void
dump_spd_constr(const spd_lim_t *spd, char desc[16])
{
	switch (spd->type) {
	case SPD_LIM_NONE:
		desc[0] = 0;
		break;
	case SPD_LIM_AT_OR_BLW:
		sprintf(desc, ",S<=%u", spd->spd1);
		break;
	default:
		ASSERT(0);
	}
}

#define	DUMP_ALT_LIM(alt) \
	char alt_lim_desc[64]; \
	dump_alt_constr((alt), alt_lim_desc)

#define	DUMP_SPD_LIM(spd) \
	char spd_lim_desc[32]; \
	dump_spd_constr((spd), spd_lim_desc)

#define	FIX_PRINTF_ARG(wpt) \
	(wpt)->name, (wpt)->pos.lat, (wpt)->pos.lon

static void
dump_AF_seg(char **result, size_t *result_sz, const navproc_seg_t *seg)
{
	DUMP_ALT_LIM(&seg->alt_lim);
	DUMP_SPD_LIM(&seg->spd_lim);
	append_format(result, result_sz, "\tAF,N:%s(%lfx%lf),SR:%.01lf,"
	    "ER:%.01lf,r:%.01lf,F:%s(%lfx%lf)%s%s\n",
	    FIX_PRINTF_ARG(&seg->leg_cmd.dme_arc.navaid),
	    seg->leg_cmd.dme_arc.start_radial, seg->leg_cmd.dme_arc.end_radial,
	    seg->leg_cmd.dme_arc.radius,
	    FIX_PRINTF_ARG(&seg->term_cond.fix), alt_lim_desc,
	    spd_lim_desc);
}

static bool_t
parse_CA_seg(char **comps, size_t num_comps, navproc_seg_t *seg)
{
	CHECK_NUM_COMPS(11, CA);
	seg->type = NAVPROC_SEG_TYPE_CRS_TO_ALT;
	seg->leg_cmd.hdg.hdg = atof(comps[2]);
	seg->leg_cmd.hdg.turn = atoi(comps[1]);
	if (!is_valid_hdg(seg->leg_cmd.hdg.hdg) ||
	    !is_valid_turn(seg->leg_cmd.hdg.turn) ||
	    !parse_alt_spd_term(&comps[3], &seg->term_cond.alt,
	    &seg->spd_lim) ||
	    /* altitude constraint is required for CA segs */
	    seg->term_cond.alt.type == ALT_LIM_NONE)
		return (B_FALSE);
	seg->alt_lim = seg->term_cond.alt;
	return (B_TRUE);
}

static void
dump_CA_seg(char **result, size_t *result_sz, const navproc_seg_t *seg)
{
	DUMP_ALT_LIM(&seg->term_cond.alt);
	DUMP_SPD_LIM(&seg->spd_lim);
	append_format(result, result_sz, "\tCA,C:%.01lf,T:%s%s%s\n",
	    seg->leg_cmd.hdg.hdg, dump_turn(seg->leg_cmd.hdg.turn),
	    alt_lim_desc, spd_lim_desc);
}

static bool_t
parse_CD_seg(char **comps, size_t num_comps, navproc_seg_t *seg,
    const airport_t *arpt, const navaid_db_t *db)
{
	CHECK_NUM_COMPS(18, CD);
	seg->type = NAVPROC_SEG_TYPE_CRS_TO_DME;
	seg->leg_cmd.hdg.hdg = atof(comps[8]);
	seg->leg_cmd.hdg.turn = atoi(comps[2]);
	seg->term_cond.dme.dist = atof(comps[9]);
	if (!is_valid_hdg(seg->leg_cmd.hdg.hdg) ||
	    !is_valid_turn(seg->leg_cmd.hdg.turn) ||
	    !parse_alt_spd_term(&comps[10], &seg->alt_lim, &seg->spd_lim) ||
	    !proc_navaid_lookup(comps[5], &seg->term_cond.dme.navaid,
	    arpt, NULL, db, NAVAID_TYPE_ANY))
		return (B_FALSE);
	return (B_TRUE);
}

static void
dump_CD_seg(char **result, size_t *result_sz, const navproc_seg_t *seg)
{
	DUMP_ALT_LIM(&seg->alt_lim);
	DUMP_SPD_LIM(&seg->spd_lim);
	append_format(result, result_sz,
	    "\tCD,C:%.01lf,T:%s,N:%s(%lfx%lf),d:%.01lf%s%s\n",
	    seg->leg_cmd.hdg.hdg, dump_turn(seg->leg_cmd.hdg.turn),
	    FIX_PRINTF_ARG(&seg->term_cond.dme.navaid),
	    seg->term_cond.dme.dist, alt_lim_desc, spd_lim_desc);
}

static bool_t
parse_CF_seg(char **comps, size_t num_comps, navproc_seg_t *seg,
    const airport_t *arpt, const navaid_db_t *db)
{
	CHECK_NUM_COMPS(18, CF);
	seg->type = NAVPROC_SEG_TYPE_CRS_TO_FIX;
	seg->leg_cmd.navaid_crs.crs = atof(comps[8]);
	seg->leg_cmd.navaid_crs.turn = atoi(comps[4]);
	memset(&seg->leg_cmd.navaid_crs.navaid, 0,
	    sizeof (seg->leg_cmd.navaid_crs.navaid));
	if (strcmp(comps[5], " ") != 0) {
		/* this is optional */
		(void) proc_navaid_lookup(comps[5],
		    &seg->leg_cmd.navaid_crs.navaid,
		    arpt, NULL, db, NAVAID_TYPE_ANY);
	}
	if (!is_valid_hdg(seg->leg_cmd.navaid_crs.crs) ||
	    !is_valid_turn(seg->leg_cmd.navaid_crs.turn) ||
	    !parse_proc_seg_wpt(&comps[1], &seg->term_cond.fix) ||
	    !parse_alt_spd_term(&comps[10], &seg->alt_lim, &seg->spd_lim))
		return (B_FALSE);
	return (B_TRUE);
}

static void
dump_CF_seg(char **result, size_t *result_sz, const navproc_seg_t *seg)
{
	DUMP_ALT_LIM(&seg->alt_lim);
	DUMP_SPD_LIM(&seg->spd_lim);
	append_format(result, result_sz,
	    "\tCF,N:%s(%lfx%lf),C:%.01lf,T:%s,F:%s(%lfx%lf)%s%s\n",
	    FIX_PRINTF_ARG(&seg->leg_cmd.navaid_crs.navaid),
	    seg->leg_cmd.navaid_crs.crs,
	    dump_turn(seg->leg_cmd.navaid_crs.turn),
	    FIX_PRINTF_ARG(&seg->term_cond.fix), alt_lim_desc, spd_lim_desc);
}

static bool_t
parse_CI_CR_seg(char **comps, size_t num_comps, navproc_seg_t *seg,
    bool_t is_CI, const airport_t *arpt, const navaid_db_t *db)
{
	if (is_CI)
		CHECK_NUM_COMPS(13, CI);
	else
		CHECK_NUM_COMPS(13, CR);
	seg->type = NAVPROC_SEG_TYPE_CRS_TO_INTCP;
	seg->term_cond.radial.radial = atof(comps[3]);
	seg->leg_cmd.hdg.hdg = atof(comps[4]);
	seg->leg_cmd.hdg.turn = atoi(comps[1]);
	if (!is_valid_hdg(seg->leg_cmd.hdg.hdg) ||
	    !is_valid_turn(seg->leg_cmd.hdg.turn) ||
	    !parse_alt_spd_term(&comps[5], &seg->alt_lim, &seg->spd_lim) ||
	    (!is_CI && !is_valid_hdg(seg->term_cond.radial.radial)) ||
	    (is_CI && strcmp(comps[2], " ") != 0 &&
	    !proc_navaid_lookup(comps[2], &seg->term_cond.radial.navaid,
	    arpt, NULL, db, NAVAID_TYPE_ANY)) ||
	    (!is_CI && !proc_navaid_lookup(comps[2],
	    &seg->term_cond.radial.navaid, arpt, NULL, db, NAVAID_TYPE_ANY)))
		return (B_FALSE);
	return (B_TRUE);
}

static void
dump_CI_CR_seg(char **result, size_t *result_sz, const navproc_seg_t *seg)
{
	DUMP_ALT_LIM(&seg->alt_lim);
	DUMP_SPD_LIM(&seg->spd_lim);
	if (seg->type == NAVPROC_SEG_TYPE_CRS_TO_INTCP) {
		append_format(result, result_sz,
		    "\tCI,C:%.01lf,T:%s,N:%s(%lfx%lf)%s%s\n",
		    seg->leg_cmd.hdg.hdg, dump_turn(seg->leg_cmd.hdg.turn),
		    FIX_PRINTF_ARG(&seg->term_cond.radial.navaid),
		    alt_lim_desc, spd_lim_desc);
	} else {
		append_format(result, result_sz,
		    "\tCR,C:%.01lf,T:%s,N:%s(%lfx%lf),R:%.01f%s%s\n",
		    seg->leg_cmd.hdg.hdg, dump_turn(seg->leg_cmd.hdg.turn),
		    FIX_PRINTF_ARG(&seg->term_cond.radial.navaid),
		    seg->term_cond.radial.radial, alt_lim_desc, spd_lim_desc);
	}
}

static bool_t
parse_DF_TF_seg(char **comps, size_t num_comps, navproc_seg_t *seg,
    bool_t is_DF)
{
	if (is_DF)
		CHECK_NUM_COMPS(16, DF);
	else
		CHECK_NUM_COMPS(18, TF);
	seg->type = is_DF ? NAVPROC_SEG_TYPE_DIR_TO_FIX :
	    NAVPROC_SEG_TYPE_TRK_TO_FIX;
	if (!parse_proc_seg_wpt(&comps[1], &seg->term_cond.fix) ||
	    !parse_alt_spd_term(&comps[is_DF ? 8 : 10], &seg->alt_lim,
	    &seg->spd_lim))
		return (B_FALSE);
	return (B_TRUE);
}

static void
dump_DF_TF_seg(char **result, size_t *result_sz, const navproc_seg_t *seg)
{
	DUMP_ALT_LIM(&seg->alt_lim);
	DUMP_SPD_LIM(&seg->spd_lim);
	append_format(result, result_sz, "\t%s,F:%s(%lfx%lf)%s%s\n",
	    seg->type == NAVPROC_SEG_TYPE_DIR_TO_FIX ? "DF" : "TF",
	    seg->term_cond.fix.name, seg->term_cond.fix.pos.lat,
	    seg->term_cond.fix.pos.lon, alt_lim_desc, spd_lim_desc);
}

static bool_t
parse_FA_seg(char **comps, size_t num_comps, navproc_seg_t *seg)
{
	CHECK_NUM_COMPS(17, FA);
	seg->type = NAVPROC_SEG_TYPE_FIX_TO_ALT;
	seg->leg_cmd.fix_crs.crs = atof(comps[8]);
	if (!parse_proc_seg_wpt(&comps[1], &seg->leg_cmd.fix_crs.fix) ||
	    !is_valid_hdg(seg->leg_cmd.fix_crs.crs) ||
	    !parse_alt_spd_term(&comps[9], &seg->term_cond.alt,
	    &seg->spd_lim) ||
	    /* altitude constraint is required for FA segs */
	    seg->term_cond.alt.type == ALT_LIM_NONE)
		return (B_FALSE);
	seg->alt_lim = seg->term_cond.alt;
	return (B_TRUE);
}

static void
dump_FA_seg(char **result, size_t *result_sz, const navproc_seg_t *seg)
{
	DUMP_ALT_LIM(&seg->term_cond.alt);
	DUMP_SPD_LIM(&seg->spd_lim);
	append_format(result, result_sz, "\tFA,F:%s(%lfx%lf),c:%lf%s%s\n",
	    FIX_PRINTF_ARG(&seg->leg_cmd.fix_crs.fix),
	    seg->leg_cmd.fix_crs.crs, alt_lim_desc, spd_lim_desc);
}

static bool_t
parse_FC_seg(char **comps, size_t num_comps, navproc_seg_t *seg)
{
	CHECK_NUM_COMPS(18, FC);
	seg->type = NAVPROC_SEG_TYPE_FIX_TO_DIST;

	seg->leg_cmd.fix_crs.crs = atof(comps[8]);
	seg->term_cond.dist = atof(comps[9]);
	if (!parse_proc_seg_wpt(&comps[1], &seg->leg_cmd.fix_crs.fix) ||
	    !is_valid_hdg(seg->leg_cmd.fix_crs.crs) ||
	    !parse_alt_spd_term(&comps[10], &seg->alt_lim, &seg->spd_lim))
		return (B_FALSE);
	return (B_TRUE);
}

static void
dump_FC_seg(char **result, size_t *result_sz, const navproc_seg_t *seg)
{
	DUMP_ALT_LIM(&seg->alt_lim);
	DUMP_SPD_LIM(&seg->spd_lim);
	append_format(result, result_sz, "\tFC,F:%s(%lfx%lf),c:%.1lf,"
	    "d:%.01f%s%s\n", FIX_PRINTF_ARG(&seg->leg_cmd.fix_crs.fix),
	    seg->leg_cmd.fix_crs.crs, seg->term_cond.dist, alt_lim_desc,
	    spd_lim_desc);
}

static bool_t
parse_FD_seg(char **comps, size_t num_comps, navproc_seg_t *seg,
    const airport_t *arpt, const navaid_db_t *db)
{
	CHECK_NUM_COMPS(18, FD);
	seg->type = NAVPROC_SEG_TYPE_FIX_TO_DME;

	seg->term_cond.dme.dist = atof(comps[7]);
	seg->leg_cmd.fix_crs.crs = atof(comps[8]);
	if (!parse_proc_seg_wpt(&comps[1], &seg->leg_cmd.fix) ||
	    !parse_alt_spd_term(&comps[10], &seg->alt_lim, &seg->spd_lim) ||
	    !proc_navaid_lookup(comps[5], &seg->term_cond.dme.navaid,
	    arpt, NULL, db, NAVAID_TYPE_ANY))
		return (B_FALSE);
	return (B_TRUE);
}

static void
dump_FD_seg(char **result, size_t *result_sz, const navproc_seg_t *seg)
{
	DUMP_ALT_LIM(&seg->alt_lim);
	DUMP_SPD_LIM(&seg->spd_lim);
	append_format(result, result_sz,
	    "\tFD,F:%s(%lfx%lf),N:%s(%lfx%lf),d:%.01f%s%s\n",
	    FIX_PRINTF_ARG(&seg->leg_cmd.fix_crs.fix),
	    FIX_PRINTF_ARG(&seg->term_cond.dme.navaid),
	    seg->term_cond.dme.dist, alt_lim_desc, spd_lim_desc);
}

static bool_t
parse_FM_seg(char **comps, size_t num_comps, navproc_seg_t *seg)
{
	CHECK_NUM_COMPS(17, FM);
	seg->type = NAVPROC_SEG_TYPE_FIX_TO_MANUAL;
	seg->leg_cmd.fix_crs.crs = atof(comps[8]);
	if (!parse_proc_seg_wpt(&comps[1], &seg->leg_cmd.fix_crs.fix) ||
	    !is_valid_hdg(seg->leg_cmd.fix_crs.crs) ||
	    !parse_alt_spd_term(&comps[9], &seg->alt_lim, &seg->spd_lim))
		return (B_FALSE);
	return (B_TRUE);
}

static void
dump_FM_seg(char **result, size_t *result_sz, const navproc_seg_t *seg)
{
	DUMP_ALT_LIM(&seg->alt_lim);
	DUMP_SPD_LIM(&seg->spd_lim);
	append_format(result, result_sz, "\tFM,F:%s(%lfx%lf),T:%.01lf%s%s\n",
	    FIX_PRINTF_ARG(&seg->leg_cmd.fix_crs.fix),
	    seg->leg_cmd.fix_crs.crs, alt_lim_desc, spd_lim_desc);
}

static bool_t
parse_HA_HF_HM_seg(char **comps, size_t num_comps, navproc_seg_t *seg,
    navproc_seg_type_t type)
{
	if (type == NAVPROC_SEG_TYPE_HOLD_TO_ALT)
		CHECK_NUM_COMPS(19, HA);
	else if (type == NAVPROC_SEG_TYPE_HOLD_TO_FIX)
		CHECK_NUM_COMPS(19, HF);
	else if (type == NAVPROC_SEG_TYPE_HOLD_TO_MANUAL)
		CHECK_NUM_COMPS(19, HM);
	else
		ASSERT(0);
	seg->type = type;
	seg->leg_cmd.hold.turn_right = atoi(comps[4]);
	seg->leg_cmd.hold.inbd_crs = atof(comps[8]);
	seg->leg_cmd.hold.leg_len = atof(comps[9]);
	if (!parse_proc_seg_wpt(&comps[1], &seg->leg_cmd.hold.wpt) ||
	    (seg->leg_cmd.hold.turn_right != 1 &&
	    seg->leg_cmd.hold.turn_right != 2) ||
	    !parse_alt_spd_term(&comps[10], &seg->alt_lim, &seg->spd_lim) ||
	    /* alt constr is mandatory on HA segs */
	    (type == NAVPROC_SEG_TYPE_HOLD_TO_ALT &&
	    seg->alt_lim.type == ALT_LIM_NONE) ||
	    !is_valid_hdg(seg->leg_cmd.hold.inbd_crs))
		return (B_FALSE);
	if (type == NAVPROC_SEG_TYPE_HOLD_TO_ALT)
		seg->term_cond.alt = seg->alt_lim;
	else if (type == NAVPROC_SEG_TYPE_HOLD_TO_FIX)
		seg->term_cond.fix = seg->leg_cmd.hold.wpt;
	/* change turn flag from 1-2 to 0-1 */
	seg->leg_cmd.hold.turn_right--;
	return (B_TRUE);
}

static void
dump_HA_HF_HM_seg(char **result, size_t *result_sz, const navproc_seg_t *seg)
{
	DUMP_SPD_LIM(&seg->spd_lim);
	if (seg->type == NAVPROC_SEG_TYPE_HOLD_TO_ALT) {
		DUMP_ALT_LIM(&seg->term_cond.alt);
		append_format(result, result_sz,
		    "\tHA,F:%s(%lfx%lf),IC:%.01lf,L:%.01lf,R:%d%s%s\n",
		    FIX_PRINTF_ARG(&seg->leg_cmd.hold.wpt),
		    seg->leg_cmd.hold.inbd_crs, seg->leg_cmd.hold.leg_len,
		    seg->leg_cmd.hold.turn_right, alt_lim_desc, spd_lim_desc);
	} else {
		DUMP_ALT_LIM(&seg->alt_lim);
		append_format(result, result_sz,
		    "\t%s,F:%s(%lfx%lf),IC:%.01lf,L:%.01lf,R:%d%s%s\n",
		    seg->type == NAVPROC_SEG_TYPE_HOLD_TO_FIX ? "HF" : "HM",
		    FIX_PRINTF_ARG(&seg->leg_cmd.hold.wpt),
		    seg->leg_cmd.hold.inbd_crs, seg->leg_cmd.hold.leg_len,
		    seg->leg_cmd.hold.turn_right, alt_lim_desc, spd_lim_desc);
	}
}

static bool_t
parse_IF_seg(char **comps, size_t num_comps, navproc_seg_t *seg)
{
	CHECK_NUM_COMPS(15, IF);
	seg->type = NAVPROC_SEG_TYPE_INIT_FIX;
	if (!parse_proc_seg_wpt(&comps[1], &seg->leg_cmd.fix) ||
	    !parse_alt_spd_term(&comps[7], &seg->alt_lim, &seg->spd_lim))
		return (B_FALSE);
	return (B_TRUE);
}

static void
dump_IF_seg(char **result, size_t *result_sz, const navproc_seg_t *seg)
{
	DUMP_ALT_LIM(&seg->alt_lim);
	DUMP_SPD_LIM(&seg->spd_lim);
	append_format(result, result_sz, "\tIF,F:%s(%lfx%lf)%s%s\n",
	    FIX_PRINTF_ARG(&seg->leg_cmd.fix), alt_lim_desc, spd_lim_desc);
}

static bool_t
parse_PI_seg(char **comps, size_t num_comps, navproc_seg_t *seg,
    const airport_t *arpt, const navaid_db_t *db)
{
	int turn_dir;

	CHECK_NUM_COMPS(18, PI);
	seg->type = NAVPROC_SEG_TYPE_PROC_TURN;
	turn_dir = atoi(comps[4]);
	seg->leg_cmd.proc_turn.outbd_turn_hdg = atof(comps[6]);
	seg->leg_cmd.proc_turn.max_excrs_dist = atof(comps[7]);
	seg->leg_cmd.proc_turn.outbd_radial = atof(comps[8]);
	seg->leg_cmd.proc_turn.max_excrs_time = atof(comps[9]);
	if (!parse_proc_seg_wpt(&comps[1], &seg->leg_cmd.proc_turn.startpt) ||
	    (turn_dir != 1 && turn_dir != 2) ||
	    !is_valid_hdg(seg->leg_cmd.proc_turn.outbd_turn_hdg) ||
	    !is_valid_hdg(seg->leg_cmd.proc_turn.outbd_radial) ||
	    !parse_alt_spd_term(&comps[10], &seg->alt_lim, &seg->spd_lim) ||
	    !proc_navaid_lookup(comps[5], &seg->leg_cmd.proc_turn.navaid,
	    arpt, NULL, db, NAVAID_TYPE_ANY))
		return (B_FALSE);
	seg->leg_cmd.proc_turn.turn_right = (turn_dir == 1);
	return (B_TRUE);
}

static void
dump_PI_seg(char **result, size_t *result_sz, const navproc_seg_t *seg)
{
	DUMP_ALT_LIM(&seg->alt_lim);
	DUMP_SPD_LIM(&seg->spd_lim);
	append_format(result, result_sz,
	    "\tPI,SP:%s(%lfx%lf),OR:%.01lf,TH:%.01lf,"
	    "right:%d,MD:%.01lf,MT:%.01lf,N:%s(%lfx%lf)%s%s\n",
	    FIX_PRINTF_ARG(&seg->leg_cmd.proc_turn.startpt),
	    seg->leg_cmd.proc_turn.outbd_radial,
	    seg->leg_cmd.proc_turn.outbd_turn_hdg,
	    seg->leg_cmd.proc_turn.turn_right,
	    seg->leg_cmd.proc_turn.max_excrs_dist,
	    seg->leg_cmd.proc_turn.max_excrs_time,
	    FIX_PRINTF_ARG(&seg->leg_cmd.proc_turn.navaid),
	    alt_lim_desc, spd_lim_desc);
}

static bool_t
parse_RF_seg(char **comps, size_t num_comps, navproc_seg_t *seg,
    const airport_t *arpt, const waypoint_db_t *db)
{
	CHECK_NUM_COMPS(16, RF);
	seg->type = NAVPROC_SEG_TYPE_RADIUS_ARC_TO_FIX;
	seg->leg_cmd.radius_arc.cw = atoi(comps[4]);
	if (seg->leg_cmd.radius_arc.cw != 1 &&
	    seg->leg_cmd.radius_arc.cw != 2)
		return (B_FALSE);
	/* change CW flag from 1-2 to 0-1 */
	seg->leg_cmd.radius_arc.cw--;
	seg->leg_cmd.radius_arc.radius = atof(comps[7]);
	if (!parse_proc_seg_wpt(&comps[1], &seg->term_cond.fix) ||
	    !is_valid_arc_radius(seg->leg_cmd.radius_arc.radius) ||
	    !parse_alt_spd_term(&comps[8], &seg->alt_lim, &seg->spd_lim) ||
	    !proc_navaid_lookup(comps[5], &seg->leg_cmd.radius_arc.ctr_wpt,
	    arpt, db, NULL, NAVAID_TYPE_ANY))
		return (B_FALSE);

	return (B_TRUE);
}

static void
dump_RF_seg(char **result, size_t *result_sz, const navproc_seg_t *seg)
{
	DUMP_ALT_LIM(&seg->alt_lim);
	DUMP_SPD_LIM(&seg->spd_lim);
	append_format(result, result_sz,
	    "\tRF,F:%s(%lfx%lf),r:%.01lf,cw:%d,F:%s(%lfx%lf)%s%s\n",
	    FIX_PRINTF_ARG(&seg->leg_cmd.radius_arc.ctr_wpt),
	    seg->leg_cmd.radius_arc.radius, seg->leg_cmd.radius_arc.cw,
	    FIX_PRINTF_ARG(&seg->term_cond.fix), alt_lim_desc, spd_lim_desc);
}

static bool_t
parse_VA_seg(char **comps, size_t num_comps, navproc_seg_t *seg)
{
	CHECK_NUM_COMPS(11, VA);
	seg->type = NAVPROC_SEG_TYPE_HDG_TO_ALT;
	seg->leg_cmd.hdg.hdg = atof(comps[2]);
	seg->leg_cmd.hdg.turn = atoi(comps[1]);
	if (!is_valid_hdg(seg->leg_cmd.hdg.hdg) ||
	    !is_valid_turn(seg->leg_cmd.hdg.turn) ||
	    !parse_alt_spd_term(&comps[3], &seg->alt_lim, &seg->spd_lim) ||
	    /* alt constr mandatory on VA segs */
	    seg->alt_lim.type == ALT_LIM_NONE) {
		openfmc_log(OPENFMC_LOG_ERR, "Error parsing VA segment line");
		return (B_FALSE);
	}
	seg->term_cond.alt = seg->alt_lim;

	return (B_TRUE);
}

static void
dump_VA_seg(char **result, size_t *result_sz, const navproc_seg_t *seg)
{
	DUMP_ALT_LIM(&seg->term_cond.alt);
	DUMP_SPD_LIM(&seg->spd_lim);
	append_format(result, result_sz, "\tVA,H:%.01lf,T:%s%s%s\n",
	    seg->leg_cmd.hdg.hdg, dump_turn(seg->leg_cmd.hdg.turn),
	    alt_lim_desc, spd_lim_desc);
}

static bool_t
parse_VD_seg(char **comps, size_t num_comps, navproc_seg_t *seg,
    const airport_t *arpt, const navaid_db_t *db)
{
	CHECK_NUM_COMPS(18, VD);
	seg->type = NAVPROC_SEG_TYPE_HDG_TO_DME;
	seg->leg_cmd.hdg.hdg = atof(comps[8]);
	seg->leg_cmd.hdg.turn = atoi(comps[3]);
	seg->term_cond.dme.dist = atof(comps[9]);
	if (!is_valid_hdg(seg->leg_cmd.hdg.hdg) ||
	    !is_valid_turn(seg->leg_cmd.hdg.turn) ||
	    !parse_alt_spd_term(&comps[10], &seg->alt_lim, &seg->spd_lim) ||
	    !proc_navaid_lookup(comps[5], &seg->term_cond.dme.navaid,
	    arpt, NULL, db, NAVAID_TYPE_ANY)) {
		openfmc_log(OPENFMC_LOG_ERR, "Error parsing VD segment line");
		return (B_FALSE);
	}

	return (B_TRUE);
}

static void
dump_VD_seg(char **result, size_t *result_sz, const navproc_seg_t *seg)
{
	DUMP_ALT_LIM(&seg->alt_lim);
	DUMP_SPD_LIM(&seg->spd_lim);
	append_format(result, result_sz,
	    "\tVA,H:%.01lf,T:%s,N:%s(%lfx%lf),d:%.01lf%s%s\n",
	    seg->leg_cmd.hdg.hdg, dump_turn(seg->leg_cmd.hdg.turn),
	    FIX_PRINTF_ARG(&seg->term_cond.dme.navaid),
	    seg->term_cond.dme.dist, alt_lim_desc, spd_lim_desc);
}

static bool_t
parse_VI_VM_VR_seg(char **comps, size_t num_comps, navproc_seg_t *seg,
    navproc_seg_type_t type, const airport_t *arpt, const navaid_db_t *db)
{
	char *leg_name;
	if (type == NAVPROC_SEG_TYPE_HDG_TO_INTCP) {
		CHECK_NUM_COMPS(13, VI);
		leg_name = "VI";
		seg->leg_cmd.hdg.turn = atoi(comps[1]);
	} else if (type == NAVPROC_SEG_TYPE_HDG_TO_MANUAL) {
		CHECK_NUM_COMPS(13, VM);
		leg_name = "VM";
		seg->leg_cmd.hdg.turn = atoi(comps[3]);
	} else if (type == NAVPROC_SEG_TYPE_HDG_TO_RADIAL) {
		CHECK_NUM_COMPS(13, VR);
		leg_name = "VR";
		seg->leg_cmd.hdg.turn = atoi(comps[1]);
	} else {
		ASSERT(0);
	}
	seg->type = type;
	seg->leg_cmd.hdg.hdg = atof(comps[4]);
	seg->term_cond.radial.radial = atof(comps[3]);
	if (!is_valid_hdg(seg->leg_cmd.hdg.hdg) ||
	    !is_valid_turn(seg->leg_cmd.hdg.turn) ||
	    (type == NAVPROC_SEG_TYPE_HDG_TO_RADIAL &&
	    !is_valid_hdg(seg->term_cond.radial.radial)) ||
	    !parse_alt_spd_term(&comps[5], &seg->alt_lim, &seg->spd_lim)) {
		openfmc_log(OPENFMC_LOG_ERR, "Error parsing %s segment line",
		    leg_name);
		return (B_FALSE);
	}
	/* Grab a non-empty navaid to termination condition on VI segs */
	if (type == NAVPROC_SEG_TYPE_HDG_TO_INTCP &&
	    strcmp(comps[2], " ") != 0 && !proc_navaid_lookup(comps[2],
	    &seg->term_cond.fix, arpt, NULL, db, NAVAID_TYPE_ANY))
		return (B_FALSE);
	else if (type == NAVPROC_SEG_TYPE_HDG_TO_RADIAL &&
	    !proc_navaid_lookup(comps[2], &seg->term_cond.radial.navaid,
	    arpt, NULL, db, NAVAID_TYPE_ANY))
		return (B_FALSE);

	return (B_TRUE);
}

static void
dump_VI_VM_VR_seg(char **result, size_t *result_sz, const navproc_seg_t *seg)
{
	DUMP_ALT_LIM(&seg->alt_lim);
	DUMP_SPD_LIM(&seg->spd_lim);
	if (seg->type == NAVPROC_SEG_TYPE_HDG_TO_INTCP) {
		append_format(result, result_sz,
		    "\tVI,H:%.01lf,T:%s,N:%s(%lfx%lf)%s%s\n",
		    seg->leg_cmd.hdg.hdg, dump_turn(seg->leg_cmd.hdg.turn),
		    FIX_PRINTF_ARG(&seg->term_cond.fix),
		    alt_lim_desc, spd_lim_desc);
	} else if (seg->type == NAVPROC_SEG_TYPE_HDG_TO_MANUAL) {
		append_format(result, result_sz, "\tVM,H:%.01lf,T:%s%s%s\n",
		    seg->leg_cmd.hdg.hdg, dump_turn(seg->leg_cmd.hdg.turn),
		    alt_lim_desc, spd_lim_desc);
	} else {
		append_format(result, result_sz,
		    "\tVR,H:%.01lf,T:%s,N:%s(%lfx%lf),R:%.01lf%s%s\n",
		    seg->leg_cmd.hdg.hdg, dump_turn(seg->leg_cmd.hdg.turn),
		    FIX_PRINTF_ARG(&seg->term_cond.radial.navaid),
		    seg->term_cond.radial.radial, alt_lim_desc, spd_lim_desc);
	}
}

static bool_t
parse_proc_seg_line(const char *line, navproc_t *proc, const airport_t *arpt,
    const waypoint_db_t *wptdb, const navaid_db_t *navdb)
{
	char		*comps[24];
	size_t		num_comps;
	navproc_seg_t	seg;
	char		line_copy[256];

	STRLCPY_CHECK_ERROUT(line_copy, line);
	num_comps = explode_line(line_copy, ',', comps, 24);
	memset(&seg, 0, sizeof (seg));

	ASSERT(num_comps > 0);
	if (strcmp(comps[0], "AF") == 0) {
		if (!parse_AF_seg(comps, num_comps, &seg, arpt, navdb))
			goto errout;
	} else if (strcmp(comps[0], "CA") == 0) {
		if (!parse_CA_seg(comps, num_comps, &seg))
			goto errout;
	} else if (strcmp(comps[0], "CD") == 0) {
		if (!parse_CD_seg(comps, num_comps, &seg, arpt, navdb))
			goto errout;
	} else if (strcmp(comps[0], "CF") == 0) {
		if (!parse_CF_seg(comps, num_comps, &seg, arpt, navdb))
			goto errout;
	} else if (strcmp(comps[0], "CI") == 0) {
		if (!parse_CI_CR_seg(comps, num_comps, &seg, 1, arpt, navdb))
			goto errout;
	} else if (strcmp(comps[0], "CR") == 0) {
		if (!parse_CI_CR_seg(comps, num_comps, &seg, 0, arpt, navdb))
			goto errout;
	} else if (strcmp(comps[0], "DF") == 0) {
		if (!parse_DF_TF_seg(comps, num_comps, &seg, 1))
			goto errout;
	} else if (strcmp(comps[0], "FA") == 0) {
		if (!parse_FA_seg(comps, num_comps, &seg))
			goto errout;
	} else if (strcmp(comps[0], "FC") == 0) {
		if (!parse_FC_seg(comps, num_comps, &seg))
			goto errout;
	} else if (strcmp(comps[0], "FD") == 0) {
		if (!parse_FD_seg(comps, num_comps, &seg, arpt, navdb))
			goto errout;
	} else if (strcmp(comps[0], "FM") == 0) {
		if (!parse_FM_seg(comps, num_comps, &seg))
			goto errout;
	} else if (strcmp(comps[0], "HA") == 0) {
		if (!parse_HA_HF_HM_seg(comps, num_comps, &seg,
		    NAVPROC_SEG_TYPE_HOLD_TO_ALT))
			goto errout;
	} else if (strcmp(comps[0], "HF") == 0) {
		if (!parse_HA_HF_HM_seg(comps, num_comps, &seg,
		    NAVPROC_SEG_TYPE_HOLD_TO_FIX))
			goto errout;
	} else if (strcmp(comps[0], "HM") == 0) {
		if (!parse_HA_HF_HM_seg(comps, num_comps, &seg,
		    NAVPROC_SEG_TYPE_HOLD_TO_MANUAL))
			goto errout;
	} else if (strcmp(comps[0], "IF") == 0) {
		if (!parse_IF_seg(comps, num_comps, &seg))
			goto errout;
	} else if (strcmp(comps[0], "PI") == 0) {
		if (!parse_PI_seg(comps, num_comps, &seg, arpt, navdb))
			goto errout;
	} else if (strcmp(comps[0], "RF") == 0) {
		if (!parse_RF_seg(comps, num_comps, &seg, arpt, wptdb))
			goto errout;
	} else if (strcmp(comps[0], "TF") == 0) {
		if (!parse_DF_TF_seg(comps, num_comps, &seg, 0))
			goto errout;
	} else if (strcmp(comps[0], "VA") == 0) {
		if (!parse_VA_seg(comps, num_comps, &seg))
			goto errout;
	} else if (strcmp(comps[0], "VD") == 0) {
		if (!parse_VD_seg(comps, num_comps, &seg, arpt, navdb))
			goto errout;
	} else if (strcmp(comps[0], "VI") == 0) {
		if (!parse_VI_VM_VR_seg(comps, num_comps, &seg,
		    NAVPROC_SEG_TYPE_HDG_TO_INTCP, arpt, navdb))
			goto errout;
	} else if (strcmp(comps[0], "VM") == 0) {
		if (!parse_VI_VM_VR_seg(comps, num_comps, &seg,
		    NAVPROC_SEG_TYPE_HDG_TO_MANUAL, arpt, navdb))
			goto errout;
	} else if (strcmp(comps[0], "VR") == 0) {
		if (!parse_VI_VM_VR_seg(comps, num_comps, &seg,
		    NAVPROC_SEG_TYPE_HDG_TO_RADIAL, arpt, navdb))
			goto errout;
	} else {
		/* Unknown segment type */
		openfmc_log(OPENFMC_LOG_ERR, "Unknown procedure segment "
		    "type: %s", comps[0]);
		goto errout;
	}

	/* segment complete and validated, store it in the procedure */
	proc->num_segs++;
	proc->segs = realloc(proc->segs, sizeof (seg) * proc->num_segs);
	memcpy(&proc->segs[proc->num_segs - 1], &seg, sizeof (seg));

	return (B_TRUE);
errout:
	openfmc_log(OPENFMC_LOG_ERR, "Error parsing procedure segment line. "
	    "Offending line was: \"%s\".", line);
	return (B_FALSE);
}

/*
 * Returns the string name of a navproc_seg_type.
 */
const char *
navproc_seg_type2str(navproc_seg_type_t type)
{
	static const char *typenames[NAVPROC_SEG_TYPES] = {
		"AF",	/* NAVPROC_SEG_TYPE_ARC_TO_FIX		*/
		"CA",	/* NAVPROC_SEG_TYPE_CRS_TO_ALT		*/
		"CD",	/* NAVPROC_SEG_TYPE_CRS_TO_DME		*/
		"CF",	/* NAVPROC_SEG_TYPE_CRS_TO_FIX		*/
		"CI",	/* NAVPROC_SEG_TYPE_CRS_TO_INTCP	*/
		"CR",	/* NAVPROC_SEG_TYPE_CRS_TO_RADIAL	*/
		"DF",	/* NAVPROC_SEG_TYPE_DIR_TO_FIX		*/
		"FA",	/* NAVPROC_SEG_TYPE_FIX_TO_ALT		*/
		"FC",	/* NAVPROC_SEG_TYPE_FIX_TO_DIST		*/
		"FD",	/* NAVPROC_SEG_TYPE_FIX_TO_DME		*/
		"FM",	/* NAVPROC_SEG_TYPE_FIX_TO_MANUAL	*/
		"HA",	/* NAVPROC_SEG_TYPE_HOLD_TO_ALT		*/
		"HF",	/* NAVPROC_SEG_TYPE_HOLD_TO_FIX		*/
		"HM",	/* NAVPROC_SEG_TYPE_HOLD_TO_MANUAL	*/
		"IF",	/* NAVPROC_SEG_TYPE_INIT_FIX		*/
		"PI",	/* NAVPROC_SEG_TYPE_PROC_TURN		*/
		"RF",	/* NAVPROC_SEG_TYPE_RADIUS_ARC_TO_FIX	*/
		"TF",	/* NAVPROC_SEG_TYPE_TRK_TO_FIX		*/
		"VA",	/* NAVPROC_SEG_TYPE_HDG_TO_ALT		*/
		"VD",	/* NAVPROC_SEG_TYPE_HDG_TO_DME		*/
		"VI",	/* NAVPROC_SEG_TYPE_HDG_TO_INTCP	*/
		"VM",	/* NAVPROC_SEG_TYPE_HDG_TO_MANUAL	*/
		"VR"	/* NAVPROC_SEG_TYPE_HDG_TO_RADIAL	*/
	};
	ASSERT(type < NAVPROC_SEG_TYPES);
	return (typenames[type]);
}

/*
 * Returns the initial wpt of a navproc_seg_t. Not all navproc segments have
 * an initial point, so the passed seg must be one of:
 *	NAVPROC_SEG_TYPE_CRS_TO_FIX (with a non-NULL navaid set)
 *	NAVPROC_SEG_TYPE_FIX_TO_ALT
 *	NAVPROC_SEG_TYPE_FIX_TO_DIST
 *	NAVPROC_SEG_TYPE_FIX_TO_DME
 *	NAVPROC_SEG_TYPE_FIX_TO_MANUAL
 *	NAVPROC_SEG_TYPE_INIT_FIX
 *	NAVPROC_SEG_TYPE_PROC_TURN
 *	NAVPROC_SEG_TYPE_HOLD_TO_ALT
 *	NAVPROC_SEG_TYPE_HOLD_TO_FIX
 *	NAVPROC_SEG_TYPE_HOLD_TO_MANUAL
 * Navproc segments not of this type will return null_wpt instead.
 * All non-SID procedures must start with a navproc seg that has a definite
 * wpt (otherwise it can't be connected to a preceding navproc_t).
 */
const wpt_t *
navproc_seg_get_start_wpt(const navproc_seg_t *seg)
{
	switch (seg->type) {
	case NAVPROC_SEG_TYPE_CRS_TO_FIX:
		return (&seg->term_cond.fix);
	case NAVPROC_SEG_TYPE_FIX_TO_DIST:
	case NAVPROC_SEG_TYPE_FIX_TO_DME:
	case NAVPROC_SEG_TYPE_FIX_TO_MANUAL:
		return (&seg->leg_cmd.fix_crs.fix);
	case NAVPROC_SEG_TYPE_INIT_FIX:
	case NAVPROC_SEG_TYPE_FIX_TO_ALT:
		return (&seg->leg_cmd.fix);
	case NAVPROC_SEG_TYPE_PROC_TURN:
		return (&seg->leg_cmd.proc_turn.startpt);
	case NAVPROC_SEG_TYPE_HOLD_TO_ALT:
	case NAVPROC_SEG_TYPE_HOLD_TO_FIX:
	case NAVPROC_SEG_TYPE_HOLD_TO_MANUAL:
		return (&seg->leg_cmd.hold.wpt);
	default:
		return (&null_wpt);
	}
}

/*
 * Returns a pointer to the end wpt of a particular navproc segment. If the
 * segment doesn't end in a wpt, returns `null_wpt' instead.
 */
const wpt_t *
navproc_seg_get_end_wpt(const navproc_seg_t *seg)
{
	switch (seg->type) {
	case NAVPROC_SEG_TYPE_ARC_TO_FIX:
	case NAVPROC_SEG_TYPE_CRS_TO_FIX:
	case NAVPROC_SEG_TYPE_DIR_TO_FIX:
	case NAVPROC_SEG_TYPE_RADIUS_ARC_TO_FIX:
	case NAVPROC_SEG_TYPE_TRK_TO_FIX:
	case NAVPROC_SEG_TYPE_HDG_TO_INTCP:
		return (&seg->term_cond.fix);
	case NAVPROC_SEG_TYPE_INIT_FIX:
		return (&seg->leg_cmd.fix);
	case NAVPROC_SEG_TYPE_HOLD_TO_ALT:
	case NAVPROC_SEG_TYPE_HOLD_TO_FIX:
	case NAVPROC_SEG_TYPE_HOLD_TO_MANUAL:
		ASSERT(!IS_NULL_WPT(&seg->leg_cmd.hold.wpt));
		return (&seg->leg_cmd.hold.wpt);
	default:
		return (&null_wpt);
	}
}

/*
 * Sets the end wpt of `seg' to `wpt'. If the seg is of a type that doesn't
 * take fixes, causes an assertion failure.
 */
void
navproc_seg_set_end_wpt(navproc_seg_t *seg, const wpt_t *wpt)
{
	switch (seg->type) {
	case NAVPROC_SEG_TYPE_ARC_TO_FIX:
	case NAVPROC_SEG_TYPE_CRS_TO_FIX:
	case NAVPROC_SEG_TYPE_DIR_TO_FIX:
	case NAVPROC_SEG_TYPE_RADIUS_ARC_TO_FIX:
	case NAVPROC_SEG_TYPE_TRK_TO_FIX:
	case NAVPROC_SEG_TYPE_HDG_TO_INTCP:
		seg->term_cond.fix = *wpt;
		break;
	case NAVPROC_SEG_TYPE_INIT_FIX:
		seg->leg_cmd.fix = *wpt;
		break;
	case NAVPROC_SEG_TYPE_HOLD_TO_ALT:
	case NAVPROC_SEG_TYPE_HOLD_TO_FIX:
	case NAVPROC_SEG_TYPE_HOLD_TO_MANUAL:
		seg->leg_cmd.hold.wpt = *wpt;
		break;
	default:
		assert(0);
	}
}

/*
 * Returns a malloc'd string describing the specified navproc. The caller
 * is responsible for freeing the string.
 */
char *
navproc_seg_get_descr(const navproc_seg_t *seg)
{
	char *result = NULL;
	size_t result_sz = 0;

	ASSERT(seg->type < NAVPROC_SEG_TYPES);
	navproc_seg_dump_funcs[seg->type](&result, &result_sz, seg);

	return (result);
}

/*
 * Returns the start wpt of a procedure. All procedures have a definite start
 * wpt defined as:
 *	1) the departure runway threshold for NAVPROC_TYPE_SID procedures
 *	2) the initial segment wpt's start wpt for all other procedure types
 */
wpt_t
navproc_get_start_wpt(const navproc_t *proc)
{
	wpt_t wpt;

	switch (proc->type) {
	case NAVPROC_TYPE_SID: {
		CTASSERT(sizeof (wpt.name) >= sizeof (proc->rwy->ID));
		(void) strcpy(wpt.name, proc->rwy->ID);
		wpt.pos = GEO3_TO_GEO2(proc->rwy->thr_pos);
		break;
	}
	default:
		ASSERT(proc->num_segs != 0);
		wpt = *navproc_seg_get_start_wpt(&proc->segs[0]);
		ASSERT(!IS_NULL_WPT(&wpt));
		break;
	}

	return (wpt);
}

const wpt_t *
navproc_get_end_wpt(const navproc_t *proc)
{
	return (navproc_seg_get_end_wpt(&proc->segs[proc->num_segs - 1]));
}

static int
parse_proc(FILE *fp, size_t *line_num, navproc_t *proc, airport_t *arpt,
    const waypoint_db_t *wptdb, const navaid_db_t *navdb)
{
	size_t		line_cap = 0;
	ssize_t		line_len = 0;
	char		*line = NULL;
	char		line_copy[128];
	char		*comps[8];
	size_t		num_comps;

	memset(proc, 0, sizeof (*proc));
	while ((line_len = parser_get_next_line(fp, &line, &line_cap,
	    line_num)) == 0)
		;
	if (line_len == -1) {
		/* EOF */
		return (0);
	}
	STRLCPY_CHECK_ERROUT(line_copy, line);
	num_comps = explode_line(line_copy, ',', comps, 8);
	ASSERT(num_comps != 0);

	if (strcmp(comps[0], "SID") == 0) {
		if (!parse_sid_proc_line(arpt, comps, num_comps, proc))
			goto errout;
	} else if (strcmp(comps[0], "STAR") == 0) {
		if (!parse_star_proc_line(arpt, comps, num_comps, proc))
			goto errout;
	} else if (strcmp(comps[0], "APPTR") == 0) {
		if (!parse_apptr_proc_line(arpt, comps, num_comps, proc))
			goto errout;
	} else if (strcmp(comps[0], "FINAL") == 0) {
		if (!parse_final_proc_line(arpt, comps, num_comps, proc))
			goto errout;
	} else {
		goto errout;
	}

	while ((line_len = parser_get_next_line(fp, &line, &line_cap,
	    line_num)) != -1) {
		if (line_len == 0)
			break;
		if (!parse_proc_seg_line(line, proc, arpt, wptdb, navdb))
			goto errout;
	}
	if (proc->num_segs == 0) {
		openfmc_log(OPENFMC_LOG_ERR, "Error parsing %s procedure "
		    "\"%s\": no segments found.",
		    navproc_type_to_str[proc->type], proc->name);
		goto errout;
	}
	if (proc->type != NAVPROC_TYPE_SID &&
	    IS_NULL_WPT(navproc_seg_get_start_wpt(&proc->segs[0]))) {
		openfmc_log(OPENFMC_LOG_ERR, "Error parsing %s procedure "
		    "\"%s\": procedure doesn't start with appropriate leg.",
		    navproc_type_to_str[proc->type], proc->name);
		goto errout;
	}

	proc->arpt = arpt;

	free(line);

	return (1);
errout:
	free(proc->segs);
	free(line);
	return (-1);
}

static void
parse_proc_file(FILE *fp, airport_t *arpt, const waypoint_db_t *wptdb,
    const navaid_db_t *navdb)
{
	navproc_t	proc;
	int		n;
	size_t		line_num = 0;

	while ((n = parse_proc(fp, &line_num, &proc, arpt, wptdb,
	    navdb)) != 0) {
		if (n == -1) {
			/* broken procedure, skip over it */
			continue;
		}
		arpt->num_procs++;
		arpt->procs = realloc(arpt->procs, sizeof (navproc_t) *
		    arpt->num_procs);
		(void) memcpy(&arpt->procs[arpt->num_procs - 1], &proc,
		    sizeof (proc));
	}
}

airport_t *
airport_open(const char *arpt_icao, const char *navdata_dir,
    const waypoint_db_t *wptdb, const navaid_db_t *navdb)
{
	airport_t	*arpt;
	FILE		*arpt_fp = NULL, *proc_fp = NULL;
	char		*arpt_fname = NULL;
	char		*proc_fname = NULL;
	ssize_t		line_len = 0;
	size_t		line_cap = 0, line_num = 0;
	char		*line = NULL;

	arpt = calloc(sizeof (*arpt), 1);
	if (!arpt)
		return (NULL);

	ASSERT(strlen(arpt_icao) == 4);
	strcpy(arpt->icao, arpt_icao);

	/* Open Airports.txt */
	arpt_fname = malloc(strlen(navdata_dir) +
	    strlen(PATHSEP "Airports.txt") + 1);
	sprintf(arpt_fname, "%s" PATHSEP "Airports.txt", navdata_dir);
	arpt_fp = fopen(arpt_fname, "r");
	if (arpt_fp == NULL) {
		openfmc_log(OPENFMC_LOG_ERR, "Can't open %s: %s", arpt_fname,
		    strerror(errno));
		goto errout;
	}

	/* Locate airport starting line & parse it */
	while ((line_len = parser_get_next_line(arpt_fp, &line, &line_cap,
	    &line_num)) != -1) {
		if (line_len == 0)
			continue;
		if (parse_arpt_line(line, arpt))
			break;
	}
	if (line_len == -1) {
		/* error reading/locating airport */
		openfmc_log(OPENFMC_LOG_ERR, "Error opening airport %s: "
		    "airport not found.", arpt_icao);
		goto errout;
	}

	/* airport found, read non-empty runway lines */
	while ((line_len = parser_get_next_line(arpt_fp, &line, &line_cap,
	    &line_num)) > 0) {
		if (line_len == 0)
			break;
		arpt->num_rwys++;
		arpt->rwys = realloc(arpt->rwys, sizeof (*arpt->rwys) *
		    arpt->num_rwys);
		memset(&arpt->rwys[arpt->num_rwys - 1], 0, sizeof (runway_t));
		if (!parse_rwy_line(line, &arpt->rwys[arpt->num_rwys - 1],
		    arpt))
			goto errout;
	}

	/* airport must have at least one runway */
	if (arpt->num_rwys == 0) {
		openfmc_log(OPENFMC_LOG_ERR, "Error opening airport %s: "
		    "airport has no runways.", arpt_icao);
		goto errout;
	}

	/* Try to read any procedures we may have available for this airport */
	proc_fname = malloc(strlen(navdata_dir) +
	    strlen(PATHSEP "Proc" PATHSEP "XXXX.txt") + 1);
	sprintf(proc_fname, "%s" PATHSEP "Proc" PATHSEP "%s.txt",
	    navdata_dir, arpt->icao);
	proc_fp = fopen(proc_fname, "r");
	if (proc_fp != NULL)
		parse_proc_file(proc_fp, arpt, wptdb, navdb);

	free(line);
	free(arpt_fname);
	free(proc_fname);
	(void) fclose(arpt_fp);
	if (proc_fp)
		(void) fclose(proc_fp);

	return (arpt);
errout:
	if (arpt)
		airport_close(arpt);
	free(arpt_fname);
	free(proc_fname);
	if (arpt_fp)
		(void) fclose(arpt_fp);
	if (proc_fp)
		(void) fclose(proc_fp);
	free(line);

	return (NULL);
}

void
airport_close(airport_t *arpt)
{
	free(arpt->rwys);
	for (unsigned i = 0; i < arpt->num_procs; i++) {
		navproc_t *proc = &arpt->procs[i];
		free(proc->segs);
	}
	free(arpt->procs);
	free(arpt->gates);
	free(arpt);
}

char *
airport_dump(const airport_t *arpt)
{
	char	*result = NULL;
	size_t	result_sz = 0;

	append_format(&result, &result_sz,
	    "Airport:\n"
	    "  name: \"%s\"\n"
	    "  ICAO: %s\n"
	    "  refpt: %lf x %lf\n"
	    "  TA: %u\n"
	    "  TL: %u\n"
	    "  true_hdg: %d\n"
	    "  longest_rwy: %u\n\n"
	    "  Runways (%u):\n"
	    "    RWY hdg   len wide LOC    LOCfreq LOCcrs    thr_lat "
	    "     thr_lon gp_angle\n"
	    "    --- --- ----- ---- --- ---------- ------ ---------- "
	    "------------ --------\n",
	    arpt->name, arpt->icao, arpt->refpt.lat, arpt->refpt.lon,
	    arpt->TA, arpt->TL, arpt->true_hdg, arpt->longest_rwy,
	    arpt->num_rwys);

	for (unsigned i = 0; i < arpt->num_rwys; i++) {
		const runway_t *rwy = &arpt->rwys[i];
		append_format(&result, &result_sz,
		    "    %3s %3u %5u %4u %3s %6.2lf MHz %6u %10.6lf %12.6lf "
		    "%8.1lf\n",
		    rwy->ID, rwy->hdg, rwy->length, rwy->width,
		    rwy->loc_avail ? "yes" : "no",
		    rwy->loc_freq / 1000000.0, rwy->loc_fcrs, rwy->thr_pos.lat,
		    rwy->thr_pos.lon, rwy->gp_angle);
	}

	append_format(&result, &result_sz, "\n  Procedures (%u)\n",
	    arpt->num_procs);
	for (unsigned i = 0; i < arpt->num_procs; i++) {
		const navproc_t *proc = &arpt->procs[i];
		char final_type[32];

		if (proc->type == NAVPROC_TYPE_FINAL)
			sprintf(final_type, "%s",
			    navproc_final_types_to_str[proc->final_type]);
		else
			final_type[0] = 0;
		append_format(&result, &result_sz,
		    "    %-7s %6s%7s%s %s\n"
		    "      Segments (%u/%u):\n",
		    navproc_type_to_str[proc->type], proc->name, final_type,
		    proc->rwy->ID, proc->tr_name,
		    proc->num_segs, proc->num_main_segs);
		for (unsigned j = 0; j < proc->num_segs; j++)
			navproc_seg_dump_funcs[proc->segs[j].type](&result,
			    &result_sz, &proc->segs[j]);
		append_format(&result, &result_sz, "\n");
	}

	append_format(&result, &result_sz, "  Gates (%u):\n", arpt->num_gates);
	for (unsigned i = 0; i < arpt->num_gates; i++) {
		const wpt_t *gate = &arpt->gates[i];
		append_format(&result, &result_sz, "    %s  [%lf x %lf]\n",
		    gate->name, gate->pos.lat, gate->pos.lon);
	}

	return (result);
}

/*
 * Locates and returns a runway structure at an airport based on runway ID.
 */
const runway_t *
airport_find_rwy_by_ID(const airport_t *arpt, const char *rwy_ID)
{
	for (unsigned i = 0; i < arpt->num_rwys; i++) {
		if (strcmp(arpt->rwys[i].ID, rwy_ID) == 0)
			return (&arpt->rwys[i]);
	}
	return (NULL);
}

/*
 * Returns the position of a gate at an airport based on gate ID.
 */
geo_pos2_t
airport_find_gate_pos(const airport_t *arpt, const char *gate_ID)
{
	for (unsigned i = 0; i < arpt->num_gates; i++) {
		if (strcmp(arpt->gates[i].name, gate_ID) == 0)
			return arpt->gates[i].pos;
	}
	return (NULL_GEO_POS2);
}
