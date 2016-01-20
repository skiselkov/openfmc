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

#ifndef	_OPENFMC_FMS_H_
#define	_OPENFMC_FMS_H_

#include <regex.h>

#include "airac.h"
#include "perf.h"
#include "wmm.h"

typedef enum {
	FLT_PHASE_TO,	/* Takeoff */
	FLT_PHASE_CLB,	/* Climb to CRZ ALT */
	FLT_PHASE_CRZ,	/* Cruise */
	FLT_PHASE_DES,	/* Descent from CRZ ALT to runway */
	FLT_PHASE_GA	/* Go-around */
} flt_phase_t;

typedef struct {
	time_t		valid_from;
	time_t		valid_to;
	unsigned	airac_cycle;

	char		*navdata_dir;
	airway_db_t	*awydb;
	waypoint_db_t	*wptdb;
	navaid_db_t	*navaiddb;

	char		*wmm_file;
	wmm_t		*wmm;
} fms_navdb_t;

typedef struct {
	fms_navdb_t	*navdb;

	int		wpt_seq_num;

	struct {
		regex_t	*wptname;		/* 'DOT', 'ALPHA', etc */
		regex_t	*arpticao;		/* 'KJFK', 'KMIA', etc */

		regex_t	*geo_nw_blw100;		/* 5010N = N50 W010 */
		regex_t	*geo_nw_abv100;		/* 50N10 = N50 W110 */

		regex_t	*geo_ne_blw100;		/* 5010E = N50 E010 */
		regex_t	*geo_ne_abv100;		/* 50E10 = N50 E110 */

		regex_t	*geo_sw_blw100;		/* 5010W = S50 W010 */
		regex_t	*geo_sw_abv100;		/* 50W10 = S50 W110 */

		regex_t	*geo_se_blw100;		/* 5010S = S50 E010 */
		regex_t	*geo_se_abv100;		/* 50S10 = S50 E110 */

		regex_t	*geo_long;		/* N47W008 */
		regex_t	*geo_detailed;		/* N4715.4W00803.4 */

		regex_t	*geo_report;		/* W060-10 */

		/* SEA330/10 = SEA VOR 330 radial, 10 DME */
		regex_t	*radial_dme;
		/* SEA330/OLM020 = intersection SEA 330 & OLM 020 radials */
		regex_t	*radial_isect;

		regex_t	*along_trk;		/* VAMPS/25, ELN/-30 */
	} regex;

	acft_perf_t	*acft;
	flt_perf_t	*flt;
} fms_t;

fms_t *fms_new(const char *navdata_dir, const char *wmm_file,
    const char *acft_perf_file);
void fms_destroy(fms_t *fms);

fms_navdb_t *fms_navdb_open(const char *navdata_dir, const char *wmm_file);
void fms_navdb_close(fms_navdb_t *navdb);

bool_t navdb_is_current(const fms_navdb_t *navdb);
bool_t navdata_is_current(const char *navdata_dir);

wpt_t *fms_wpt_name_decode(const char *name, fms_t *fms, size_t *num,
    bool_t *is_wpt_seq);

const acft_perf_t *fms_acft_perf(const fms_t *fms);
flt_perf_t *fms_flt_perf(fms_t *fms);

#endif	/* _OPENFMC_FMS_H_ */
