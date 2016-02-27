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

#ifndef	_OPENFMC_AIRAC_H_
#define	_OPENFMC_AIRAC_H_

#include "geom.h"
#include "htbl.h"
#include "types.h"

#ifdef	__cplusplus
extern "C" {
#endif

#define	NAV_NAME_LEN		8
#define	ICAO_NAME_LEN		4
#define	ICAO_COUNTRY_CODE_LEN	2
#define	RWY_ID_LEN		4

/* Forward declarations */
typedef struct runway_s runway_t;
typedef struct airport_s airport_t;

/* Airway structures */

typedef struct {
	char		name[NAV_NAME_LEN];
	char		icao_country_code[3];
	geo_pos2_t	pos;
} wpt_t;
const wpt_t null_wpt;
#define	WPT_EQ(f1, f2)	\
	(memcmp((f1)->name, (f2)->name, sizeof ((f1)->name)) == 0 && \
	(f1)->pos.lat == (f2)->pos.lat && (f1)->pos.lon == (f2)->pos.lon && \
	!IS_NULL_WPT((f1)))
#define	WPT_EQ_POS(f1, f2) \
	(!IS_NULL_WPT((f1)) && (f1)->pos.lat == (f2)->pos.lat && \
	(f1)->pos.lon == (f2)->pos.lon)
#define	IS_NULL_WPT(f)	(memcmp((f), &null_wpt, sizeof (wpt_t)) == 0)

typedef struct {
	wpt_t		endpt[2];
} airway_seg_t;

typedef struct {
	char		name[NAV_NAME_LEN];
	unsigned	num_segs;
	airway_seg_t	*segs;
} airway_t;

typedef struct {
	htbl_t		by_awy_name;
	htbl_t		by_fix_name;
} airway_db_t;

typedef struct {
	htbl_t		by_name;
} waypoint_db_t;

airway_db_t *airway_db_open(const char *navdata_dir, size_t num_waypoints);
void airway_db_close(airway_db_t *db);
char *airway_db_dump(const airway_db_t *db, bool_t by_awy_name);

/* Airway lookup */
const airway_t *airway_db_lookup(const airway_db_t *db, const char *awyname,
    const wpt_t *start_wpt, const char *end_wpt_name, const wpt_t **endfixpp);
const wpt_t *airway_db_lookup_awy_intersection(const airway_db_t *db,
    const char *awy1_name, const char *awy1_start_wpt_name,
    const char *awy2_name);
bool_t airway_db_wpt_on_awy(const airway_db_t *db, const wpt_t *wpt,
    const char *awyname);

waypoint_db_t *waypoint_db_open(const char *navdata_dir);
void waypoint_db_close(waypoint_db_t *db);
char *waypoint_db_dump(const waypoint_db_t *db);


/* Navaid structures */

typedef enum {
	NAVAID_TYPE_VOR		= 1 << 0,	/* VHF Omni Range (VOR) */
	NAVAID_TYPE_VORDME	= 1 << 1,	/* VOR w/ DME */
	NAVAID_TYPE_LOC		= 1 << 4,	/* VHF Localizer (LOC) */
	NAVAID_TYPE_LOCDME	= 1 << 5,	/* LOC w/ DME */
	NAVAID_TYPE_NDB		= 1 << 6,	/* Non-Directional Beacon */
	NAVAID_TYPE_TACAN	= 1 << 7,	/* Military TACAN 133-136 MHz */
	NAVAID_TYPE_UNKNOWN	= 1 << 8,	/* Known position w/o type */
	NAVAID_TYPE_ANY_VOR	= NAVAID_TYPE_VOR | NAVAID_TYPE_VORDME,
	NAVAID_TYPE_ANY_LOC	= NAVAID_TYPE_LOC | NAVAID_TYPE_LOCDME,
	NAVAID_TYPE_ANY		= ((NAVAID_TYPE_UNKNOWN << 1) - 1)
} navaid_type_t;

typedef struct {
	char		ID[NAV_NAME_LEN];
	char		name[16];
	char		icao_country_code[ICAO_COUNTRY_CODE_LEN + 1];
	geo_pos3_t	pos;
	navaid_type_t	type;
	unsigned	freq;		/* in Hz */
} navaid_t;

typedef struct {
	htbl_t		by_id;
} navaid_db_t;

navaid_db_t *navaid_db_open(const char *navdata_dir);
void navaid_db_close(navaid_db_t *db);
char *navaid_db_dump(const navaid_db_t *db);


/* Procedure structures */

typedef enum {
	NAVPROC_TYPE_SID,		/* <runway> -> SID/SID_COMMON */
	NAVPROC_TYPE_SID_COMMON,	/* common SID portion, "ALL" trans */
	NAVPROC_TYPE_SID_TRANS,		/* SID -> <fix> */
	NAVPROC_TYPE_STAR,		/* STAR -> <rwy> */
	NAVPROC_TYPE_STAR_COMMON,	/* common STAR portion, "ALL" trans */
	NAVPROC_TYPE_STAR_TRANS,	/* <fix> -> STAR/STAR_COMMON */
	NAVPROC_TYPE_FINAL_TRANS,	/* appr trans, <fix> -> <rwy> */
	NAVPROC_TYPE_FINAL,		/* final appr: -> <rwy> */
	NAVPROC_TYPES
} navproc_type_t;

typedef enum {
	NAVPROC_SEG_TYPE_ARC_TO_FIX,		/* AF */
	NAVPROC_SEG_TYPE_CRS_TO_ALT,		/* CA */
	NAVPROC_SEG_TYPE_CRS_TO_DME,		/* CD */
	NAVPROC_SEG_TYPE_CRS_TO_FIX,		/* CF */
	NAVPROC_SEG_TYPE_CRS_TO_INTCP,		/* CI */
	NAVPROC_SEG_TYPE_CRS_TO_RADIAL,		/* CR */
	NAVPROC_SEG_TYPE_DIR_TO_FIX,		/* DF */
	NAVPROC_SEG_TYPE_FIX_TO_ALT,		/* FA */
	NAVPROC_SEG_TYPE_FIX_TO_DIST,		/* FC */
	NAVPROC_SEG_TYPE_FIX_TO_DME,		/* FD */
	NAVPROC_SEG_TYPE_FIX_TO_MANUAL,		/* FM */
	NAVPROC_SEG_TYPE_HOLD_TO_ALT,		/* HA */
	NAVPROC_SEG_TYPE_HOLD_TO_FIX,		/* HF */
	NAVPROC_SEG_TYPE_HOLD_TO_MANUAL,	/* HM */
	NAVPROC_SEG_TYPE_INIT_FIX,		/* IF */
	NAVPROC_SEG_TYPE_PROC_TURN,		/* PI */
	NAVPROC_SEG_TYPE_RADIUS_ARC_TO_FIX,	/* RF */
	NAVPROC_SEG_TYPE_TRK_TO_FIX,		/* TF */
	NAVPROC_SEG_TYPE_HDG_TO_ALT,		/* VA */
	NAVPROC_SEG_TYPE_HDG_TO_DME,		/* VD */
	NAVPROC_SEG_TYPE_HDG_TO_INTCP,		/* VI */
	NAVPROC_SEG_TYPE_HDG_TO_MANUAL,		/* VM */
	NAVPROC_SEG_TYPE_HDG_TO_RADIAL,		/* VR */
	NAVPROC_SEG_TYPES
} navproc_seg_type_t;

typedef enum {
	ALT_LIM_NONE = 0,	/* ALT unconstrained */
	ALT_LIM_AT,		/* ALT == alt1 */
	ALT_LIM_AT_OR_ABV,	/* ALT >= alt1 */
	ALT_LIM_AT_OR_BLW,	/* ALT <= alt1 */
	ALT_LIM_BETWEEN		/* alt1 >= ALT && ALT >= alt2 */
} alt_lim_type_t;

typedef enum {
	SPD_LIM_NONE = 0,	/* SPD unconstrained */
	SPD_LIM_AT		/* SPD == spd1 */
} spd_lim_type_t;

typedef struct {
	alt_lim_type_t	type;
	unsigned	alt1;
	unsigned	alt2;
} alt_lim_t;

typedef struct {
	spd_lim_type_t	type;
	unsigned	spd1;
} spd_lim_t;

typedef enum {
	TURN_ANY,
	TURN_LEFT,
	TURN_RIGHT
} turn_t;

typedef struct {
	navproc_seg_type_t	type;

	/* Segment leg */
	union {
		struct {	/* CA, CD, CI, CR, VA, VD, VI, VM, VR */
			double	hdg;
			turn_t	turn;
		} hdg;
		struct {	/* FA, FC, FD, FM */
			wpt_t	fix;
			double	crs;
		} fix_crs;
		/*
		 * CF legs can omit the navaid portion if it is unavailable
		 * or unreliable, provided that the previous leg ends in a
		 * definite fix.
		 */
		struct {	/* CF */
			wpt_t		navaid;
			double		crs;
			turn_t		turn;
		} navaid_crs;
		struct {	/* AF */
			wpt_t		navaid;
			double		start_radial;
			double		end_radial;
			double		radius;
			bool_t		cw;
		} dme_arc;
		struct {	/* RF */
			wpt_t		ctr_wpt;
			double		radius;
			bool_t		cw;	/* clockwise or counter-CW */
		} radius_arc;
		wpt_t			fix;	/* IF */
		struct {	/* HA, HF, HM */
			wpt_t		wpt;
			double		inbd_crs;
			double		leg_len;
			bool_t		turn_right;
		} hold;
		struct {	/* PI */
			wpt_t		startpt;
			double		outbd_radial;
			double		outbd_turn_hdg;
			double		max_excrs_dist;
			double		max_excrs_time;
			bool_t		turn_right;
			wpt_t		navaid;
		} proc_turn;
	} leg_cmd;

	/* Segment termination condition */
	union {
		wpt_t			fix;	/* AF, CF, DF, RF, TF, VI */
		alt_lim_t		alt;	/* CA, FA, HA, VA */
		struct {			/* CR, CI (optional), VR */
			wpt_t		navaid;
			double		radial;
		} radial;
		struct {			/* CD, FD, VD */
			wpt_t		navaid;
			double		dist;
		} dme;
		double			dist;	/* FC */
	} term_cond;

	/* Generic segment constraints */
	spd_lim_t	spd_lim;
	alt_lim_t	alt_lim;
	bool_t		ovrfly;
} navproc_seg_t;

typedef enum {
	NAVPROC_FINAL_ILS,		/* type: I subtype: I */
	NAVPROC_FINAL_VOR,		/* type: D subtype: S or V */
	NAVPROC_FINAL_VORDME,		/* type: D subtype: D */
	NAVPROC_FINAL_LOC,		/* type: D subtype: L */
	NAVPROC_FINAL_LOCBC,		/* type: D subtype: B */
	NAVPROC_FINAL_TACAN,		/* type: D subtype: T */
	NAVPROC_FINAL_NDB,		/* type: N subtype: N */
	NAVPROC_FINAL_NDBDME,		/* type: N subtype: Q */
	NAVPROC_FINAL_LDA,		/* type: C subtype: X */
	NAVPROC_FINAL_SDF,		/* type: D subtype: U */
	NAVPROC_FINAL_IGS,		/* type: G subtype: G */
	NAVPROC_FINAL_GLS,		/* type: G subtype: J */
	NAVPROC_FINAL_GPS,		/* type: G subtype: P */
	NAVPROC_FINAL_RNAV,		/* type: G subtype: R */
	NAVPROC_FINAL_TYPES
} navproc_final_t;

typedef struct navproc_s {
	navproc_type_t	type;
	char		name[NAV_NAME_LEN];
	airport_t	*arpt;		/* backref to parent airport_t */
	char		tr_name[NAV_NAME_LEN];
	const runway_t	*rwy;
	unsigned	num_segs;
	navproc_seg_t	*segs;
	/* number of main procedure segments, remainder is for go-around */
	unsigned	num_main_segs;
	navproc_final_t	final_type;
} navproc_t;

const char *navproc_seg_type2str(navproc_seg_type_t type);
const wpt_t *navproc_seg_get_start_wpt(const navproc_seg_t *seg);
const wpt_t *navproc_seg_get_end_wpt(const navproc_seg_t *seg);
void navproc_seg_set_end_wpt(navproc_seg_t *seg, const wpt_t *fix);
char *navproc_seg_get_descr(const navproc_seg_t *seg);

wpt_t navproc_get_start_wpt(const navproc_t *proc);
const wpt_t *navproc_get_end_wpt(const navproc_t *proc);

/* Airport structures */

struct runway_s {
	char		ID[RWY_ID_LEN + 1];
	airport_t	*arpt;		/* backref to parent airport_t */
	unsigned	hdg;
	unsigned	length;
	unsigned	width;
	bool_t		loc_avail;
	unsigned	loc_freq;	/* in Hz */
	unsigned	loc_fcrs;
	geo_pos3_t	thr_pos;
	double		gp_angle;
};

struct airport_s {
	char		name[32];
	char		icao[ICAO_NAME_LEN + 1];
	geo_pos3_t	refpt;
	unsigned	TA;
	unsigned	TL;
	unsigned	longest_rwy;
	unsigned	num_rwys;
	runway_t	*rwys;
	unsigned	num_procs;
	navproc_t	*procs;
	unsigned	num_gates;
	wpt_t		*gates;
	bool_t		true_hdg;
};

airport_t *airport_open(const char *arpt_icao, const char *navdata_dir,
    const waypoint_db_t *wptdb, const navaid_db_t *navdb);
void airport_close(airport_t *arpt);
char *airport_dump(const airport_t *arpt);

const runway_t *airport_find_rwy_by_ID(const airport_t *arpt,
    const char *rwy_ID);
geo_pos2_t airport_find_gate_pos(const airport_t *arpt, const char *gate_ID);

#ifdef	__cplusplus
}
#endif

#endif	/* _OPENFMC_AIRAC_H_ */
