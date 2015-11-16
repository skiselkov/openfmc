#ifndef	_OPENFMC_AIRAC_H_
#define	_OPENFMC_AIRAC_H_

#include "geom.h"

#define	NAV_NAME_LEN		8
#define	ICAO_NAME_LEN		4
#define	ICAO_COUNTRY_CODE_LEN	2
#define	RWY_ID_LEN		3

/* Airway structures */

typedef struct {
	char		name[NAV_NAME_LEN];
	char		icao_country_code[3];
	geo_pos_2d_t	pos;
} fix_t;

typedef struct {
	fix_t		endpts[2];
} airway_seg_t;

typedef struct {
	char		name[NAV_NAME_LEN];
	unsigned	num_segs;
	airway_seg_t	*segs;
} airway_t;

/* Navaid structures */

typedef enum {
	NAVAID_TYPE_VORDME,	/* VHF Omni Range + DME */
	NAVAID_TYPE_VORTAC,	/* VHF Omni Range + TACAN */
	NAVAID_TYPE_VOR,	/* VHF Omni Range only */
	NAVAID_TYPE_NDB,	/* Non-Directional Beacon */
	NAVAID_TYPE_LOCBC,	/* VHF Localizer /w back-course beam */
	NAVAID_TYPE_LOC		/* VHF Localizer */
} navaid_type_t;

typedef struct {
	char		name[NAV_NAME_LEN];
	char		icao_country_code[ICAO_COUNTRY_CODE_LEN + 1];
	navaid_type_t	type;
	double		freq;
	geo_pos_3d_t	pos;
} navaid_t;

/* Procedure structures */

typedef enum {
	NAVPROC_TYPE_SID,		/* <runway> -> SID/SID_COMMON */
	NAVPROC_TYPE_SID_COMMON,	/* common SID portion, "ALL" trans */
	NAVPROC_TYPE_SID_TRANS,		/* SID -> <fix> */
	NAVPROC_TYPE_STAR,		/* STAR -> <rwy> */
	NAVPROC_TYPE_STAR_COMMON,	/* common STAR portion, "ALL" trans */
	NAVPROC_TYPE_STAR_TRANS,	/* <fix> -> STAR/STAR_COMMON */
	NAVPROC_TYPE_FINAL_TRANS,	/* appr trans, <fix> -> <rwy> */
	NAVPROC_TYPE_FINAL		/* final appr: -> <rwy> */
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
	NAVPROC_SEG_TYPE_HDG_TO_RADIAL		/* VR */
} navproc_seg_type_t;

typedef enum {
	ALT_CONSTR_NONE = 0,	/* ALT unconstrained */
	ALT_CONSTR_AT,		/* ALT == alt1 */
	ALT_CONSTR_AT_OR_ABV,	/* ALT >= alt1 */
	ALT_CONSTR_AT_OR_BLW,	/* ALT <= alt1 */
	ALT_CONSTR_BETWEEN,	/* alt1 >= ALT && ALT >= alt2 */
} alt_type_t;

typedef enum {
	SPD_CONSTR_NONE = 0,
	SPD_CONSTR_AT_OR_BLW	/* SPD <= spd1 */
} spd_type_t;

typedef struct {
	alt_type_t	type;
	unsigned	alt1;
	unsigned	alt2;
} alt_constr_t;

typedef struct {
	spd_type_t	type;
	unsigned	spd1;
} spd_constr_t;

typedef struct navproc_seg_s {
	navproc_seg_type_t	type;

	/* Segment leg */
	union {
		double		hdg;		/* VA, VD, VI, VM, VR */
		double		crs;		/* CA, CD, CF, CI, CR */
		struct {			/* AF */
			char	navaid[NAV_NAME_LEN];
			double	start_radial;
			double	end_radial;
			double	radius;
		} dme_arc;
		struct {			/* RF */
			char	navaid[NAV_NAME_LEN];
			double	end_radial;
			double	radius;
			int	cw;		/* clockwise or counter-CW */
		} radius_arc;
		fix_t		fix;		/* FA, IF */
		struct {			/* FC, FD */
			fix_t	fix;
			double	leg_crs;
			double	leg_len;
		} dist;
		struct {			/* HA, HF, HM */
			fix_t	fix;
			double	inbd_crs;
			double	leg_len;
		} hold;
		struct {			/* PI */
			fix_t	startpt;
			double	outbd_radial;
			double	outbd_turn_hdg;
			double	max_excrs_dist;
			double	max_excrs_time;
			int	turn_right;
			char	navaid[NAV_NAME_LEN];
		} proc_turn;
	} leg_cmd;

	/* Segment termination condition */
	union {
		fix_t		fix;		/* AF, CF, DF, RF, TF */
		alt_constr_t	alt;		/* CA, FA, HA, VA */
		struct {			/* CR, CI (optional), VR */
			char	navaid[NAV_NAME_LEN];
			double	radial;
		} radial;
		struct {			/* CD */
			char	navaid[NAV_NAME_LEN];
			double	dist;
		} dme;
		char		navaid[NAV_NAME_LEN];	/* VI (optional) */
	} term_cond;

	/* Generic segment constraints */
	spd_constr_t spd_constr;
	alt_constr_t alt_constr;
} navproc_seg_t;

typedef enum {
	NAVPROC_FINAL_ILS,		/* I */
	NAVPROC_FINAL_VOR,		/* D */
	NAVPROC_FINAL_NDB,		/* N */
	NAVPROC_FINAL_RNAV,		/* G */
	NAVPROC_FINAL_LDA		/* C */
} navproc_final_t;

typedef struct navproc_s {
	navproc_type_t	type;
	char		name[NAV_NAME_LEN];
	char		rwy_ID[RWY_ID_LEN + 1];
	char		fix_name[NAV_NAME_LEN];
	unsigned	num_segs;
	navproc_seg_t	*segs;
	/* number of main procedure segments, remainder is for go-around */
	unsigned	num_main_segs;
	navproc_final_t	final_type;
} navproc_t;

/* Airport structures */

typedef struct runway_s {
	char		rwy_ID[RWY_ID_LEN + 1];
	unsigned	hdg;
	unsigned	length;
	unsigned	width;
	int		loc_avail;
	double		loc_freq;
	unsigned	loc_fcrs;
	geo_pos_3d_t	thr_pos;
	double		gp_angle;
} runway_t;

typedef struct airport_s {
	char		name[16];
	char		icao[ICAO_NAME_LEN + 1];
	geo_pos_3d_t	refpt_pos;
	unsigned	TA;
	unsigned	TL;
	unsigned	longest_rwy;
	unsigned	num_rwys;
	runway_t	*rwys;
	unsigned	num_procs;
	navproc_t	*procs;
	unsigned	num_gates;
	fix_t		*gates;
} airport_t;

airport_t *airport_open(const char *arpt_icao, const char *navdata_dir);
void airport_free(airport_t *arpt);

const runway_t *airport_find_rwy_by_ID(const airport_t *arpt,
    const char *rwy_ID);
geo_pos_2d_t airport_find_gate_pos(const airport_t *arpt, const char *gate_ID);

#endif	/* _OPENFMC_AIRAC_H_ */