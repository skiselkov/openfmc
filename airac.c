#include <stdlib.h>
#include <string.h>
#include <assert.h>
#include <stdio.h>
#include <math.h>
#include <errno.h>

#include "airac.h"
#include "helpers.h"
#include "log.h"

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
		/*
		 * Don't log this error, this function is used to look for
		 * airport lines.
		 */
		goto errout;
	}

	(void) strlcpy(arpt->name, comps[2], sizeof (arpt->name));
	if (!geo_pos_3d_from_str(comps[3], comps[4], comps[5],
	    &arpt->refpt_pos)) {
		openfmc_log(OPENFMC_LOG_ERR, "Error parsing initial airport "
		    "line: reference point coordinates invalid.");
		goto errout;
	}
	if (/* transition altitude must be uint && a valid altitude */
	    sscanf(comps[6], "%u", &arpt->TA) != 1 || !is_valid_alt(arpt->TA) ||
	    /* transition level must be uint && a valid altitude */
	    sscanf(comps[7], "%u", &arpt->TL) != 1 || !is_valid_alt(arpt->TL) ||
	    /* longest rwy must be non-zero uint && <= MAX_RWY_LEN */
	    sscanf(comps[8], "%u", &arpt->longest_rwy) != 1 ||
	    arpt->longest_rwy == 0 || arpt->longest_rwy > MAX_RWY_LEN) {
		openfmc_log(OPENFMC_LOG_ERR, "Error parsing initial airport "
		    "line: TA, TL or longest runway parameters invalid.");
		goto errout;
	}

	free(comps);
	return (1);
errout:
	free(comps);
	return (0);
}

static int
parse_rwy_line(const char *line, runway_t *rwy)
{
	size_t num_comps;
	char *line_copy = strdup(line);
	char **comps = explode_line(line_copy, ",", &num_comps);

	/* Line must start with "R" keyword */
	if (strcmp(comps[0], "R") != 0) {
		openfmc_log(OPENFMC_LOG_ERR, "Error parsing runway line: "
		    "runway doesn't start with 'R'.");
		goto errout;
	}

	if (!is_valid_rwy_ID(comps[1])) {
		openfmc_log(OPENFMC_LOG_ERR, "Error parsing runway line: "
		    "runway doesn't start with 'R'.");
		goto errout;
	}
	(void) strcpy(rwy->rwy_ID, comps[1]);
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
	    rwy->gp_angle < 0.0 || rwy->gp_angle > GP_MAX_ANGLE) {
		openfmc_log(OPENFMC_LOG_ERR, "Error parsing runway line: "
		    "invalid parameters found.");
		goto errout;
	}

	free(comps);
	free(line_copy);
	return (1);
errout:
	openfmc_log(OPENFMC_LOG_ERR, "Error parsing runway line \"%s\".", line);
	free(comps);
	free(line_copy);
	return (0);
}

static int
parse_sid_proc_line(char **comps, size_t num_comps, navproc_t *proc)
{
	if (num_comps != 4) {
		openfmc_log(OPENFMC_LOG_ERR, "Error parsing SID line: "
		    "incorrect number of columns.");
		return (0);
	}
	(void) strlcpy(proc->name, comps[1], sizeof (proc->name));
	if (is_valid_rwy_ID(comps[2])) {
		proc->type = NAVPROC_TYPE_SID;
		(void) strlcpy(proc->rwy_ID, comps[2], sizeof (proc->rwy_ID));
	} else if (strcmp(comps[2], "ALL") == 0) {
		proc->type = NAVPROC_TYPE_SID_COMMON;
	} else {
		proc->type = NAVPROC_TYPE_SID_TRANS;
		(void) strlcpy(proc->fix_name, comps[2],
		    sizeof (proc->fix_name));
	}
	return (1);
}

static int
parse_star_proc_line(char **comps, size_t num_comps, navproc_t *proc)
{
	if (num_comps != 4) {
		openfmc_log(OPENFMC_LOG_ERR, "Error parsing STAR line: "
		    "incorrect number of columns.");
		return (0);
	}
	(void) strlcpy(proc->name, comps[1], sizeof (proc->name));
	if (is_valid_rwy_ID(comps[2])) {
		proc->type = NAVPROC_TYPE_STAR;
		(void) strlcpy(proc->rwy_ID, comps[2], sizeof (proc->rwy_ID));
	} else if (strcmp(comps[2], "ALL") == 0) {
		proc->type = NAVPROC_TYPE_STAR_COMMON;
	} else {
		proc->type = NAVPROC_TYPE_STAR_TRANS;
		(void) strlcpy(proc->fix_name, comps[2],
		    sizeof (proc->fix_name));
	}
	return (1);
}

static int
parse_apptr_proc_line(char **comps, size_t num_comps, navproc_t *proc)
{
	if (num_comps != 4) {
		openfmc_log(OPENFMC_LOG_ERR, "Error parsing APPTR line: "
		    "incorrect number of columns.");
		return (0);
	}
	proc->type = NAVPROC_TYPE_FINAL_TRANS;
	if (!is_valid_rwy_ID(comps[2])) {
		openfmc_log(OPENFMC_LOG_ERR, "Error parsing APPTR line: "
		    "invalid runway ID \"%s\".", comps[2]);
		return (0);
	}
	(void) strlcpy(proc->name, comps[1], sizeof (proc->name));
	(void) strlcpy(proc->rwy_ID, comps[2], sizeof (proc->rwy_ID));
	(void) strlcpy(proc->fix_name, comps[3], sizeof (proc->fix_name));
	return (1);
}

static int
parse_final_proc_line(char **comps, size_t num_comps, navproc_t *proc)
{
	if (num_comps != 5) {
		openfmc_log(OPENFMC_LOG_ERR, "Error parsing FINAL line: "
		    "incorrect number of columns.");
		return (0);
	}
	proc->type = NAVPROC_TYPE_FINAL;
	if (!is_valid_rwy_ID(comps[2])) {
		openfmc_log(OPENFMC_LOG_ERR, "Error parsing FINAL line: "
		    "invalid runway ID \"%s\".", comps[2]);
		return (0);
	}
	(void) strlcpy(proc->name, comps[1], sizeof (proc->name));
	(void) strlcpy(proc->rwy_ID, comps[2], sizeof (proc->rwy_ID));
	if (strlen(comps[3]) != 1) {
		openfmc_log(OPENFMC_LOG_ERR, "Error parsing FINAL line: "
		    "invalid approach type code \"%s\".", comps[3]);
		return (0);
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
		return (0);
	}
	if (sscanf(comps[4], "%u", &proc->num_main_segs) != 1) {
		openfmc_log(OPENFMC_LOG_ERR, "Error parsing FINAL line: "
		    "invalid number of main segments \"%s\".", comps[4]);
		return (0);
	}
	return (1);
}

static int
parse_alt_constr(char *comps[3], alt_constr_t *alt)
{
	if (sscanf(comps[0], "%d", &alt->type) != 1) {
		openfmc_log(OPENFMC_LOG_ERR, "Error parsing altitude "
		    "constraint: invalid constraint type %s.", comps[0]);
		return (0);
	}
	switch (alt->type) {
	case ALT_CONSTR_NONE:
		break;
	case ALT_CONSTR_AT:
	case ALT_CONSTR_AT_OR_ABV:
	case ALT_CONSTR_AT_OR_BLW:
		if (sscanf(comps[1], "%u", &alt->alt1) != 1 ||
		    !is_valid_alt(alt->alt1)) {
			openfmc_log(OPENFMC_LOG_ERR, "Error parsing altitude "
			    "constraint: invalid altitude value %s.", comps[1]);
			return (0);
		}
		break;
	case ALT_CONSTR_BETWEEN:
		if (sscanf(comps[1], "%u", &alt->alt1) != 1 ||
		    !is_valid_alt(alt->alt1) ||
		    sscanf(comps[2], "%u", &alt->alt2) != 1 ||
		    !is_valid_alt(alt->alt2)) {
			openfmc_log(OPENFMC_LOG_ERR, "Error parsing altitude "
			    "constraint: invalid altitude values %s,%s.",
			    comps[1], comps[2]);
			return (0);
		}
		break;
	default:
		openfmc_log(OPENFMC_LOG_ERR, "Error parsing altitude "
		    "constraint: unknown constraint type %s.", comps[0]);
		return (0);
	}

	return (1);
}

static int
parse_spd_constr(char *comps[2], spd_constr_t *spd)
{
	if (sscanf(comps[0], "%d", &spd->type) != 1) {
		openfmc_log(OPENFMC_LOG_ERR, "Error parsing speed "
		    "constraint: invalid constraint type %s.", comps[0]);
		return (0);
	}
	switch (spd->type) {
	case SPD_CONSTR_NONE:
		break;
	case SPD_CONSTR_AT_OR_BLW:
		if (sscanf(comps[1], "%u", &spd->spd1) != 1 ||
		    !is_valid_spd(spd->spd1)) {
			openfmc_log(OPENFMC_LOG_ERR, "Error parsing speed "
			    "constraint: invalid speed value %s.", comps[1]);
			return (0);
		}
		break;
	default:
		openfmc_log(OPENFMC_LOG_ERR, "Error parsing speed "
		    "constraint: unknown constraint type %s.", comps[0]);
		return (0);
	}
	return (1);
}

static int
parse_proc_seg_fix(char *comps[3], fix_t *fix)
{
	if (strlen(comps[0]) > sizeof (fix->name) - 1)
		return (0);
	(void) strlcpy(fix->name, comps[0], sizeof (fix->name));
	if (!geo_pos_2d_from_str(comps[1], comps[2], &fix->pos))
		return (0);
	return (1);
}

#define	CHECK_NUM_COMPS(n, seg_type) \
	do { \
		if (num_comps != (n)) { \
			openfmc_log(OPENFMC_LOG_ERR, "Error parsing %s " \
			    "leg definition line: invalid number of columns " \
			    "on line, wanted %d, got %lu.", #seg_type, n, \
			    num_comps); \
			return (0); \
		} \
	} while (0)

static int
parse_AF_seg(char **comps, size_t num_comps, navproc_seg_t *seg)
{
	int dir;

	CHECK_NUM_COMPS(17, AF);
	seg->type = NAVPROC_SEG_TYPE_ARC_TO_FIX;
	if (sscanf(comps[4], "%d", &dir) != 1 || (dir != 1 && dir != 2)) {
		openfmc_log(OPENFMC_LOG_ERR, "Error parsing AF segment: "
		    "arc direction value invalid: %s.", comps[4]);
		return (0);
	}
	(void) strlcpy(seg->leg_cmd.dme_arc.navaid, comps[5],
	    sizeof (seg->leg_cmd.dme_arc.navaid));
	if (!parse_proc_seg_fix(&comps[1], &seg->term_cond.fix) ||
	    sscanf(comps[6], "%lf", &seg->leg_cmd.dme_arc.start_radial) != 1 ||
	    !is_valid_hdg(seg->leg_cmd.dme_arc.start_radial) ||
	    sscanf(comps[7], "%lf", &seg->leg_cmd.dme_arc.radius) != 1 ||
	    !is_valid_arc_radius(seg->leg_cmd.dme_arc.radius) ||
	    sscanf(comps[8], "%lf", &seg->leg_cmd.dme_arc.end_radial) != 1 ||
	    !is_valid_hdg(seg->leg_cmd.dme_arc.end_radial) ||
	    !parse_alt_constr(&comps[9], &seg->alt_constr) ||
	    !parse_spd_constr(&comps[12], &seg->spd_constr)) {
		openfmc_log(OPENFMC_LOG_ERR, "Error parsing AF segment "
		    "data fields");
		return (0);
	}
	if (dir == 2) {
		/*
		 * Swap start_radial and end_radial, we always store the
		 * ARC direction as going from start_radial to end_radial.
		 */
		double tmp = seg->leg_cmd.dme_arc.start_radial;
		seg->leg_cmd.dme_arc.start_radial =
		    seg->leg_cmd.dme_arc.end_radial;
		seg->leg_cmd.dme_arc.end_radial = tmp;
	}

	return (1);
}

static int
parse_CA_seg(char **comps, size_t num_comps, navproc_seg_t *seg)
{
	CHECK_NUM_COMPS(11, CA);
	seg->type = NAVPROC_SEG_TYPE_CRS_TO_ALT;
	if (sscanf(comps[2], "%lf", &seg->leg_cmd.crs) != 1 ||
	    !is_valid_hdg(seg->leg_cmd.crs))
		return (0);
	if (!parse_alt_constr(&comps[3], &seg->term_cond.alt) ||
	    /* altitude constraint is required for CA segs */
	    seg->term_cond.alt.type == ALT_CONSTR_NONE ||
	    !parse_spd_constr(&comps[6], &seg->spd_constr))
		return (0);
	seg->alt_constr = seg->term_cond.alt;
	return (1);
}

static int
parse_CD_seg(char **comps, size_t num_comps, navproc_seg_t *seg)
{
	CHECK_NUM_COMPS(18, CD);
	seg->type = NAVPROC_SEG_TYPE_CRS_TO_DME;
	if (sscanf(comps[8], "%lf", &seg->leg_cmd.crs) != 1 ||
	    !is_valid_hdg(seg->leg_cmd.crs) ||
	    sscanf(comps[9], "%lf", &seg->term_cond.dme.dist) != 1 ||
	    !parse_alt_constr(&comps[10], &seg->alt_constr) ||
	    !parse_spd_constr(&comps[13], &seg->spd_constr))
		return (0);
	(void) strlcpy(seg->term_cond.dme.navaid, comps[5],
	    sizeof (seg->term_cond.dme.navaid));
	return (1);
}

static int
parse_CF_seg(char **comps, size_t num_comps, navproc_seg_t *seg)
{
	CHECK_NUM_COMPS(18, CF);
	seg->type = NAVPROC_SEG_TYPE_CRS_TO_FIX;
	if (sscanf(comps[8], "%lf", &seg->leg_cmd.crs) != 1 ||
	    !is_valid_hdg(seg->leg_cmd.crs) ||
	    !parse_proc_seg_fix(&comps[1], &seg->term_cond.fix) ||
	    !parse_alt_constr(&comps[10], &seg->alt_constr) ||
	    !parse_spd_constr(&comps[13], &seg->spd_constr))
		return (0);
	return (1);
}

static int
parse_CI_CR_seg(char **comps, size_t num_comps, navproc_seg_t *seg,
    int is_CI)
{
	if (is_CI)
		CHECK_NUM_COMPS(13, CI);
	else
		CHECK_NUM_COMPS(13, CR);
	seg->type = NAVPROC_SEG_TYPE_CRS_TO_INTCP;
	if (sscanf(comps[4], "%lf", &seg->leg_cmd.crs) != 1 ||
	    !is_valid_hdg(seg->leg_cmd.crs) ||
	    !parse_alt_constr(&comps[5], &seg->alt_constr) ||
	    !parse_spd_constr(&comps[8], &seg->spd_constr) ||
	    sscanf(comps[3], "%lf", &seg->term_cond.radial.radial) != 1 ||
	    (!is_CI && !is_valid_hdg(seg->term_cond.radial.radial)))
		return (0);
	(void) strlcpy(seg->term_cond.radial.navaid, comps[2],
	    sizeof (seg->term_cond.radial.navaid));
	return (1);
}

static int
parse_DF_TF_seg(char **comps, size_t num_comps, navproc_seg_t *seg,
    int is_DF)
{
	if (is_DF)
		CHECK_NUM_COMPS(16, DF);
	else
		CHECK_NUM_COMPS(18, TF);
	seg->type = is_DF ? NAVPROC_SEG_TYPE_DIR_TO_FIX :
	    NAVPROC_SEG_TYPE_TRK_TO_FIX;
	if (!geo_pos_2d_from_str(comps[2], comps[3], &seg->term_cond.fix.pos) ||
	    !parse_alt_constr(&comps[is_DF ? 8 : 10], &seg->alt_constr) ||
	    !parse_spd_constr(&comps[is_DF ? 11 : 13], &seg->spd_constr))
		return (0);
	(void) strlcpy(seg->term_cond.fix.name, comps[1],
	    sizeof (seg->term_cond.fix.name));
	return (1);
}

static int
parse_FA_seg(char **comps, size_t num_comps, navproc_seg_t *seg)
{
	CHECK_NUM_COMPS(17, FA);
	seg->type = NAVPROC_SEG_TYPE_FIX_TO_ALT;
	if (!geo_pos_2d_from_str(comps[2], comps[3], &seg->leg_cmd.fix.pos) ||
	    !parse_alt_constr(&comps[9], &seg->term_cond.alt) ||
	    /* altitude constraint is required for CA segs */
	    seg->term_cond.alt.type == ALT_CONSTR_NONE ||
	    !parse_spd_constr(&comps[12], &seg->spd_constr))
		return (0);
	seg->alt_constr = seg->term_cond.alt;
	(void) strlcpy(seg->leg_cmd.fix.name, comps[1],
	    sizeof (seg->leg_cmd.fix.name));
	return (1);
}

static int
parse_FC_FD_seg(char **comps, size_t num_comps, navproc_seg_t *seg,
    int is_FC)
{
	if (is_FC) {
		CHECK_NUM_COMPS(18, FC);
		seg->type = NAVPROC_SEG_TYPE_FIX_TO_DIST;
	} else {
		CHECK_NUM_COMPS(18, FD);
		seg->type = NAVPROC_SEG_TYPE_FIX_TO_DME;
	}
	if (!geo_pos_2d_from_str(comps[2], comps[3], &seg->leg_cmd.fix.pos) ||
	    sscanf(comps[9], "%lf", &seg->term_cond.dme.dist) != 1 ||
	    !parse_alt_constr(&comps[10], &seg->alt_constr) ||
	    !parse_spd_constr(&comps[13], &seg->spd_constr))
		return (0);
	(void) strlcpy(seg->leg_cmd.fix.name, comps[1],
	    sizeof (seg->leg_cmd.fix.name));
	(void) strlcpy(seg->term_cond.dme.navaid, comps[5],
	    sizeof (seg->term_cond.dme.navaid));
	return (1);
}

static int
parse_FM_seg(char **comps, size_t num_comps, navproc_seg_t *seg)
{
	CHECK_NUM_COMPS(17, FM);
	seg->type = NAVPROC_SEG_TYPE_FIX_TO_MANUAL;
	if (!geo_pos_2d_from_str(comps[2], comps[3],
	    &seg->leg_cmd.fix_trk.fix.pos) ||
	    sscanf(comps[8], "%lf", &seg->leg_cmd.fix_trk.crs) != 1 ||
	    !parse_alt_constr(&comps[9], &seg->alt_constr) ||
	    !parse_spd_constr(&comps[12], &seg->spd_constr))
		return (0);
	(void) strlcpy(seg->leg_cmd.fix_trk.fix.name, comps[1],
	    sizeof (seg->leg_cmd.fix_trk.fix.name));
	return (1);
}

static int
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
		assert(0);
	seg->type = type;

	if (!geo_pos_2d_from_str(comps[2], comps[3],
	    &seg->leg_cmd.hold.fix.pos) ||
	    !parse_alt_constr(&comps[10], &seg->alt_constr) ||
	    /* alt constr is mandatory on HA segs */
	    (type == NAVPROC_SEG_TYPE_HOLD_TO_ALT &&
	    seg->alt_constr.type == ALT_CONSTR_NONE) ||
	    !parse_spd_constr(&comps[13], &seg->spd_constr))
		return (0);
	(void) strlcpy(seg->leg_cmd.hold.fix.name, comps[1],
	    sizeof (seg->leg_cmd.fix.name));
	(void) strlcpy(seg->term_cond.dme.navaid, comps[5],
	    sizeof (seg->term_cond.dme.navaid));
	if (type == NAVPROC_SEG_TYPE_HOLD_TO_ALT)
		seg->term_cond.alt = seg->alt_constr;
	else if (type == NAVPROC_SEG_TYPE_HOLD_TO_FIX)
		seg->term_cond.fix = seg->leg_cmd.hold.fix;
	return (1);
}

static int
parse_IF_seg(char **comps, size_t num_comps, navproc_seg_t *seg)
{
	CHECK_NUM_COMPS(15, IF);
	seg->type = NAVPROC_SEG_TYPE_INIT_FIX;
	if (!geo_pos_2d_from_str(comps[2], comps[3], &seg->leg_cmd.fix.pos) ||
	    !parse_alt_constr(&comps[7], &seg->alt_constr) ||
	    !parse_spd_constr(&comps[10], &seg->spd_constr))
		return (0);
	(void) strlcpy(seg->leg_cmd.fix.name, comps[1],
	    sizeof (seg->leg_cmd.fix.name));
	return (1);
}

static int
parse_PI_seg(char **comps, size_t num_comps, navproc_seg_t *seg)
{
	int turn_dir;

	CHECK_NUM_COMPS(18, PI);
	seg->type = NAVPROC_SEG_TYPE_PROC_TURN;
	if (!geo_pos_2d_from_str(comps[2], comps[3],
	    &seg->leg_cmd.proc_turn.startpt.pos) ||
	    sscanf(comps[4], "%d", &turn_dir) ||
	    (turn_dir != 1 && turn_dir != 2) ||
	    sscanf(comps[6], "%lf", &seg->leg_cmd.proc_turn.outbd_turn_hdg)
	    != 1 || !is_valid_hdg(seg->leg_cmd.proc_turn.outbd_turn_hdg) ||
	    sscanf(comps[7], "%lf", &seg->leg_cmd.proc_turn.max_excrs_dist)
	    != 1 ||
	    sscanf(comps[8], "%lf", &seg->leg_cmd.proc_turn.outbd_radial)
	    != 1 || !is_valid_hdg(seg->leg_cmd.proc_turn.outbd_radial) ||
	    sscanf(comps[9], "%lf", &seg->leg_cmd.proc_turn.max_excrs_time)
	    != 1 ||
	    !parse_alt_constr(&comps[10], &seg->alt_constr) ||
	    !parse_spd_constr(&comps[12], &seg->spd_constr))
		return (0);
	(void) strlcpy(seg->leg_cmd.proc_turn.startpt.name, comps[1],
	    sizeof (seg->leg_cmd.proc_turn.startpt.name));
	(void) strlcpy(seg->leg_cmd.proc_turn.navaid, comps[5],
	    sizeof (seg->leg_cmd.proc_turn.navaid));
	return (1);
}

static int
parse_RF_seg(char **comps, size_t num_comps, navproc_seg_t *seg)
{
	CHECK_NUM_COMPS(16, RF);
	seg->type = NAVPROC_SEG_TYPE_RADIUS_ARC_TO_FIX;
	if (sscanf(comps[4], "%d", &seg->leg_cmd.radius_arc.cw) != 1 ||
	    (seg->leg_cmd.radius_arc.cw != 1 &&
	    seg->leg_cmd.radius_arc.cw != 2))
		return (0);
	/* change CW flag from 1-2 to 0-1 */
	seg->leg_cmd.radius_arc.cw--;
	(void) strlcpy(seg->leg_cmd.radius_arc.navaid, comps[5],
	    sizeof (seg->leg_cmd.radius_arc.navaid));
	if (!parse_proc_seg_fix(&comps[1], &seg->term_cond.fix) ||
	    sscanf(comps[6], "%lf", &seg->leg_cmd.radius_arc.end_radial) != 1 ||
	    !is_valid_hdg(seg->leg_cmd.radius_arc.end_radial) ||
	    sscanf(comps[7], "%lf", &seg->leg_cmd.radius_arc.radius) != 1 ||
	    !is_valid_arc_radius(seg->leg_cmd.radius_arc.radius) ||
	    !parse_alt_constr(&comps[8], &seg->alt_constr) ||
	    !parse_spd_constr(&comps[11], &seg->spd_constr))
		return (0);

	return (1);
}

static int
parse_VA_seg(char **comps, size_t num_comps, navproc_seg_t *seg)
{
	CHECK_NUM_COMPS(11, VA);
	seg->type = NAVPROC_SEG_TYPE_HDG_TO_ALT;
	if (sscanf(comps[2], "%lf", &seg->leg_cmd.hdg) != 1 ||
	    !is_valid_hdg(seg->leg_cmd.hdg) ||
	    !parse_alt_constr(&comps[3], &seg->alt_constr) ||
	    /* alt constr mandatory on VA segs */
	    seg->alt_constr.type == ALT_CONSTR_NONE ||
	    !parse_spd_constr(&comps[6], &seg->spd_constr)) {
		openfmc_log(OPENFMC_LOG_ERR, "Error parsing VA segment line");
		return (0);
	}
	seg->term_cond.alt = seg->alt_constr;

	return (1);
}

static int
parse_VD_seg(char **comps, size_t num_comps, navproc_seg_t *seg)
{
	CHECK_NUM_COMPS(18, VD);
	seg->type = NAVPROC_SEG_TYPE_HDG_TO_DME;
	if (sscanf(comps[8], "%lf", &seg->leg_cmd.hdg) != 1 ||
	    !is_valid_hdg(seg->leg_cmd.hdg) ||
	    sscanf(comps[9], "%lf", &seg->term_cond.dme.dist) != 1 ||
	    !parse_alt_constr(&comps[10], &seg->alt_constr) ||
	    !parse_spd_constr(&comps[13], &seg->spd_constr)) {
		openfmc_log(OPENFMC_LOG_ERR, "Error parsing VD segment line");
		return (0);
	}
	(void) strlcpy(seg->term_cond.dme.navaid, comps[5],
	    sizeof (seg->term_cond.dme.navaid));

	return (1);
}

static int
parse_VI_VM_VR_seg(char **comps, size_t num_comps, navproc_seg_t *seg,
    navproc_seg_type_t type)
{
	char *leg_name;
	if (type == NAVPROC_SEG_TYPE_HDG_TO_INTCP) {
		CHECK_NUM_COMPS(13, VI);
		leg_name = "VI";
	} else if (type == NAVPROC_SEG_TYPE_HDG_TO_MANUAL) {
		CHECK_NUM_COMPS(13, VM);
		leg_name = "VM";
	} else if (type == NAVPROC_SEG_TYPE_HDG_TO_RADIAL) {
		CHECK_NUM_COMPS(13, VR);
		leg_name = "VR";
	} else {
		assert(0);
	}
	seg->type = type;
	if (sscanf(comps[4], "%lf", &seg->leg_cmd.hdg) != 1 ||
	    !is_valid_hdg(seg->leg_cmd.hdg) ||
	    (type == NAVPROC_SEG_TYPE_HDG_TO_RADIAL &&
	    (sscanf(comps[4], "%lf", &seg->term_cond.radial.radial) != 1 ||
	    !is_valid_hdg(seg->term_cond.radial.radial))) ||
	    !parse_alt_constr(&comps[5], &seg->alt_constr) ||
	    !parse_spd_constr(&comps[8], &seg->spd_constr)) {
		openfmc_log(OPENFMC_LOG_ERR, "Error parsing %s segment line",
		    leg_name);
		return (0);
	}
	if (type == NAVPROC_SEG_TYPE_HDG_TO_INTCP &&
	    strcmp(comps[2], " ") != 0) {
		/* Copy non-empty navaid to termination condition on VI segs */
		(void) strlcpy(seg->term_cond.navaid, comps[2],
		    sizeof (seg->term_cond.navaid));
	} else if (type == NAVPROC_SEG_TYPE_HDG_TO_RADIAL) {
		(void) strlcpy(seg->term_cond.radial.navaid, comps[2],
		    sizeof (seg->term_cond.radial.navaid));
	}

	return (1);
}

static int
parse_proc_seg_line(const char *line, navproc_t *proc)
{
	char		**comps = NULL;
	size_t		num_comps = 0;
	navproc_seg_t	seg;
	char		*line_copy = strdup(line);

	comps = explode_line(line_copy, ",", &num_comps);

	assert(num_comps > 0);
	if (strcmp(comps[0], "AF") == 0) {
		if (!parse_AF_seg(comps, num_comps, &seg))
			goto errout;
	} else if (strcmp(comps[0], "CA") == 0) {
		if (!parse_CA_seg(comps, num_comps, &seg))
			goto errout;
	} else if (strcmp(comps[0], "CD") == 0) {
		if (!parse_CD_seg(comps, num_comps, &seg))
			goto errout;
	} else if (strcmp(comps[0], "CF") == 0) {
		if (!parse_CF_seg(comps, num_comps, &seg))
			goto errout;
	} else if (strcmp(comps[0], "CI") == 0) {
		if (!parse_CI_CR_seg(comps, num_comps, &seg, 1))
			goto errout;
	} else if (strcmp(comps[0], "CR") == 0) {
		if (!parse_CI_CR_seg(comps, num_comps, &seg, 0))
			goto errout;
	} else if (strcmp(comps[0], "DF") == 0) {
		if (!parse_DF_TF_seg(comps, num_comps, &seg, 1))
			goto errout;
	} else if (strcmp(comps[0], "FA") == 0) {
		if (!parse_FA_seg(comps, num_comps, &seg))
			goto errout;
	} else if (strcmp(comps[0], "FC") == 0) {
		if (!parse_FC_FD_seg(comps, num_comps, &seg, 1))
			goto errout;
	} else if (strcmp(comps[0], "FD") == 0) {
		if (!parse_FC_FD_seg(comps, num_comps, &seg, 0))
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
		if (!parse_PI_seg(comps, num_comps, &seg))
			goto errout;
	} else if (strcmp(comps[0], "RF") == 0) {
		if (!parse_RF_seg(comps, num_comps, &seg))
			goto errout;
	} else if (strcmp(comps[0], "TF") == 0) {
		if (!parse_DF_TF_seg(comps, num_comps, &seg, 0))
			goto errout;
	} else if (strcmp(comps[0], "VA") == 0) {
		if (!parse_VA_seg(comps, num_comps, &seg))
			goto errout;
	} else if (strcmp(comps[0], "VD") == 0) {
		if (!parse_VD_seg(comps, num_comps, &seg))
			goto errout;
	} else if (strcmp(comps[0], "VI") == 0) {
		if (!parse_VI_VM_VR_seg(comps, num_comps, &seg,
		    NAVPROC_SEG_TYPE_HDG_TO_INTCP))
			goto errout;
	} else if (strcmp(comps[0], "VM") == 0) {
		if (!parse_VI_VM_VR_seg(comps, num_comps, &seg,
		    NAVPROC_SEG_TYPE_HDG_TO_MANUAL))
			goto errout;
	} else if (strcmp(comps[0], "VR") == 0) {
		if (!parse_VI_VM_VR_seg(comps, num_comps, &seg,
		    NAVPROC_SEG_TYPE_HDG_TO_RADIAL))
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

	free(comps);
	free(line_copy);
	return (1);
errout:
	openfmc_log(OPENFMC_LOG_ERR, "Error parsing procedure segment line. "
	    "Offending line was: \"%s\".", line);
	free(comps);
	free(line_copy);
	return (0);
}

static int
parse_proc(FILE *fp, navproc_t *proc)
{
	size_t		line_cap = 0;
	ssize_t		line_len = 0;
	char		*line = NULL;
	char		*line_copy;
	char		**comps = NULL;
	size_t		num_comps = 0;

	memset(proc, 0, sizeof (*proc));
	line_len = getline(&line, &line_cap, fp);
	if (line_len == -1) {
		/* EOF */
		return (0);
	}
	strip_newline(line);
	line_copy = strdup(line);
	comps = explode_line(line_copy, ",", &num_comps);
	assert(num_comps != 0);

	if (strcmp(comps[0], "SID") == 0) {
		if (!parse_sid_proc_line(comps, num_comps, proc))
			goto errout;
	} else if (strcmp(comps[0], "STAR") == 0) {
		if (!parse_star_proc_line(comps, num_comps, proc))
			goto errout;
	} else if (strcmp(comps[0], "APPTR") == 0) {
		if (!parse_apptr_proc_line(comps, num_comps, proc))
			goto errout;
	} else if (strcmp(comps[0], "FINAL") == 0) {
		if (!parse_final_proc_line(comps, num_comps, proc))
			goto errout;
	} else {
		goto errout;
	}

	free(comps);
	comps = NULL;
	num_comps = 0;

	while ((line_len = getline(&line, &line_cap, fp)) != -1) {
		strip_newline(line);
		if (strlen(line) == 0)
			break;
		if (!parse_proc_seg_line(line, proc))
			goto errout;
	}
	if (proc->num_segs == 0) {
		/* faulty procedure with no segments */
		openfmc_log(OPENFMC_LOG_ERR, "Error parsing procedure: "
		    "no segments found");
		goto errout;
	}

	free(line);
	free(line_copy);

	return (1);
errout:
	free(proc->segs);
	free(comps);
	free(line);
	free(line_copy);
	return (-1);
}

static int
parse_proc_file(FILE *fp, airport_t *arpt)
{
	navproc_t	proc;
	int		n;

	while ((n = parse_proc(fp, &proc)) != -1) {
		if (n == 0) {
			/* EOF */
			break;
		}
		arpt->num_procs++;
		arpt->procs = realloc(arpt->procs, sizeof (navproc_t) *
		    arpt->num_procs);
		(void) memcpy(&arpt->procs[arpt->num_procs - 1], &proc,
		    sizeof (proc));
	}
	if (n == -1)
		return (0);

	return (1);
}

airport_t *
airport_open(const char *arpt_icao, const char *navdata_dir)
{
	airport_t	*arpt;
	FILE		*arpt_fp = NULL, *proc_fp = NULL;
	char		*arpt_fname = NULL;
	char		*proc_fname = NULL;
	ssize_t		line_len = 0;
	size_t		line_cap = 0;
	char		*line = NULL;
	int		done = 0;

	arpt = calloc(sizeof (*arpt), 1);
	if (!arpt)
		return (NULL);

	assert(strlen(arpt_icao) == 4);
	strcpy(arpt->icao, arpt_icao);

	/* Open Airports.txt */
	arpt_fname = malloc(strlen(navdata_dir) + strlen("/Airports.txt") + 1);
	sprintf(arpt_fname, "%s/Airports.txt", navdata_dir);
	arpt_fp = fopen(arpt_fname, "r");
	if (arpt_fp == NULL) {
		openfmc_log(OPENFMC_LOG_ERR, "Can't open %s: %s", arpt_fname,
		    strerror(errno));
		goto errout;
	}

	/* Locate airport starting line & parse it */
	while ((line_len = getline(&line, &line_cap, arpt_fp)) != -1 && !done) {
		strip_newline(line);
		if (parse_arpt_line(line, arpt)) {
			done = 1;
			break;
		}
	}
	if (done == 0) {
		/* error reading/locating airport */
		openfmc_log(OPENFMC_LOG_ERR, "Error opening airport %s: "
		    "airport not found.", arpt_icao);
		goto errout;
	}

	/* airport found, read non-empty runway lines */
	while ((line_len = getline(&line, &line_cap, arpt_fp)) > 0) {
		strip_newline(line);
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
	if (arpt->num_rwys == 0) {
		openfmc_log(OPENFMC_LOG_ERR, "Error opening airport %s: "
		    "airport has no runways.", arpt_icao);
		goto errout;
	}

	/* Try to read any procedures we may have available for this airport */
	proc_fname = malloc(strlen(navdata_dir) + strlen("/Proc/XXXX.txt") + 1);
	sprintf(proc_fname, "%s/Proc/%s.txt", navdata_dir, arpt->icao);
	proc_fp = fopen(proc_fname, "r");
	if (proc_fp != NULL) {
		if (!parse_proc_file(proc_fp, arpt)) {
			openfmc_log(OPENFMC_LOG_ERR, "Error parsing airport "
			    "procedures file %s", proc_fname);
			goto errout;
		}
	}

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
