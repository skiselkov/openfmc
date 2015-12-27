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
#include <stddef.h>
#include <string.h>
#include <stdio.h>

#include "math.h"
#include "perf.h"
#include "log.h"
#include "route.h"

#define	DIR_V(hdg)	\
	(hdg2dir(wmm_mag2true(wmm, (hdg), GEO2_TO_GEO3(cur_pos, 0))))
#define	SAME_DIR(a, b)	((a).x * (b).x >= 0 && (a).y * (b).y >= 0)

static void rlg_connect(route_t *route, route_leg_group_t *prev_rlg,
    route_leg_group_t *next_rlg, bool_t allow_mod, bool_t allow_add_legs);
static void rlg_bypass(route_t *route, route_leg_group_t *rlg,
    bool_t allow_mod, bool_t allow_add_legs);
static route_leg_t * last_leg_before_rlg(route_t *route,
    route_leg_group_t *rlg);
static bool_t navprocs_related(const navproc_t *nv1, const navproc_t *nv2);

typedef geo_pos2_t (*leg_intc_func_t)(geo_pos2_t cur_pos,
    const route_leg_t *rl, const list_t *legs, const wmm_t *wmm);

static geo_pos2_t calc_dir_leg_intc(geo_pos2_t cur_pos,
    const route_leg_t *rl, const list_t *legs, const wmm_t *wmm);
static geo_pos2_t calc_dist_leg_intc(geo_pos2_t cur_pos,
    const route_leg_t *rl, const list_t *legs, const wmm_t *wmm);
static geo_pos2_t calc_radial_leg_intc(geo_pos2_t cur_pos,
    const route_leg_t *rl, const list_t *legs, const wmm_t *wmm);
static geo_pos2_t calc_manual_leg_intc(geo_pos2_t cur_pos,
    const route_leg_t *rl, const list_t *legs, const wmm_t *wmm);
static geo_pos2_t calc_vect_leg_intc(geo_pos2_t cur_pos,
    const route_leg_t *rl, const list_t *legs, const wmm_t *wmm);
static geo_pos2_t calc_alt_leg_intc(geo_pos2_t cur_pos,
    const route_leg_t *rl, const list_t *legs, const wmm_t *wmm);
static geo_pos2_t calc_proc_leg_intc(geo_pos2_t cur_pos,
    const route_leg_t *rl, const list_t *legs, const wmm_t *wmm);

static bool_t rl_find_leg_seg(const route_leg_t *rl, geo_pos2_t oldpos,
    const route_leg_t *next_rl, const wmm_t *wmm, route_seg_t *seg);

static route_seg_t *rs_new_direct(geo_pos2_t start, geo_pos2_t end,
    route_seg_join_type_t join_type);
static route_seg_t *rs_new_arc(geo_pos2_t start, geo_pos2_t end,
    geo_pos2_t center, bool_t cw, route_seg_join_type_t join_type);

static double arc_seg_get_radius(const route_seg_t *rs);

/*
 * Functions that compute leg intersections. Order must follow
 * navproc_seg_type_t.
 */
static leg_intc_func_t const leg_intc_func_tbl[NAVPROC_SEG_TYPES] = {
	calc_dir_leg_intc,	/* NAVPROC_SEG_TYPE_ARC_TO_FIX */
	calc_alt_leg_intc,	/* NAVPROC_SEG_TYPE_CRS_TO_ALT */
	calc_dist_leg_intc,	/* NAVPROC_SEG_TYPE_CRS_TO_DME */
	calc_dir_leg_intc,	/* NAVPROC_SEG_TYPE_CRS_TO_FIX */
	calc_vect_leg_intc,	/* NAVPROC_SEG_TYPE_CRS_TO_INTCP */
	calc_radial_leg_intc,	/* NAVPROC_SEG_TYPE_CRS_TO_RADIAL */
	calc_dir_leg_intc,	/* NAVPROC_SEG_TYPE_DIR_TO_FIX */
	calc_alt_leg_intc,	/* NAVPROC_SEG_TYPE_FIX_TO_ALT */
	calc_dist_leg_intc,	/* NAVPROC_SEG_TYPE_FIX_TO_DIST */
	calc_dist_leg_intc,	/* NAVPROC_SEG_TYPE_FIX_TO_DME */
	calc_manual_leg_intc,	/* NAVPROC_SEG_TYPE_FIX_TO_MANUAL */
	calc_dir_leg_intc,	/* NAVPROC_SEG_TYPE_HOLD_TO_ALT */
	calc_dir_leg_intc,	/* NAVPROC_SEG_TYPE_HOLD_TO_FIX */
	calc_dir_leg_intc,	/* NAVPROC_SEG_TYPE_HOLD_TO_MANUAL */
	calc_dir_leg_intc,	/* NAVPROC_SEG_TYPE_INIT_FIX */
	calc_proc_leg_intc,	/* NAVPROC_SEG_TYPE_PROC_TURN */
	calc_dir_leg_intc,	/* NAVPROC_SEG_TYPE_RADIUS_ARC_TO_FIX */
	calc_dir_leg_intc,	/* NAVPROC_SEG_TYPE_TRK_TO_FIX */
	calc_alt_leg_intc,	/* NAVPROC_SEG_TYPE_HDG_TO_ALT */
	calc_dist_leg_intc,	/* NAVPROC_SEG_TYPE_HDG_TO_DME */
	calc_vect_leg_intc,	/* NAVPROC_SEG_TYPE_HDG_TO_INTCP */
	calc_manual_leg_intc,	/* NAVPROC_SEG_TYPE_HDG_TO_MANUAL */
	calc_radial_leg_intc	/* NAVPROC_SEG_TYPE_HDG_TO_RADIAL */
};

/*
 * Destroys and frees a route_seg_t.
 */
static void
rs_destroy(route_seg_t *seg)
{
	ASSERT(!list_link_active(&seg->route_segs_node));
	free(seg);
}

/*
 * Destroys and frees a route_leg_t.
 */
static void
rl_destroy(route_leg_t *rl)
{
	ASSERT(!list_link_active(&rl->leg_group_legs_node));
	ASSERT(!list_link_active(&rl->route_legs_node));
	free(rl);
}

/*
 * Destroys and frees a route_leg_group_t. This also remove all of the
 * leg group's associated legs. It does not handle reconnecting the adjacent
 * leg groups.
 */
static void
rlg_destroy(route_t *route, route_leg_group_t *rlg)
{
	if (rlg->type == ROUTE_LEG_GROUP_TYPE_DISCO) {
		ASSERT(!list_is_empty(&rlg->legs));
	}

	for (route_leg_t *rl = list_head(&rlg->legs); rl != NULL;
	    rl = list_head(&rlg->legs)) {
		list_remove(&rlg->legs, rl);
		list_remove(&route->legs, rl);
		rl_destroy(rl);
	}

	if (list_link_active(&rlg->route_leg_groups_node))
		list_remove(&route->leg_groups, rlg);
	list_destroy(&rlg->legs);
	free(rlg);
}

/*
 * Creates a new route leg group of `type' and returns it.
 */
static route_leg_group_t *
rlg_new(route_leg_group_type_t type, route_t *route)
{
	route_leg_group_t *rlg = calloc(sizeof (*rlg), 1);

	rlg->type = type;
	rlg->route = route;
	rlg->start_wpt = null_wpt;
	rlg->end_wpt = null_wpt;
	list_create(&rlg->legs, sizeof (route_leg_t),
	    offsetof(route_leg_t, leg_group_legs_node));

	return (rlg);
}

/*
 * Creates a new discontinuity leg group and inserts it after `prev_rlg'.
 * A discontinuity cannot be the first or last leg group in a route.
 */
static void
rlg_new_disco(route_t *route, route_leg_group_t *prev_rlg)
{
	route_leg_group_t *rlg = rlg_new(ROUTE_LEG_GROUP_TYPE_DISCO, route);
	route_leg_t *rl = calloc(sizeof (*rl), 1);

	/* A disco can't be first or last in the list */
	ASSERT(prev_rlg != NULL);
	ASSERT(list_tail(&route->leg_groups) != prev_rlg);
	list_insert_after(&route->leg_groups, prev_rlg, rlg);
	rl->disco = B_TRUE;
	rl->rlg = rlg;
	list_insert_head(&rlg->legs, rl);
	list_insert_after(&route->legs, last_leg_before_rlg(route, rlg), rl);
}

/*
 * Returns a pointer to the end wpt of a particular route leg. If the
 * route leg doesn't end in a wpt, returns a pointer to `null_wpt' instead.
 */
static const wpt_t *
leg_get_end_wpt(const route_leg_t *leg)
{
	return (!leg->disco ? navproc_seg_get_end_wpt(&leg->seg) : &null_wpt);
}

/*
 * Sets the end wpt of `leg' to `wpt'. If the leg is of a type that doesn't
 * take wpts, causes an assertion failure.
 */
static void
leg_set_end_wpt(route_leg_t *leg, const wpt_t *wpt)
{
	navproc_seg_set_end_wpt(&leg->seg, wpt);
}

static bool_t
chk_awy_fix_adjacent(route_leg_group_t *rlg, const wpt_t *wpt, bool_t head)
{
	unsigned i;

	ASSERT(rlg->type == ROUTE_LEG_GROUP_TYPE_AIRWAY);
	if (IS_NULL_WPT(&rlg->start_wpt) || IS_NULL_WPT(&rlg->end_wpt))
		return (B_FALSE);
	for (i = 0; i < rlg->awy->num_segs &&
	    !WPT_EQ(&rlg->awy->segs[i].endpt[0], head ? wpt : &rlg->end_wpt);
	    i++)
		;
	return (i < rlg->awy->num_segs &&
	    WPT_EQ(&rlg->awy->segs[i].endpt[1], head ? &rlg->start_wpt : wpt));
}

/*
 * Creates a new DF (direct-to-fix) leg and returns it.
 */
static route_leg_t *
rl_new_direct(const wpt_t *fix, route_leg_group_t *rlg)
{
	route_leg_t *rl = calloc(sizeof (*rl), 1);
	rl->seg.type = NAVPROC_SEG_TYPE_DIR_TO_FIX;
	rl->seg.term_cond.fix = *fix;
	rl->rlg = rlg;
	return (rl);
}

/*
 * Looks for the last route leg occurring before any legs on `rlg' and returns
 * it. If there no legs in front of `rlg', returns NULL instead.
 */
static route_leg_t *
last_leg_before_rlg(route_t *route, route_leg_group_t *rlg)
{
	for (rlg = list_prev(&route->leg_groups, rlg); rlg != NULL;
	    rlg = list_prev(&route->leg_groups, rlg)) {
		route_leg_t *rl = list_tail(&rlg->legs);
		if (rl != NULL)
			return (rl);
	}
	return (NULL);
}

/*
 * Checks the leg of a leg group to make sure that it ends at the appropriate
 * wpt.
 *
 * @param route The route to which the leg group belongs.
 * @param rlg The route leg group to which the leg is supposed to belong.
 * @param rl The route leg to check. If you pass NULL here, the leg will be
 *	created (as a DF leg) and will be inserted into the rlg leg group
 *	group following the `prev_rlg_rl' route leg.
 * @param prev_rlg_rl The leg in the leg group which should precede the leg
 *	being checked. If you pass NULL, the new leg will be inserted at the
 *	start of the leg group's legs. If you pass non-NULL here, it must be
 *	the same leg as `prev_route_rl' (i.e. legs must be identically
 *	sequenced in the rlg's leg list as in the route's leg list).
 * @param prev_route_rl The leg in the route which should precede the leg
 *	being checked. If you pass NULL, the new leg will be inserted at the
 *	start of the route's legs.
 */
static route_leg_t *
rlg_update_leg(route_t *route, route_leg_group_t *rlg, route_leg_t *rl,
    const wpt_t *end_wpt, route_leg_t *prev_rlg_rl, route_leg_t *prev_route_rl)
{
	if (rl == NULL) {
		/* Route leg doesn't exist, recreate & reinsert. */
		rl = rl_new_direct(end_wpt, rlg);
		list_insert_after(&rlg->legs, prev_rlg_rl, rl);
		list_insert_after(&route->legs, prev_route_rl, rl);
	} else {
		/* Route leg exists, check settings & position in leg list. */
		ASSERT(rl->seg.type == NAVPROC_SEG_TYPE_DIR_TO_FIX);
		ASSERT(rl != prev_rlg_rl);
		ASSERT(rl != prev_route_rl);
		if (!WPT_EQ(leg_get_end_wpt(rl), end_wpt)) {
			/* End fix incorrect, reset */
			leg_set_end_wpt(rl, end_wpt);
			route->segs_dirty = B_TRUE;
		}
		ASSERT(prev_rlg_rl == NULL || prev_rlg_rl == prev_route_rl);
		if (list_prev(&rlg->legs, rl) != prev_rlg_rl) {
			/* Position in leg group leg sequence incorrect, move */
			list_remove(&rlg->legs, rl);
			list_insert_after(&rlg->legs, prev_rlg_rl, rl);
			route->segs_dirty = B_TRUE;
		}
		if (list_prev(&route->legs, rl) != prev_route_rl) {
			/* Position in route leg sequence incorrect, move */
			list_remove(&route->legs, rl);
			list_insert_after(&route->legs, prev_route_rl, rl);
			route->segs_dirty = B_TRUE;
		}
	}
	return (rl);
}

/*
 * Updates the route legs of `rlg' to correspond to the rlg settings.
 * This adds/removes route legs as necessary to complete the airway.
 */
static void
rlg_update_awy_legs(route_t *route, route_leg_group_t *rlg, bool_t lookup)
{
	ASSERT(rlg->type == ROUTE_LEG_GROUP_TYPE_AIRWAY);

	if (lookup) {
		const wpt_t *endfix;
		const airway_t *awy;
		awy = airway_db_lookup(route->navdb->awydb, rlg->awy->name,
		    !IS_NULL_WPT(&rlg->start_wpt) ? &rlg->start_wpt : NULL,
		    !IS_NULL_WPT(&rlg->end_wpt) ? rlg->end_wpt.name : NULL,
		    &endfix);
		VERIFY(awy != NULL);
		ASSERT(IS_NULL_WPT(&rlg->end_wpt) ||
		    WPT_EQ(&rlg->end_wpt, endfix));
		rlg->awy = awy;
	}

	if (IS_NULL_WPT(&rlg->start_wpt) || IS_NULL_WPT(&rlg->end_wpt)) {
		/*
		 * If we're missing either the start/end wpt, we can't
		 * contain any legs, so the flush has changed us.
		 */
		if (list_head(&rlg->legs) == NULL)
			return;
		for (route_leg_t *rl = list_head(&rlg->legs); rl != NULL;
		    rl = list_head(&rlg->legs)) {
			list_remove(&route->legs, rl);
			list_remove(&rlg->legs, rl);
			free(rl);
		}
		route->segs_dirty = B_TRUE;
	} else {
		route_leg_t *prev_route_rl = last_leg_before_rlg(route, rlg);
		route_leg_t *prev_awy_rl = NULL;
		route_leg_t *rl = list_head(&rlg->legs);
		unsigned i = 0;

		/* Locate the initial airway segment */
		for (; i < rlg->awy->num_segs &&
		    !WPT_EQ(&rlg->start_wpt, &rlg->awy->segs[i].endpt[0]); i++)
			;
		ASSERT(i < rlg->awy->num_segs);
		/* Pass over all airway segments in order & adapt our legs */
		for (; i < rlg->awy->num_segs &&
		    !WPT_EQ(&rlg->end_wpt, &rlg->awy->segs[i].endpt[0]); i++) {
			rl = rlg_update_leg(route, rlg, rl,
			    &rlg->awy->segs[i].endpt[1], prev_awy_rl,
			    prev_route_rl);
			prev_awy_rl = rl;
			prev_route_rl = rl;
			rl = list_next(&rlg->legs, rl);
		}
		/* Delete any extraneous legs */
		for (rl = list_next(&rlg->legs, prev_awy_rl); rl != NULL;
		    rl = list_next(&rlg->legs, prev_awy_rl)) {
			list_remove(&rlg->legs, rl);
			list_remove(&route->legs, rl);
			free(rl);
			route->segs_dirty = B_TRUE;
		}
	}
}

/*
 * Updates the route leg of a DIRECT rlg to correspond to the rlg's settings.
 */
static void
rlg_update_direct_leg(route_t *route, route_leg_group_t *rlg)
{
	ASSERT(rlg->type == ROUTE_LEG_GROUP_TYPE_DIRECT);
	(void) rlg_update_leg(route, rlg, list_head(&rlg->legs), &rlg->end_wpt,
	    NULL, last_leg_before_rlg(route, rlg));
}

/*
 * Returns the first non-DISCO route leg following `ref' in list `legs'
 * If `ref' is NULL, returns the first leg group (which is never a DISCO).
 */
static route_leg_t *
rl_next_ndisc(const list_t *legs, const route_leg_t *ref)
{
	if (ref == NULL)
		return (list_head(legs));

	for (route_leg_t *rl = list_next(legs, ref); rl;
	    rl = list_next(legs, rl)) {
		if (!rl->disco)
			return (rl);
	}
	return (NULL);
}

/*
 * Returns the first non-DISCO route leg group following `ref'. If `ref'
 * is NULL, returns the first leg group (which is never a DISCO).
 */
static route_leg_group_t *
rlg_next_ndisc(route_t *route, const route_leg_group_t *ref)
{
	if (ref == NULL)
		return (list_head(&route->leg_groups));

	for (route_leg_group_t *rlg = list_next(&route->leg_groups, ref); rlg;
	    rlg = list_next(&route->leg_groups, rlg)) {
		if (rlg->type != ROUTE_LEG_GROUP_TYPE_DISCO)
			return (rlg);
	}
	return (NULL);
}

/*
 * Returns the first non-DISCO route leg group preceding `ref'. If `ref'
 * is NULL, returns the last leg group (which is never a DISCO).
 */
static route_leg_group_t *
rlg_prev_ndisc(route_t *route, const route_leg_group_t *ref)
{
	if (ref == NULL)
		return (list_tail(&route->leg_groups));

	for (route_leg_group_t *rlg = list_prev(&route->leg_groups, ref); rlg;
	    rlg = list_prev(&route->leg_groups, rlg)) {
		if (rlg->type != ROUTE_LEG_GROUP_TYPE_DISCO)
			return (rlg);
	}
	return (NULL);
}

/*
 * Returns the first non-DISCO route leg preceding `ref'.
 */
static route_leg_t *
rl_prev_ndisc(route_t *route, route_leg_t *ref)
{
	for (route_leg_t *rl = list_prev(&route->legs, ref); rl;
	    rl = list_prev(&route->legs, rl)) {
		if (!rl->disco)
			return (rl);
	}
	return (NULL);
}

/*
 * Returns the last non-DISCO route leg group, checking that it's not a DISCO.
 */
static route_leg_group_t *
rlg_tail_ndisc(route_t *route)
{
	route_leg_group_t *rlg = list_tail(&route->leg_groups);
	ASSERT(rlg == NULL || rlg->type != ROUTE_LEG_GROUP_TYPE_DISCO);
	return (rlg);
}

static wpt_t
rlg_find_start_fix(const route_leg_group_t *rlg)
{
	route_leg_t *rl = list_head(&rlg->legs);
	ASSERT(rl != NULL);
	return (*navproc_seg_get_start_wpt(&rl->seg));
}

static wpt_t
rlg_find_end_wpt(const route_leg_group_t *rlg)
{
	route_leg_t *rl = list_tail(&rlg->legs);
	ASSERT(rl != NULL);
	return (*navproc_seg_get_end_wpt(&rl->seg));
}

/*
 * Returns B_TRUE if the two navprocs are related, B_FALSE otherwise.
 * Related navprocs are defined as:
 *	a) Both belonging to the same airport.
 *	b) Both being of the same class of navproc. There are two classes
 *	   of navprocs:
 *		1) Departure navprocs: SID, SID_COMMON, SID_TRANS.
 *		2) Arrival navprocs: STAR, STAR_COMMON, STAR_TRANS, FINAL,
 *		   FINAL_TRANS.
 */
static bool_t
navprocs_related(const navproc_t *nv1, const navproc_t *nv2)
{
	return (nv1->arpt == nv2->arpt &&
	    ((nv1->type <= NAVPROC_TYPE_SID_TRANS &&
	    nv2->type <= NAVPROC_TYPE_SID_TRANS) ||
	    (nv1->type >= NAVPROC_TYPE_STAR &&
	    nv2->type >= NAVPROC_TYPE_STAR)));
}

/*
 * Given a route let and current altitude, determines what adjustment to the
 * altitude is needed to satisfy the route leg's altitude constraint. If the
 * leg's altitude is unconstrained, returns the input altitude.
 */
static double
rl_alt_lim_adj(const route_leg_t *rl, double alt)
{
	alt_lim_t lim = route_l_get_alt_lim(rl);

	switch (lim.type) {
	case ALT_LIM_NONE:
		return (alt);
	case ALT_LIM_AT:
		return (lim.alt1);
	case ALT_LIM_AT_OR_ABV:
		return (MAX(alt, lim.alt1));
	case ALT_LIM_AT_OR_BLW:
		return (MIN(alt, lim.alt1));
	case ALT_LIM_BETWEEN:
		return (MAX(MIN(alt, lim.alt1), lim.alt2));
	default:
		assert(0);
	}
}

#define	ALT_GUESS_DISPLACE	100.0

/*
 * Given a route leg terminating a route segment and a segment starting
 * position, attempts to complete the route segment. This function is a
 * complement to rl_find_leg_seg. rl_find_leg_seg tries to determine a
 * leg segment's starting point from a route leg and an initial aircraft
 * position. Then if the leg segment can be completed using the same route
 * leg, it passes that leg to rl_complete_seg to fill in the rest of the
 * leg segment. Othewise it attempts to pass the *next* route leg following
 * the one being examined to try and terminate the leg segment.
 *
 * This is because route legs can be one of the following:
 *
 *	1) A leg with a definite starting point but no end point at all
 *	   (i.e. the route leg is just a single point): IF
 *	2) A leg with a definite starting point, a direction of travel
 *	   but no end point (i.e. the route leg is an intercept command):
 *	   CI, VI, CR, VR
 *	3) A leg with a definite starting point and an end point: AF,
 */
static bool_t
rl_complete_seg(const route_leg_t *rl, geo_pos2_t start, const wmm_t *wmm,
    route_seg_t *seg)
{
	if (rl->disco)
		return (B_FALSE);

	switch(rl->seg.type) {
	case NAVPROC_SEG_TYPE_ARC_TO_FIX:
		seg->type = ROUTE_SEG_TYPE_ARC;
		seg->arc.center = rl->seg.leg_cmd.dme_arc.navaid.pos;
		seg->arc.start = geo_displace_mag(&wgs84, wmm,
		    rl->seg.leg_cmd.dme_arc.navaid.pos,
		    rl->seg.leg_cmd.dme_arc.start_radial,
		    NM2MET(rl->seg.leg_cmd.dme_arc.radius));
		seg->arc.end = rl->seg.term_cond.fix.pos;
		return (B_TRUE);
	case NAVPROC_SEG_TYPE_CRS_TO_ALT:
		seg->direct.start = start;
		seg->direct.end = geo_displace_mag(&wgs84, wmm, start,
		    rl->seg.leg_cmd.hdg.hdg, NM2MET(ALT_GUESS_DISPLACE));
		return (B_TRUE);
	case NAVPROC_SEG_TYPE_CRS_TO_DME:
		seg->type = ROUTE_SEG_TYPE_DIRECT;
		seg->direct.start = start;
		seg->direct.end = calc_dist_leg_intc(start, rl, NULL, wmm);
		return (B_TRUE);
	case NAVPROC_SEG_TYPE_CRS_TO_FIX:
		seg->type = ROUTE_SEG_TYPE_DIRECT;
		seg->direct.start = start;
		seg->direct.end = rl->seg.term_cond.fix.pos;
		return (B_TRUE);
	case NAVPROC_SEG_TYPE_CRS_TO_INTCP:
		return (B_FALSE);
	case NAVPROC_SEG_TYPE_CRS_TO_RADIAL:
	case NAVPROC_SEG_TYPE_HDG_TO_RADIAL:
		seg->type = ROUTE_SEG_TYPE_DIRECT;
		seg->direct.start = start;
		seg->direct.end = calc_radial_leg_intc(start, rl, NULL, wmm);
		return (B_TRUE);
	case NAVPROC_SEG_TYPE_DIR_TO_FIX:
		seg->type = ROUTE_SEG_TYPE_DIRECT;
		seg->direct.start = start;
		seg->direct.end = rl->seg.term_cond.fix.pos;
		return (B_TRUE);
	case NAVPROC_SEG_TYPE_FIX_TO_ALT:
		seg->type = ROUTE_SEG_TYPE_DIRECT;
		seg->direct.start = rl->seg.leg_cmd.fix_crs.fix.pos;
		seg->direct.end = geo_displace_mag(&wgs84, wmm,
		    rl->seg.leg_cmd.fix_crs.fix.pos,
		    rl->seg.leg_cmd.fix_crs.crs, NM2MET(ALT_GUESS_DISPLACE));
		return (B_TRUE);
	case NAVPROC_SEG_TYPE_FIX_TO_DIST:
		seg->type = ROUTE_SEG_TYPE_DIRECT;
		seg->direct.start = rl->seg.leg_cmd.fix_crs.fix.pos;
		seg->direct.end = geo_displace_mag(&wgs84, wmm,
		    rl->seg.leg_cmd.fix_crs.fix.pos,
		    rl->seg.leg_cmd.fix_crs.crs,
		    NM2MET(rl->seg.term_cond.dist));
		return (B_TRUE);
	case NAVPROC_SEG_TYPE_FIX_TO_DME:
		seg->type = ROUTE_SEG_TYPE_DIRECT;
		seg->direct.start = rl->seg.leg_cmd.fix_crs.fix.pos;
		seg->direct.end = calc_dist_leg_intc(start, rl, NULL, wmm);
		return (B_TRUE);
	case NAVPROC_SEG_TYPE_FIX_TO_MANUAL:
		return (B_FALSE);
	case NAVPROC_SEG_TYPE_HOLD_TO_ALT:
	case NAVPROC_SEG_TYPE_HOLD_TO_FIX:
	case NAVPROC_SEG_TYPE_HOLD_TO_MANUAL:
		seg->type = ROUTE_SEG_TYPE_DIRECT;
		seg->direct.start = start;
		seg->direct.end = rl->seg.leg_cmd.hold.wpt.pos;
		return (B_TRUE);
	case NAVPROC_SEG_TYPE_INIT_FIX:
		seg->type = ROUTE_SEG_TYPE_DIRECT;
		seg->direct.start = start;
		seg->direct.end = rl->seg.leg_cmd.fix.pos;
		return (B_TRUE);
	case NAVPROC_SEG_TYPE_PROC_TURN:
		seg->type = ROUTE_SEG_TYPE_DIRECT;
		seg->direct.start = start;
		seg->direct.end = rl->seg.leg_cmd.proc_turn.startpt.pos;
		return (B_TRUE);
	case NAVPROC_SEG_TYPE_RADIUS_ARC_TO_FIX:
		return (B_FALSE);
	case NAVPROC_SEG_TYPE_TRK_TO_FIX:
		seg->type = ROUTE_SEG_TYPE_DIRECT;
		seg->direct.start = start;
		seg->direct.end = rl->seg.term_cond.fix.pos;
		return (B_TRUE);
	case NAVPROC_SEG_TYPE_HDG_TO_ALT:
		seg->type = ROUTE_SEG_TYPE_DIRECT;
		seg->direct.start = start;
		seg->direct.end = geo_displace_mag(&wgs84, wmm, start,
		    rl->seg.leg_cmd.hdg.hdg, NM2MET(ALT_GUESS_DISPLACE));
		return (B_TRUE);
	case NAVPROC_SEG_TYPE_HDG_TO_DME:
		seg->type = ROUTE_SEG_TYPE_DIRECT;
		seg->direct.start = start;
		seg->direct.end = calc_dist_leg_intc(start, rl, NULL, wmm);
		return (B_TRUE);
	case NAVPROC_SEG_TYPE_HDG_TO_INTCP:
	case NAVPROC_SEG_TYPE_HDG_TO_MANUAL:
		return (B_FALSE);
	default:
		assert(0);
	}
}

/*
 * Attempts to construct a leg segment for a leg starting at `rl'. This
 * requires that the navproc segment type of the leg is one of:
 *	1) AF: this type has a definite start & end position.
 *	2) FA, FC, FD: this type has a definable end based on a previous
 *	   segment end position.
 *	3) IF: this type has a start position. We then attempt to connect it
 *	   to the next leg.
 *
 * @param rl Current route leg for which to construct the leg segment.
 * @param oldpos Previous known aircraft position prior to transitioning to
 *	the current leg.
 * @param next_rl Next route leg following the current. NULL if unknown.
 * @param wmm World magnetic model in effect for converting headings.
 * @param seg Pointer to a route segment structure that will be filled with
 *	the leg segment information.
 *
 * @return B_TRUE if constructing the leg segment succeeded, B_FALSE if not.
 */
static bool_t
rl_find_leg_seg(const route_leg_t *rl, geo_pos2_t oldpos,
    const route_leg_t *next_rl, const wmm_t *wmm, route_seg_t *seg)
{
	switch (rl->seg.type) {
	case NAVPROC_SEG_TYPE_ARC_TO_FIX:
	case NAVPROC_SEG_TYPE_FIX_TO_ALT:
	case NAVPROC_SEG_TYPE_FIX_TO_DIST:
	case NAVPROC_SEG_TYPE_FIX_TO_DME:
		return (rl_complete_seg(rl, oldpos, wmm, seg));
	case NAVPROC_SEG_TYPE_INIT_FIX:
		if (next_rl == NULL)
			return (B_FALSE);
		return (rl_complete_seg(next_rl, rl->seg.leg_cmd.fix.pos,
		    wmm, seg));
	case NAVPROC_SEG_TYPE_PROC_TURN:
		/* TODO */
		return (B_FALSE);
	default:
		return (B_FALSE);
	}
}

/*
 * Finds the geodetic coordinate midpoint between two geodetic coordinates.
 */
static geo_pos2_t
find_geo_midpoint(geo_pos2_t a, geo_pos2_t b)
{
	vect3_t a_v = geo2ecef(GEO2_TO_GEO3(a, 0), &wgs84);
	vect3_t b_v = geo2ecef(GEO2_TO_GEO3(b, 0), &wgs84);
	vect3_t r_v = vect3_mean(a_v, b_v);
	geo_pos3_t r = ecef2geo(r_v, &wgs84);
	return (GEO3_TO_GEO2(r));
}

/*
 * Determines the best intersection of a flight route and a DME circle.
 */
static geo_pos2_t
find_best_circ_isect(geo_pos2_t cur_pos, double hdg, geo_pos2_t center_pos,
    double radius, const wmm_t *wmm)
{
	geo_pos2_t midpt = find_geo_midpoint(cur_pos, center_pos);
	fpp_t fpp = gnomo_fpp_init(midpt, 0, &wgs84, B_TRUE);
	vect2_t cur_pos_v, dir_v, center_v;
	vect2_t isects[2];
	vect2_t c2i[2];		/* from cur_pos to respective isect point */
	vect2_t res;
	unsigned n;

	cur_pos_v = geo2fpp(cur_pos, &fpp);
	dir_v = hdg2dir(wmm_mag2true(wmm, hdg, GEO2_TO_GEO3(cur_pos, 0)));
	center_v = geo2fpp(center_pos, &fpp);
	n = vect2circ_isect(dir_v, cur_pos_v, center_v, radius, B_FALSE,
	    isects);

	if (n == 0)
		return (NULL_GEO_POS2);
	if (n == 1) {
		c2i[0] = vect2_sub(isects[0], cur_pos_v);
		if (!SAME_DIR(c2i[0], dir_v))
			return (NULL_GEO_POS2);
		res = isects[0];
	} else {
		c2i[0] = vect2_sub(isects[0], cur_pos_v);
		c2i[1] = vect2_sub(isects[1], cur_pos_v);
		if (vect2_abs(c2i[0]) < vect2_abs(c2i[1]) &&
		    SAME_DIR(c2i[0], dir_v))
			res = isects[0];
		else if (SAME_DIR(c2i[1], dir_v))
			res = isects[1];
		else
			return (NULL_GEO_POS2);
	}

	return (fpp2geo(res, &fpp));
}

static geo_pos2_t
calc_manual_leg_intc(geo_pos2_t cur_pos, const route_leg_t *rl,
    const list_t *legs, const wmm_t *wmm)
{
	UNUSED(cur_pos);
	UNUSED(rl);
	UNUSED(legs);
	UNUSED(wmm);

	ASSERT(rl->seg.type == NAVPROC_SEG_TYPE_FIX_TO_MANUAL ||
	    rl->seg.type == NAVPROC_SEG_TYPE_HDG_TO_MANUAL);
	return (NULL_GEO_POS2);
}

static geo_pos2_t
calc_dir_leg_intc(geo_pos2_t cur_pos, const route_leg_t *rl,
    const list_t *legs, const wmm_t *wmm)
{
	UNUSED(cur_pos);
	UNUSED(legs);
	UNUSED(wmm);

	switch (rl->seg.type) {
	case NAVPROC_SEG_TYPE_ARC_TO_FIX:
	case NAVPROC_SEG_TYPE_CRS_TO_FIX:
	case NAVPROC_SEG_TYPE_DIR_TO_FIX:
	case NAVPROC_SEG_TYPE_RADIUS_ARC_TO_FIX:
	case NAVPROC_SEG_TYPE_TRK_TO_FIX:
		return (rl->seg.term_cond.fix.pos);
	case NAVPROC_SEG_TYPE_INIT_FIX:
		return (rl->seg.leg_cmd.fix.pos);
	case NAVPROC_SEG_TYPE_HOLD_TO_ALT:
	case NAVPROC_SEG_TYPE_HOLD_TO_FIX:
	case NAVPROC_SEG_TYPE_HOLD_TO_MANUAL:
		return (rl->seg.leg_cmd.hold.wpt.pos);
	default:
		assert(0);
	}
}

static geo_pos2_t
calc_dist_leg_intc(geo_pos2_t cur_pos, const route_leg_t *rl,
    const list_t *legs, const wmm_t *wmm)
{
	double hdg, dist;	/* hdg in deg, dist in NM */
	geo_pos2_t center;

	UNUSED(legs);
	if (IS_NULL_GEO_POS(cur_pos)) {
		/* Can't resolve without a starting point */
		openfmc_log(OPENFMC_LOG_ERR, "Cannot resolve %s leg to "
		    "%s/%.1lf: missing start pos",
		    navproc_seg_type2str(rl->seg.type),
		    rl->seg.term_cond.dme.navaid.name,
		    rl->seg.term_cond.dme.dist);
		return (NULL_GEO_POS2);
	}
	ASSERT(rl->seg.type == NAVPROC_SEG_TYPE_CRS_TO_RADIAL ||
	    rl->seg.type == NAVPROC_SEG_TYPE_FIX_TO_DIST ||
	    rl->seg.type == NAVPROC_SEG_TYPE_FIX_TO_DME ||
	    rl->seg.type == NAVPROC_SEG_TYPE_HDG_TO_DME);
	if (rl->seg.type == NAVPROC_SEG_TYPE_CRS_TO_DME) {
		hdg = rl->seg.leg_cmd.hdg.hdg;
		center = rl->seg.term_cond.dme.navaid.pos;
		dist = rl->seg.term_cond.dme.dist;
	} else if (rl->seg.type == NAVPROC_SEG_TYPE_FIX_TO_DIST) {
		cur_pos = rl->seg.leg_cmd.fix_crs.fix.pos;
		hdg = rl->seg.leg_cmd.fix_crs.crs;
		center = rl->seg.leg_cmd.fix_crs.fix.pos;
		dist = rl->seg.term_cond.dist;
	} else if (rl->seg.type == NAVPROC_SEG_TYPE_FIX_TO_DME) {
		cur_pos = rl->seg.leg_cmd.fix_crs.fix.pos;
		hdg = rl->seg.leg_cmd.fix_crs.crs;
		center = rl->seg.term_cond.dme.navaid.pos;
		dist = rl->seg.term_cond.dme.dist;
	} else {	/* NAVPROC_SEG_TYPE_HDG_TO_DME */
		hdg = rl->seg.leg_cmd.hdg.hdg;
		center = rl->seg.term_cond.dme.navaid.pos;
		dist = rl->seg.term_cond.dme.dist;
	}

	return (find_best_circ_isect(cur_pos, hdg, center, dist, wmm));
}

static geo_pos2_t
calc_radial_leg_intc(geo_pos2_t cur_pos, const route_leg_t *rl,
    const list_t *legs, const wmm_t *wmm)
{
	fpp_t fpp = gnomo_fpp_init(find_geo_midpoint(cur_pos,
	    rl->seg.term_cond.radial.navaid.pos), 0, &wgs84, B_TRUE);
	vect2_t dir_v, cur_pos_v, navaid_v, radial_dir_v, isect, c2i;

	UNUSED(legs);
	ASSERT(rl->seg.type == NAVPROC_SEG_TYPE_CRS_TO_RADIAL ||
	    rl->seg.type == NAVPROC_SEG_TYPE_HDG_TO_RADIAL);

	dir_v = DIR_V(rl->seg.leg_cmd.hdg.hdg);
	radial_dir_v = DIR_V(rl->seg.term_cond.radial.radial);
	navaid_v = geo2fpp(rl->seg.term_cond.radial.navaid.pos, &fpp);
	cur_pos_v = geo2fpp(GEO3_TO_GEO2(cur_pos), &fpp);

	isect = vect2vect_isect(dir_v, cur_pos_v, radial_dir_v, navaid_v,
	    B_FALSE);
	c2i = vect2_sub(isect, cur_pos_v);

	/* Check intersection exists and lies in direction of travel. */
	if (!IS_NULL_VECT(isect) && SAME_DIR(c2i, dir_v)) {
		return (fpp2geo(isect, &fpp));
	} else {
		return (NULL_GEO_POS2);
	}
}

static geo_pos2_t
calc_vect_leg_intc(const geo_pos2_t cur_pos, const route_leg_t *rl,
    const list_t *legs, const wmm_t *wmm)
{
	const route_leg_t *next_rl = rl_next_ndisc(legs, rl);
	route_seg_t next_seg;
	double hdg;

	UNUSED(cur_pos);
	if (IS_NULL_GEO_POS(cur_pos) || next_rl == NULL)
		return (NULL_GEO_POS2);

	ASSERT(rl->seg.type == NAVPROC_SEG_TYPE_CRS_TO_INTCP ||
	    rl->seg.type == NAVPROC_SEG_TYPE_HDG_TO_INTCP);
	hdg = rl->seg.leg_cmd.hdg.hdg;

	if (!rl_find_leg_seg(next_rl, cur_pos, rl_next_ndisc(legs, next_rl),
	    wmm, &next_seg))
		return (NULL_GEO_POS2);

	if (next_seg.type == ROUTE_SEG_TYPE_DIRECT) {
		fpp_t fpp = gnomo_fpp_init(GEO3_TO_GEO2(cur_pos), 0, &wgs84,
		    B_TRUE);
		vect2_t cur_pos_v, dir_v, start_v, end_v, s2e, isect, c2i;

		cur_pos_v = geo2fpp(cur_pos, &fpp);
		start_v = geo2fpp(next_seg.direct.start, &fpp);
		end_v = geo2fpp(next_seg.direct.end, &fpp);
		s2e = vect2_sub(end_v, start_v);
		dir_v = DIR_V(hdg);
		isect = vect2vect_isect(cur_pos_v, dir_v, start_v, s2e,
		    B_FALSE);
		c2i = vect2_sub(isect, cur_pos_v);
		if (IS_NULL_VECT(isect) || !SAME_DIR(c2i, dir_v))
			return (NULL_GEO_POS2);
		return (fpp2geo(isect, &fpp));
	} else {
		ASSERT(next_seg.type == ROUTE_SEG_TYPE_ARC);
		return (find_best_circ_isect(cur_pos, hdg, next_seg.arc.center,
		    arc_seg_get_radius(&next_seg), wmm));
	}
}

static geo_pos2_t
calc_alt_leg_intc(geo_pos2_t cur_pos, const route_leg_t *rl,
    const list_t *legs, const wmm_t *wmm)
{
	/* TODO */
	UNUSED(cur_pos);
	UNUSED(rl);
	UNUSED(legs);
	UNUSED(wmm);
	return (NULL_GEO_POS2);
}

static geo_pos2_t
calc_proc_leg_intc(geo_pos2_t cur_pos, const route_leg_t *rl,
    const list_t *legs, const wmm_t *wmm)
{
	/* TODO */
	UNUSED(cur_pos);
	UNUSED(rl);
	UNUSED(legs);
	UNUSED(wmm);
	return (NULL_GEO_POS2);
}

static geo_pos2_t
calc_leg_start_pos(const route_leg_t *targ_rl, const route_leg_t *rl,
    geo_pos2_t cur_pos, const list_t *legs)
{
#define	PROJ_LIMIT	100000
	const route_leg_group_t	*rlg = targ_rl->rlg;
	const route_t		*route = rlg->route;

	ASSERT(targ_rl != NULL && rl != NULL);
	ASSERT(!IS_NULL_GEO_POS(cur_pos));

	for (; rl != targ_rl; rl = list_next(legs, rl)) {
		ASSERT(rl != NULL);
		if (rl->disco) {
			cur_pos = NULL_GEO_POS2;
			continue;
		}
		ASSERT(leg_intc_func_tbl[rl->seg.type] != NULL);
		cur_pos = leg_intc_func_tbl[rl->seg.type](cur_pos, rl,
		    legs, route->navdb->wmm);
	}

	return (cur_pos);
}

/*
 * Looks for the first suitable starting position on a route for leg segment
 * computation. Suitable starting positions are (in order of preference):
 *	1) departure runway threshold
 *	2) departure airport reference point
 *	3) any rlg up to lim_rlg that has a defined start_wpt
 * If a starting position is found, it is returned, otherwise NULL_GEO_POS3
 * is returned.
 */
static geo_pos3_t
route_first_start_pos(const route_t *route, const route_leg_group_t *lim_rlg,
    const route_leg_group_t **rlg_start)
{
	double alt;

	if (route->dep_rwy != NULL) {
		*rlg_start = NULL;
		return (route->dep_rwy->thr_pos);
	}
	if (route->dep != NULL) {
		*rlg_start = NULL;
		return (route->dep->refpt);
	}
	for (const route_leg_group_t *rlg = list_head(&route->leg_groups); rlg;
	    rlg = list_next(&route->leg_groups, rlg)) {
		if (!IS_NULL_WPT(&rlg->start_wpt)) {
			*rlg_start = rlg;
			return (GEO2_TO_GEO3(rlg->start_wpt.pos, 0));
		}
		if (rlg == lim_rlg) {
			/* Searched too far */
			break;
		}
		for (const route_leg_t *rl = list_head(&rlg->legs); rl;
		    rl = list_next(&rlg->legs, rl)) {
			/*
			 * Figure out the altitude constraints to guesstimate
			 * suitable start point altitude.
			 */
			alt = rl_alt_lim_adj(rl, alt);
		}
	}
	*rlg_start = NULL;
	return (NULL_GEO_POS3);
}

/*
 * Returns B_TRUE if the two route leg groups intercept each other, B_FALSE
 * otherwise. Interception is defined as:
 */
static bool_t
proc_rlgs_intc(const route_leg_group_t *rlg1, const route_leg_group_t *rlg2)
{
	const route_t		*route = rlg1->route;
	wpt_t			rlg1_end_fix = rlg_find_end_wpt(rlg1);
	wpt_t			rlg2_start_fix = rlg_find_start_fix(rlg2);
	geo_pos3_t		route_start_pos;
	geo_pos2_t		leg_start, intcpt;
	const route_leg_t	*rl1;
	const route_leg_group_t	*rlg_start;

	ASSERT(rlg1 != NULL && rlg2 != NULL);
	ASSERT(rlg1->route == rlg2->route);

	/* Overlapping fixes means intercept is guaranteed */
	if (WPT_EQ_POS(&rlg2_start_fix, &rlg1_end_fix))
		return (B_TRUE);

	/*
	 * We'll need to calculate an intercept. Check if the last leg on
	 * rlg1 is even capable of that.
	 */
	rl1 = list_tail(&rlg1->legs);
	if (rl1->seg.type != NAVPROC_SEG_TYPE_CRS_TO_INTCP &&
	    rl1->seg.type != NAVPROC_SEG_TYPE_HDG_TO_INTCP &&
	    rl1->seg.type != NAVPROC_SEG_TYPE_PROC_TURN)
		return (B_FALSE);

	/* We'll need an initial route start position. */
	route_start_pos = route_first_start_pos(route, rlg1, &rlg_start);
	if (IS_NULL_GEO_POS(route_start_pos))
		return (B_FALSE);
	leg_start = calc_leg_start_pos(rl1, rlg_start != NULL ?
	    list_head(&rlg_start->legs) : list_head(&route->legs),
	    GEO3_TO_GEO2(route_start_pos), &route->legs);

	intcpt = calc_vect_leg_intc(leg_start, rl1, &route->legs,
	    route->navdb->wmm);

	return (!IS_NULL_GEO_POS(intcpt));
}

/*
 * Given an airport pointer, investigates any extraneous links and
 * references to this airport's structures and removes them from the route.
 * This involves iterating through all runways and procedures (and their
 * associated legs) and removing them.
 */
static void
route_remove_arpt_links(route_t *route, const airport_t *arpt)
{
#define	REM_BACKREF(obj) \
	do { \
		if (route->obj != NULL && route->obj->arpt == arpt) \
			route->obj = NULL; \
	} while (0)
	/* First check runways & procedures */
	REM_BACKREF(dep_rwy);
	REM_BACKREF(sid);
	REM_BACKREF(sidcm);
	REM_BACKREF(sidtr);
	REM_BACKREF(startr);
	REM_BACKREF(starcm);
	REM_BACKREF(star);
	REM_BACKREF(apprtr);
	REM_BACKREF(appr);
#undef	REM_BACKREF
	/* Now check leg groups for offending procedures & route legs */
	for (route_leg_group_t *rlg = list_head(&route->leg_groups); rlg;) {
		route_leg_group_t *rlg_next = rlg_next_ndisc(route, rlg);
		if (rlg->type == ROUTE_LEG_GROUP_TYPE_PROC &&
		    rlg->proc->arpt == arpt) {
			rlg_bypass(route, rlg, B_FALSE, B_FALSE);
		}
		rlg = rlg_next;
	}

	route->segs_dirty = B_TRUE;
}

/*
 * Attempts to connect two route leg groups. This function performs no
 * checks of the two leg groups being adjacent to each other. The rules
 * for route leg group connection are as follows:
 *
 * 1) If either the first or second leg group is a disco, no-op.
 *
 * 2) If the first leg group is an AIRWAY:
 *	a) If the second leg group is an AIRWAY:
 *		*) If the two airways already share an end/start wpt, no-op.
 *		*) Attempts to locate a suitable intersection point between
 *		   the airways and set that as the end/start wpt combo. If no
 *		   intersection point was found, returns ERR_AWY_AWY_MISMATCH.
 *	b) If the second leg group is a DIRECT:
 *		*) If the preceding airway already had an endpoint set, will
 *		   simply connect the second direct leg group's wpt to the
 *		   airway's endpoint.
 *		*) If the preceding airway had no endpoint set but does have
 *		   a start wpt, will attempt locate the subsequent direct leg
 *		   group's end wpt on the airway. If successful, will delete
 *		   the direct leg group and instead set the airway's end point
 *		   to that wpt. If not, will return ERR_AWY_WPT_MISMATCH.
 *		*) If the preceding airway is lacking a start_wpt, will always
 *		   return ERR_AWY_WPT_MISMATCH.
 *	c) If the second leg group is a PROC:
 *		*) If the airway's end wpt is identical to the procedure's
 *		   start wpt, no-op.
 *		*) Otherwise tries to look for the procedure's start wpt on
 *		   the airway and attempts to find a way to extend the airway
 *		   segment up to the procedure's start. If there is no wpt
 *		   on the airway that ends at the procedure, returns
 *		   ERR_AWY_PROC_MISMATCH.
 *
 * 3) If the first leg group is a DIRECT or PROC:
 *	a) If the second leg group is an AIRWAY, attempts to locate the
 *	   first LG's end wpt on the airway. If the airway already has an end
 *	   wpt defined, will take that into consideration so that we maintain
 *	   correct airway orientation. If the first LG's end wpt is not present
 *	   on the airway, returns ERR_AWY_WPT_MISMATCH.
 *	b) If the second leg group is a DIRECT, sets the second LG's start
 *	   wpt to the first LG's end wpt.
 *	c) If the second leg group is a PROC, checks if the first LG's end wpt
 *	   is identical to the PROC's start wpt. If it is, no-op, otherwise
 *	   returns ERR_WPT_PROC_MISMATCH.
 */
static err_t
rlg_try_connect(route_t *route, route_leg_group_t *prev_rlg,
    route_leg_group_t **next_rlgpp, bool_t allow_mod, bool_t allow_add_legs)
{
	route_leg_group_t *next_rlg = *next_rlgpp;

	ASSERT(prev_rlg != NULL);
	ASSERT(next_rlg != NULL);

	if (prev_rlg->type == ROUTE_LEG_GROUP_TYPE_DISCO ||
	    next_rlg->type == ROUTE_LEG_GROUP_TYPE_DISCO) {
		/* Check we don't have to adjacent disco's */
		ASSERT(prev_rlg->type != ROUTE_LEG_GROUP_TYPE_DISCO ||
		    next_rlg->type != ROUTE_LEG_GROUP_TYPE_DISCO);
		return (ERR_OK);
	}

	switch (prev_rlg->type) {
	case ROUTE_LEG_GROUP_TYPE_AIRWAY:
		/* AWY -> [something] */
		switch (next_rlg->type) {
		case ROUTE_LEG_GROUP_TYPE_AIRWAY: {
			/* AWY -> AWY */
			const wpt_t *isect;
			bool_t awy_end_overlap;

			if (WPT_EQ(&prev_rlg->end_wpt, &next_rlg->start_wpt))
				break;

			if (!allow_mod || !allow_add_legs)
				return (ERR_AWY_AWY_MISMATCH);

			/* locate: AWY x AWY */
			isect = airway_db_lookup_awy_intersection(
			    route->navdb->awydb, prev_rlg->awy->name,
			    prev_rlg->start_wpt.name, next_rlg->awy->name);
			if (isect == NULL)
				return (ERR_AWY_AWY_MISMATCH);
			prev_rlg->end_wpt = *isect;
			next_rlg->start_wpt = *isect;
			/*
			 * This might have shortened the airway to where its
			 * start/end starts to overlap. Resolve this by deleting
			 * the awy's end wpt and reconnecting it.
			 */
			awy_end_overlap = WPT_EQ(&next_rlg->end_wpt,
			    &next_rlg->start_wpt);
			if (awy_end_overlap)
				next_rlg->end_wpt = null_wpt;
			rlg_update_awy_legs(route, prev_rlg, B_TRUE);
			rlg_update_awy_legs(route, next_rlg, B_TRUE);
			if (awy_end_overlap) {
				rlg_connect(route, next_rlg,
				    rlg_next_ndisc(route, next_rlg), allow_mod,
				    allow_add_legs);
			}
			route->segs_dirty = B_TRUE;
			break;
		}
		case ROUTE_LEG_GROUP_TYPE_DIRECT:
			/* AWY -> DIRECT */
			if (!IS_NULL_WPT(&prev_rlg->end_wpt)) {
				/*
				 * AWY and DIRECT both end at same wpt, we
				 * can't have that.
				 */
				if (WPT_EQ(&prev_rlg->end_wpt,
				    &next_rlg->end_wpt))
					return (ERR_AWY_WPT_MISMATCH);
				if (!IS_NULL_WPT(&next_rlg->start_wpt) &&
				    !allow_mod)
					return (ERR_AWY_WPT_MISMATCH);
				next_rlg->start_wpt = prev_rlg->end_wpt;
				rlg_update_direct_leg(route, next_rlg);

				route->segs_dirty = B_TRUE;
			} else {
				/*
				 * Preceding airway is missing the endpoint,
				 * so try to locate the next DIRECT on the
				 * airway and remove the DIRECT leg group.
				 * If airway has null start_wpt, this will
				 * never succeed.
				 */
				const wpt_t *newendfix;
				const airway_t *newawy;

				if (!allow_mod)
					return (ERR_AWY_WPT_MISMATCH);
				newawy = airway_db_lookup(route->navdb->awydb,
				    prev_rlg->awy->name, &prev_rlg->start_wpt,
				    next_rlg->end_wpt.name, &newendfix);
				if (newawy == NULL ||
				    !WPT_EQ(newendfix, &next_rlg->end_wpt))
					return (ERR_AWY_WPT_MISMATCH);
				prev_rlg->awy = newawy;
				prev_rlg->end_wpt = next_rlg->end_wpt;
				rlg_update_awy_legs(route, prev_rlg, B_FALSE);
				rlg_destroy(route, next_rlg);

				route->segs_dirty = B_TRUE;
			}
			break;
		case ROUTE_LEG_GROUP_TYPE_PROC: {
			/* AWY -> PROC */
			const airway_t *newawy;

			ASSERT(!IS_NULL_WPT(&next_rlg->start_wpt));
			if (WPT_EQ(&prev_rlg->end_wpt, &next_rlg->start_wpt))
				/* already connected */
				break;
			if (!allow_mod)
				return (ERR_AWY_PROC_MISMATCH);
			newawy = airway_db_lookup(route->navdb->awydb,
			    prev_rlg->awy->name, &prev_rlg->start_wpt,
			    next_rlg->start_wpt.name, NULL);
			if (newawy == NULL)
				return (ERR_AWY_PROC_MISMATCH);
			prev_rlg->awy = newawy;
			prev_rlg->end_wpt = next_rlg->start_wpt;
			rlg_update_awy_legs(route, prev_rlg, B_FALSE);
			route->segs_dirty = B_TRUE;
			break;
		}
		default:
			assert(0);
		}
		break;
	case ROUTE_LEG_GROUP_TYPE_DIRECT:
	case ROUTE_LEG_GROUP_TYPE_PROC:
		/* [DIRECT|PROC] -> [something] */
		switch (next_rlg->type) {
		case ROUTE_LEG_GROUP_TYPE_AIRWAY: {
			/* DIRECT -> AWY */
			const wpt_t *newendfix;
			const airway_t *newawy;

			if (WPT_EQ(&prev_rlg->end_wpt, &next_rlg->start_wpt))
				/* already connected */
				break;
			if (!allow_mod)
				return (ERR_AWY_PROC_MISMATCH);
			newawy = airway_db_lookup(route->navdb->awydb,
			    next_rlg->awy->name, &prev_rlg->end_wpt,
			    IS_NULL_WPT(&next_rlg->end_wpt) ? NULL :
			    next_rlg->end_wpt.name, &newendfix);
			if (newawy != NULL &&
			    (IS_NULL_WPT(&next_rlg->end_wpt) ||
			    WPT_EQ(&next_rlg->end_wpt, newendfix))) {
				next_rlg->awy = newawy;
				next_rlg->start_wpt = prev_rlg->end_wpt;
				rlg_update_awy_legs(route, next_rlg, B_FALSE);

				route->segs_dirty = B_TRUE;
			} else {
				return (ERR_AWY_PROC_MISMATCH);
			}
			break;
		}
		case ROUTE_LEG_GROUP_TYPE_DIRECT:
			/* [DIRECT|PROC] -> DIRECT */
			ASSERT(!IS_NULL_WPT(&prev_rlg->end_wpt) ||
			    prev_rlg->type == ROUTE_LEG_GROUP_TYPE_PROC);
			if (IS_NULL_WPT(&prev_rlg->end_wpt))
				return (ERR_WPT_PROC_MISMATCH);
			if (WPT_EQ(&prev_rlg->end_wpt, &next_rlg->end_wpt)) {
				if (allow_mod) {
					/* Kill duplicates if allowed to */
					route_leg_group_t *new_next_rlg =
					    rlg_next_ndisc(route, next_rlg);
					rlg_destroy(route, next_rlg);
					rlg_connect(route, prev_rlg,
					    new_next_rlg, B_TRUE,
					    allow_add_legs);
					*next_rlgpp = NULL;
					break;
				} else {
					return (ERR_DUPLICATE_LEG);
				}
			}
			if (!IS_NULL_WPT(&next_rlg->start_wpt) && !allow_mod)
				return (ERR_WPT_PROC_MISMATCH);
			next_rlg->start_wpt = prev_rlg->end_wpt;
			break;
		case ROUTE_LEG_GROUP_TYPE_PROC:
			/* [DIRECT|PROC] -> PROC */
			/* If the fixes are equal, we're done */
			if (WPT_EQ(&prev_rlg->end_wpt, &next_rlg->start_wpt))
				break;
			if (prev_rlg->type == ROUTE_LEG_GROUP_TYPE_DIRECT) {
				/* DIRECT -> PROC cannot intercept */
				return (ERR_WPT_PROC_MISMATCH);
			}
			/*
			 * As a special case, we always consider sequenced
			 * procedures to be continuous PROVIDED they end in a
			 * suitable (INTC) leg.
			 */
			if (navprocs_related(prev_rlg->proc, next_rlg->proc) &&
			    proc_rlgs_intc(prev_rlg, next_rlg)) {
				wpt_t new_end = rlg_find_end_wpt(prev_rlg);
				if (!IS_NULL_WPT(&new_end)) {
					/*
					 * This is just for display convenience.
					 * The segment generation algorithm
					 * ignores start_wpt/end_wpt.
					 */
					prev_rlg->end_wpt = new_end;
				}
				break;
			}
			return (ERR_WPT_PROC_MISMATCH);
		default:
			assert(0);
		}
		break;
	default:
		assert(0);
	}

	return (ERR_OK);
}

/*
 * Returns B_TRUE iff rlg1 and rlg2 are separated by exactly one disco rlg.
 * rlg1 MUST precede rlg2.
 */
static bool_t
only_disco_between(route_t *route, route_leg_group_t *rlg1,
    route_leg_group_t *rlg2)
{
	route_leg_group_t *next_rlg;

	/* Check if next rlg is anything but disco */
	next_rlg = list_next(&route->leg_groups, rlg1);
	ASSERT(next_rlg != NULL);
	if (next_rlg->type != ROUTE_LEG_GROUP_TYPE_DISCO)
		return (B_FALSE);

	/* Check if next thing after disco is anything but rlg2 */
	next_rlg = list_next(&route->leg_groups, next_rlg);
	if (next_rlg != rlg2) {
		ASSERT(next_rlg != NULL);
		return (B_FALSE);
	}

	return (B_TRUE);
}

/*
 * Given two leg groups, deletes any intervening leg groups and brings the
 * two leg groups together to be adjacent. The passed two leg groups
 * MUST follow each other in sequence from prev_rlg to next_rlg.
 *
 * @param route The route to which the two leg groups belong.
 * @param prev_rlg The first route leg group to bring together.
 * @param next_rlg The second route leg group to bring together.
 */
static void
rlg_bring_together(route_t *route, route_leg_group_t *prev_rlg,
    route_leg_group_t *next_rlg)
{
	for (route_leg_group_t *rlg = list_next(&route->leg_groups, prev_rlg);
	    rlg != next_rlg; rlg = list_next(&route->leg_groups, prev_rlg)) {
		ASSERT(rlg != NULL);
		rlg_destroy(route, rlg);
		route->segs_dirty = B_TRUE;
	}
}

/*
 * Given two leg groups, brings them adjacent to each other (deleting any
 * intervening leg groups) and attempts to connect them. If the connection
 * is not possible, inserts a disco between them. The passed two leg groups
 * MUST follow each other in sequence from prev_rlg to next_rlg.
 *
 * @param route The route on which to connect the leg groups.
 * @param prev_rlg The first route leg group to connect.
 * @param next_rlg The second route leg group to connect.
 */
static void
rlg_connect(route_t *route, route_leg_group_t *prev_rlg,
    route_leg_group_t *next_rlg, bool_t allow_mod, bool_t allow_add_legs)
{
	/*
	 * If both pointers are NULL, the route leg group list must be empty,
	 * because this is not a valid scenario:
	 *
	 *            (removed)
	 * [    disco   \RLG/   disco    ]
	 *  ^                           ^
	 *  |                           |
	 * prev_rlg ------------- next_rlg
	 */
	if (prev_rlg == NULL && next_rlg == NULL) {
		ASSERT(list_is_empty(&route->leg_groups));
		return;
	}
	if (prev_rlg == NULL) {
		/*
		 * Check that there is at most one leg before this one
		 * and it's a disco.
		 */
		prev_rlg = list_prev(&route->leg_groups, next_rlg);
		if (prev_rlg != NULL) {
			ASSERT(prev_rlg == list_head(&route->leg_groups) &&
			    prev_rlg->type == ROUTE_LEG_GROUP_TYPE_DISCO);
			rlg_destroy(route, prev_rlg);
		}
		return;
	}
	if (next_rlg == NULL) {
		/*
		 * Check mirror scenario in opposite direction.
		 */
		next_rlg = list_next(&route->leg_groups, prev_rlg);
		if (next_rlg != NULL) {
			ASSERT(next_rlg == list_tail(&route->leg_groups) &&
			    next_rlg->type == ROUTE_LEG_GROUP_TYPE_DISCO);
			rlg_destroy(route, next_rlg);
		}
		return;
	}

	if (prev_rlg->type == ROUTE_LEG_GROUP_TYPE_DISCO ||
	    next_rlg->type == ROUTE_LEG_GROUP_TYPE_DISCO) {
		/* Kill one disco and we're done */
		rlg_bring_together(route, prev_rlg, next_rlg);
		rlg_destroy(route, next_rlg);
	} else if (rlg_try_connect(route, prev_rlg, &next_rlg, allow_mod,
	    allow_add_legs) == ERR_OK) {
		/* Need to recheck that the next leg wasn't deleted as dup */
		if (next_rlg != NULL) {
			/* Connection succeeded, delete any intervening legs */
			rlg_bring_together(route, prev_rlg, next_rlg);
		}
	} else if (!only_disco_between(route, prev_rlg, next_rlg)) {
		/* Connection failed and we need to insert new disco */
		rlg_bring_together(route, prev_rlg, next_rlg);
		rlg_new_disco(route, prev_rlg);
		route->segs_dirty = B_TRUE;
	}
}

/*
 * Given a route leg group, attempts to reconnect it to its non-DISCO neighbors.
 */
void
rlg_connect_neigh(route_t *route, route_leg_group_t *rlg, bool_t allow_mod,
    bool_t allow_add_legs)
{
	rlg_connect(route, rlg_prev_ndisc(route, rlg), rlg, allow_mod,
	    allow_add_legs);
	rlg_connect(route, rlg, rlg_next_ndisc(route, rlg), allow_mod,
	    allow_add_legs);
}

/*
 * Given a route leg, deletes its entire route leg group and reconnects
 * its former neighbors with each other, entirely bypassing the deleted
 * route leg group.
 */
static void
rlg_bypass(route_t *route, route_leg_group_t *rlg, bool_t allow_mod,
    bool_t allow_add_legs)
{
	route_leg_group_t *prev_rlg = rlg_prev_ndisc(route, rlg);
	route_leg_group_t *next_rlg = rlg_next_ndisc(route, rlg);

	rlg_destroy(route, rlg);
	rlg_connect(route, prev_rlg, next_rlg, allow_mod, allow_add_legs);
}

/*
 * Given a route leg `lim_rl' of a procedure, shortens the procedure either
 * from the left or right up to (but not including) `lim_rl' and attempts to
 * reconnect it with its neighbors.
 */
static void
rlg_shorten_proc(route_t *route, route_leg_t *lim_rl, bool_t left)
{
	route_leg_group_t *rlg = lim_rl->rlg;

	ASSERT(rlg->type == ROUTE_LEG_GROUP_TYPE_PROC);
	for (route_leg_t *rl = left ? list_head(&rlg->legs) :
	    list_next(&rlg->legs, lim_rl); rl != NULL && rl != lim_rl;
	    rl = left ? list_head(&rlg->legs) :
	    list_next(&rlg->legs, lim_rl)) {
		list_remove(&route->legs, rl);
		list_remove(&rlg->legs, rl);
		free(rl);
	}
	if (left)
		rlg->start_wpt = rlg_find_start_fix(rlg);
	else
		rlg->end_wpt = rlg_find_end_wpt(rlg);
	rlg_connect_neigh(route, rlg, B_FALSE, B_FALSE);
}

/*
 * Creates a new route. The route will derive its navigation data from `navdb'.
 */
route_t *
route_create(const fms_navdb_t *navdb)
{
	route_t *route = calloc(sizeof (*route), 1);

	ASSERT(navdb != NULL);
	route->navdb = navdb;
	list_create(&route->leg_groups, sizeof (route_leg_group_t),
	    offsetof(route_leg_group_t, route_leg_groups_node));
	list_create(&route->legs, sizeof (route_leg_t),
	    offsetof(route_leg_t, route_legs_node));
	list_create(&route->segs, sizeof (route_seg_t),
	    offsetof(route_seg_t, route_segs_node));

	return (route);
}

/*
 * Destroys and frees all resources used by `route'. Pointers to any resources
 * internal to the route should be considered invalid after this.
 */
void
route_destroy(route_t *route)
{
	if (route->dep)
		airport_close(route->dep);
	if (route->arr)
		airport_close(route->arr);
	if (route->altn1)
		airport_close(route->altn1);
	if (route->altn2)
		airport_close(route->altn2);

	for (route_leg_group_t *rlg = list_head(&route->leg_groups); rlg;
	    rlg = list_head(&route->leg_groups))
		rlg_destroy(route, rlg);
	/*
	 * Should be empty now because a rlg_destroy takes the
	 * legs with it.
	 */
	ASSERT(list_head(&route->legs) == NULL);

	for (route_seg_t *rs = list_head(&route->segs); rs != NULL;
	    rs = list_head(&route->segs))
		rs_destroy(rs);

	list_destroy(&route->leg_groups);
	list_destroy(&route->legs);
	list_destroy(&route->segs);

	free(route);
}

/*void
route_update(route_t *route, acft_perf_t *acft, flt_perf_t *flt)
{
	geo_pos3_t	cur_pos;
	vect2_t		cur_flt_dir;
	double		cur_spd;

	cur_pos = route_first_start_pos(route, NULL, NULL);
}*/

/*
 * Returns B_TRUE if the route needs to be route_update'd, B_FALSE if not.
 */
bool_t
route_update_needed(const route_t *route)
{
	return (route->segs_dirty);
}

/*
 * Generic airport setter for a route. Takes a route, a pointer to an airport
 * POINTER (i.e. the field in the route_t which is the airport pointer which
 * should be set) and an airport icao code to set in the airport pointer.
 * This function closes the old airport (removing any outstanding references
 * to it from the route) and replaces it with a newly opened one. If you pass
 * NULL in icao, it just closes the airport and 
 */
static err_t
route_set_arpt(route_t *route, airport_t **arptp, const char *icao)
{
	airport_t *narpt = NULL;

	if (icao != NULL) {
		/* Don't replace the same airport - just return OK */
		if (*arptp != NULL && strcmp((*arptp)->icao, icao) == 0)
			return (ERR_OK);

		/* Try to open new airport */
		narpt = airport_open(icao, route->navdb->navdata_dir,
		    route->navdb->wptdb, route->navdb->navaiddb);
		if (narpt == NULL)
			return (ERR_ARPT_NOT_FOUND);
	}

	/* Replace the old one */
	if (*arptp != NULL) {
		route_remove_arpt_links(route, *arptp);
		airport_close(*arptp);
	}
	if (*arptp != narpt) {
		*arptp = narpt;
		route->segs_dirty = B_TRUE;
	}

	return (ERR_OK);
}

/*
 * Sets the departure airport of the route. See route_set_arpt for more info.
 */
err_t
route_set_dep_arpt(route_t *route, const char *icao)
{
	return (route_set_arpt(route, &route->dep, icao));
}

/*
 * Sets the arrival airport of the route. See route_set_arpt for more info.
 */
err_t
route_set_arr_arpt(route_t *route, const char *icao)
{
	return (route_set_arpt(route, &route->arr, icao));
}

/*
 * Sets the 1st alternate airport of the route.
 */
err_t
route_set_altn1_arpt(route_t *route, const char *icao)
{
	return (route_set_arpt(route, &route->altn1, icao));
}

/*
 * Sets the 2nd alternate airport of the route.
 */
err_t
route_set_altn2_arpt(route_t *route, const char *icao)
{
	return (route_set_arpt(route, &route->altn2, icao));
}

/*
 * Returns the departure airport for the route (or NULL if not set).
 */
const airport_t *
route_get_dep_arpt(route_t *route)
{
	return (route->dep);
}

/*
 * Returns the arrival airport for the route (or NULL if not set).
 */
const airport_t *
route_get_arr_arpt(route_t *route)
{
	return (route->arr);
}

/*
 * Returns the 1st alternate airport for the route (or NULL if not set).
 */
const airport_t *
route_get_altn1_arpt(route_t *route)
{
	return (route->altn1);
}

/*
 * Returns the 2nd alternate airport for the route (or NULL if not set).
 */
const airport_t *
route_get_altn2_arpt(route_t *route)
{
	return (route->altn2);
}

/*
 * Sets the departure runway of the route. Before setting this you must
 * set the route's departure airport. Please note that setting the departure
 * runway also deletes any previously defined departure procedures.
 */
err_t
route_set_dep_rwy(route_t *route, const char *rwy_ID)
{
	char rwy_ID_long[RWY_ID_LEN + 1];
	unsigned rwy_val;
	runway_t *rwy = NULL;

	if (route->dep == NULL)
		return (ERR_ARPT_NOT_FOUND);

	if (rwy_ID == NULL) {
		route_set_sid(route, NULL);
		route->dep_rwy = NULL;
		return (ERR_OK);
	}

	rwy_val = atoi(rwy_ID);
	if (strlen(rwy_ID) > RWY_ID_LEN)
		return (ERR_INVALID_RWY);
	if (rwy_val < 10 && rwy_ID[0] != '0') {
		/* Prepend a '0' to make it a standard rwy_ID */
		(void) strlcpy(&rwy_ID_long[1], rwy_ID,
		    sizeof (rwy_ID_long) - 1);
		rwy_ID_long[0] = '0';
	} else {
		(void) strlcpy(rwy_ID_long, rwy_ID, sizeof (rwy_ID_long));
	}
	if (!is_valid_rwy_ID(rwy_ID_long))
		return (ERR_INVALID_RWY);

	for (unsigned i = 0; i < route->dep->num_rwys; i++) {
		if (strcmp(route->dep->rwys[i].ID, rwy_ID_long) == 0) {
			rwy = &route->dep->rwys[i];
			break;
		}
	}
	if (rwy == NULL)
		return (ERR_INVALID_RWY);
	if (route->dep_rwy != rwy) {
		route_set_sid(route, NULL);
		route->dep_rwy = rwy;
	}

	return (ERR_OK);
}

/*
 * Looks through the route leg groups and locates the first procedure route
 * leg group with a navproc of type `type' and returns it. If no such rlg
 * exists, returns NULL instead.
 */
static route_leg_group_t *
route_find_proc_rlg(route_t *route, navproc_type_t type)
{
	for (route_leg_group_t *rlg = list_head(&route->leg_groups); rlg;
	    rlg = rlg_next_ndisc(route, rlg)) {
		if (rlg->type == ROUTE_LEG_GROUP_TYPE_PROC &&
		    rlg->proc->type == type) {
			/* SIDs must be first in the leg group sequence */
			ASSERT(type != NAVPROC_TYPE_SID ||
			    list_head(&route->leg_groups) == rlg);
			/* FINALs must be last in the leg group sequence */
			ASSERT(type != NAVPROC_TYPE_FINAL ||
			    list_tail(&route->leg_groups) == rlg);
			return (rlg);
		}
	}
	return (NULL);
}

/*
 * Given a navproc procedure type, will locate the route leg group that
 * links to it and removes it. If the rlg doesn't exist, this function
 * does nothing.
 */
static void
route_delete_proc_rlg(route_t *route, navproc_type_t type)
{
	route_leg_group_t *rlg = route_find_proc_rlg(route, type);

	if (rlg != NULL)
		rlg_bypass(route, rlg, B_FALSE, B_FALSE);
}

/*
 * Given a navproc, constructs the corresponding route leg group and its
 * legs and inserts the rlg after `prev_rlg' (or the start of the route
 * if `prev_rlg' is NULL). This also inserts all of the rlg's legs. The
 * function doesn't attempt connecting the rlg to its neighbors.
 */
static route_leg_group_t *
route_insert_proc_rlg(route_t *route, const navproc_t *proc,
    route_leg_group_t *prev_rlg)
{
	route_leg_group_t *rlg = rlg_new(ROUTE_LEG_GROUP_TYPE_PROC, route);
	route_leg_t *prev_rl;

	rlg->proc = proc;
	list_insert_after(&route->leg_groups, prev_rlg, rlg);
	prev_rl = last_leg_before_rlg(route, rlg);

	/* Start inserting individual procedure legs */
	for (unsigned i = 0; i < proc->num_segs; i++) {
		route_leg_t *rl = calloc(sizeof (*rl), 1);
		rl->seg = proc->segs[i];
		rl->rlg = rlg;
		list_insert_tail(&rlg->legs, rl);
		list_insert_after(&route->legs, prev_rl, rl);
		prev_rl = rl;
	}
	ASSERT(!list_is_empty(&rlg->legs));
	rlg->start_wpt = navproc_get_start_wpt(proc);
	rlg->end_wpt = rlg_find_end_wpt(rlg);

	return (rlg);
}

/*
 * Locates a navproc matching in name and parameters.
 *
 * @param arpt The airport where to look for the navproc.
 * @param type Navproc type to look for.
 * @param name Name of the navproc. Mandatory.
 * @param tr_or_rwy Optional selection criterion. This depends on the type:
 *	NAVPROC_TYPE_SID_COMMON | NAVPROC_TYPE_STAR_COMMON | NAVPROC_TYPE_FINAL
 *		For these procedures, this argument is ignored, procedures
 *		are only selected based on name.
 *	NAVPROC_TYPE_STAR
 *		For STARs, this argument is optional. If provided, it denotes
 *		runway discriminator on the procedure. If omitted, the first
 *		STAR matching the name is returned.
 *	NAVPROC_TYPE_SID | NAVPROC_TYPE_STAR_TRANS | NAVPROC_TYPE_FINAL_TRANS
 *		For these procedures, this argument is mandatory. For the
 *		*SID procedure, it defines departing runway discriminator. For
 *		*STAR_TRANS and *FINAL_TRANS is defines the transition name.
 */
static const navproc_t *
find_navproc(const airport_t *arpt, navproc_type_t type, const char *name,
    const char *tr_or_rwy)
{
	ASSERT(tr_or_rwy != NULL || type == NAVPROC_TYPE_SID_COMMON ||
	    type == NAVPROC_TYPE_STAR_COMMON || type == NAVPROC_TYPE_STAR ||
	    type == NAVPROC_TYPE_FINAL);
	for (unsigned i = 0; i < arpt->num_procs; i++) {
		if (arpt->procs[i].type != type ||
		    strcmp(arpt->procs[i].name, name) != 0)
			continue;
		if ((type == NAVPROC_TYPE_SID_TRANS ||
		    type == NAVPROC_TYPE_STAR_TRANS ||
		    type == NAVPROC_TYPE_FINAL_TRANS) &&
		    strcmp(tr_or_rwy, arpt->procs[i].tr_name) != 0) {
			continue;
		} else if ((type == NAVPROC_TYPE_SID ||
		    type == NAVPROC_TYPE_STAR) &&
		    tr_or_rwy != NULL &&
		    strcmp(tr_or_rwy, arpt->procs[i].rwy->ID) != 0) {
			continue;
		}
		return (&arpt->procs[i]);
	}
	return (NULL);
}

/*
 * Sets the standard departure procedure of the route and inserts the
 * appropriate legs. If the route had a departure transition procedure
 * set, it will be deleted. Before calling this function you must set
 * the departure airport and departure runway.
 */
err_t
route_set_sid(route_t *route, const char *sid_name)
{
	const navproc_t		*sid = NULL, *sidcm = NULL;
	route_leg_group_t	*sid_rlg = NULL, *sidcm_rlg = NULL;
	airport_t		*dep = route->dep;
	const runway_t		*dep_rwy = route->dep_rwy;

	if (dep_rwy == NULL)
		return (ERR_INVALID_ENTRY);
	ASSERT(dep != NULL);

	if (sid_name == NULL) {
		route_delete_proc_rlg(route, NAVPROC_TYPE_SID);
		route_delete_proc_rlg(route, NAVPROC_TYPE_SID_COMMON);
		route_delete_proc_rlg(route, NAVPROC_TYPE_SID_TRANS);
		route->sidtr = NULL;
		route->sidcm = NULL;
		route->sid = NULL;
		return (ERR_OK);
	}

	/* We need at least one */
	sid = find_navproc(dep, NAVPROC_TYPE_SID, sid_name, dep_rwy->ID);
	sidcm = find_navproc(dep, NAVPROC_TYPE_SID_COMMON, sid_name, NULL);
	if (sid == NULL && sidcm == NULL)
		return (ERR_INVALID_SID);

	route_delete_proc_rlg(route, NAVPROC_TYPE_SID);
	route_delete_proc_rlg(route, NAVPROC_TYPE_SID_COMMON);
	route_delete_proc_rlg(route, NAVPROC_TYPE_SID_TRANS);

	if (sid != NULL) {
		sid_rlg = route_insert_proc_rlg(route, sid, NULL);
		rlg_connect_neigh(route, sid_rlg, B_TRUE, B_FALSE);
	}
	if (sidcm != NULL) {
		sidcm_rlg = route_insert_proc_rlg(route, sidcm, sid_rlg);
		rlg_connect_neigh(route, sidcm_rlg, B_TRUE, B_FALSE);
	}

	route->sid = sid;
	route->sidcm = sidcm;
	route->sidtr = NULL;

	return (ERR_OK);
}

/*
 * Sets the standard departure transition procedure of the route and inserts
 * the appropriate legs. If the route has no standard departure procedure
 * yet set, this function will return with an INVALID INSERT error.
 */
err_t
route_set_sidtr(route_t *route, const char *tr_name)
{
	const navproc_t		*sidtr = NULL;
	route_leg_group_t	*sidtr_rlg, *sid_rlg = NULL;
	airport_t		*dep = route->dep;
	const char		*sid_name;

	if (route->sid == NULL && route->sidcm == NULL)
		return (ERR_INVALID_ENTRY);
	ASSERT(route->dep != NULL);
	ASSERT(route->dep_rwy != NULL);

	if (tr_name == NULL) {
		route_delete_proc_rlg(route, NAVPROC_TYPE_SID_TRANS);
		route->sidtr = NULL;
		return (ERR_OK);
	}
	sid_name = (route->sid ? route->sid->name : route->sidcm->name);

	/* First try SID_COMMON then SID */
	sid_rlg = route_find_proc_rlg(route, NAVPROC_TYPE_SID_COMMON);
	if (sid_rlg == NULL)
		sid_rlg = route_find_proc_rlg(route, NAVPROC_TYPE_SID);

	sidtr = find_navproc(dep, NAVPROC_TYPE_SID_TRANS, sid_name, tr_name);
	if (sidtr == NULL)
		return (ERR_INVALID_TRANS);

	route_delete_proc_rlg(route, NAVPROC_TYPE_SID_TRANS);

	sidtr_rlg = route_insert_proc_rlg(route, sidtr, sid_rlg);
	rlg_connect_neigh(route, sidtr_rlg, B_TRUE, B_FALSE);

	route->sidtr = sidtr;

	return (ERR_OK);
}

/*
 * Sets the standard arrival procedure of the route and inserts the
 * appropriate legs. If the route had an arrival transition procedure
 * set, it will be deleted.
 */
err_t
route_set_star(route_t *route, const char *star_name)
{
	const navproc_t		*star = NULL, *starcm = NULL;
	route_leg_group_t	*star_rlg, *starcm_rlg, *appr_rlg;
	airport_t		*arr = route->arr;

	if (arr == NULL)
		return (ERR_ARPT_NOT_FOUND);

	if (star_name == NULL) {
		route_delete_proc_rlg(route, NAVPROC_TYPE_STAR_TRANS);
		route_delete_proc_rlg(route, NAVPROC_TYPE_STAR_COMMON);
		route_delete_proc_rlg(route, NAVPROC_TYPE_STAR);
		route->startr = NULL;
		route->starcm = NULL;
		route->star = NULL;
		return (ERR_OK);
	}

	starcm = find_navproc(arr, NAVPROC_TYPE_STAR_COMMON, star_name, NULL);
	star = find_navproc(arr, NAVPROC_TYPE_STAR, star_name,
	    route->appr != NULL ? route->appr->rwy->ID : NULL);
	/* We need at least one */
	if (starcm == NULL && star == NULL)
		return (ERR_INVALID_STAR);

	route_delete_proc_rlg(route, NAVPROC_TYPE_STAR_TRANS);
	route_delete_proc_rlg(route, NAVPROC_TYPE_STAR_COMMON);
	route_delete_proc_rlg(route, NAVPROC_TYPE_STAR);

	/* Must go after route_delete_proc_rlg to avoid stale rlg refs */
	appr_rlg = route_find_proc_rlg(route, NAVPROC_TYPE_FINAL_TRANS);
	if (appr_rlg == NULL)
		appr_rlg = route_find_proc_rlg(route, NAVPROC_TYPE_FINAL);

	if (starcm != NULL) {
		route_leg_group_t *prev_rlg = (appr_rlg != NULL ?
		    rlg_prev_ndisc(route, appr_rlg) : rlg_tail_ndisc(route));

		starcm_rlg = route_insert_proc_rlg(route, starcm, prev_rlg);
		rlg_connect_neigh(route, starcm_rlg, B_TRUE, B_FALSE);
	}
	if (star != NULL) {
		route_leg_group_t *prev_rlg = (appr_rlg != NULL ?
		    rlg_prev_ndisc(route, appr_rlg) : rlg_tail_ndisc(route));

		star_rlg = route_insert_proc_rlg(route, star, prev_rlg);
		rlg_connect_neigh(route, star_rlg, B_TRUE, B_FALSE);
	}

	route->star = star;
	route->starcm = starcm;
	route->startr = NULL;

	return (ERR_OK);
}

/*
 * Sets the standard arrival transition procedure of the route and inserts the
 * appropriate legs. If the route has no standard arrival procedure yet set,
 * this function will return with an INVALID INSERT error.
 */
err_t
route_set_startr(route_t *route, const char *tr_name)
{
	const navproc_t		*startr = NULL;
	route_leg_group_t	*startr_rlg, *next_rlg, *prev_rlg;
	airport_t		*arr = route->arr;
	const char		*star_name;

	if (route->star == NULL && route->starcm == NULL)
		return (ERR_INVALID_ENTRY);
	ASSERT(route->arr != NULL);

	if (tr_name == NULL) {
		route_delete_proc_rlg(route, NAVPROC_TYPE_STAR_TRANS);
		route->startr = NULL;
		return (ERR_OK);
	}
	star_name = (route->star ? route->star->name : route->starcm->name);

	/* Try looking for the rlg's of the STARCM/STAR/FINALTR/FINAL */
	next_rlg = route_find_proc_rlg(route, NAVPROC_TYPE_STAR_COMMON);
	if (next_rlg == NULL)
		next_rlg = route_find_proc_rlg(route, NAVPROC_TYPE_STAR);
	if (next_rlg == NULL)
		next_rlg = route_find_proc_rlg(route, NAVPROC_TYPE_FINAL_TRANS);
	if (next_rlg == NULL)
		next_rlg = route_find_proc_rlg(route, NAVPROC_TYPE_FINAL);

	startr = find_navproc(arr, NAVPROC_TYPE_STAR_TRANS, star_name, tr_name);
	if (startr == NULL)
		return (ERR_INVALID_TRANS);

	route_delete_proc_rlg(route, NAVPROC_TYPE_STAR_TRANS);

	/* Must go after route_delete_proc_rlg to avoid stale rlg refs */
	prev_rlg = (next_rlg ? rlg_prev_ndisc(route, next_rlg) :
	    rlg_tail_ndisc(route));
	startr_rlg = route_insert_proc_rlg(route, startr, prev_rlg);
	rlg_connect_neigh(route, startr_rlg, B_TRUE, B_FALSE);

	route->startr = startr;

	return (ERR_OK);
}

/*
 * Sets the final approach procedure of the route and inserts the appropriate
 * legs. If the route had an approach transition procedure set, it will be
 * deleted.
 */
err_t
route_set_appr(route_t *route, const char *appr_name)
{
	const navproc_t		*appr = NULL;
	route_leg_group_t	*appr_rlg, *prev_rlg;
	airport_t		*arr = route->arr;
	const char		*star_name;

	if (arr == NULL)
		return (ERR_ARPT_NOT_FOUND);

	if (appr_name == NULL) {
		route_delete_proc_rlg(route, NAVPROC_TYPE_FINAL_TRANS);
		route_delete_proc_rlg(route, NAVPROC_TYPE_FINAL);
		route->apprtr = NULL;
		route->appr = NULL;
		return (ERR_OK);
	}

	appr = find_navproc(arr, NAVPROC_TYPE_FINAL, appr_name, NULL);
	if (appr == NULL)
		return (ERR_INVALID_FINAL);

	route_delete_proc_rlg(route, NAVPROC_TYPE_FINAL_TRANS);
	route_delete_proc_rlg(route, NAVPROC_TYPE_FINAL);

	/* Must go after route_delete_proc_rlg to avoid stale rlg refs */
	prev_rlg = rlg_tail_ndisc(route);

	appr_rlg = route_insert_proc_rlg(route, appr, prev_rlg);
	rlg_connect_neigh(route, appr_rlg, B_TRUE, B_FALSE);

	route->appr = appr;
	route->apprtr = NULL;

	/* The STAR may need a refresh, because it depends on appr */
	if (route->star)
		star_name = route->star->name;
	else if (route->starcm)
		star_name = route->starcm->name;
	else
		star_name = NULL;
	/* If the STAR is known, try to reset it */
	if (star_name != NULL) {
		/* Resetting the STAR clears out its transition */
		const char *tr_name =
		    (route->startr ? route->startr->tr_name : NULL);

		if (route_set_star(route, star_name) != ERR_OK) {
			/*
			 * If refreshing the STAR failed, clear it out, it
			 * is not applicable to this approach.
			 */
			route_set_star(route, NULL);
		} else if (tr_name != NULL) {
			/* STAR reset succeeded, try resetting its TRANS */
			route_set_startr(route, tr_name);
		}
	}

	return (ERR_OK);
}

/*
 * Sets the final approach transition procedure of the route and inserts the
 * appropriate legs. If the route has no final approach procedure yet set,
 * this function will return with an INVALID INSERT error.
 */
err_t
route_set_apprtr(route_t *route, const char *tr_name)
{
	const navproc_t		*apprtr = NULL;
	route_leg_group_t	*apprtr_rlg, *next_rlg, *prev_rlg;
	airport_t		*arr = route->arr;
	const char		*appr_name;

	if (route->appr == NULL)
		return (ERR_INVALID_ENTRY);
	ASSERT(route->arr != NULL);

	if (tr_name == NULL) {
		route_delete_proc_rlg(route, NAVPROC_TYPE_FINAL_TRANS);
		route->apprtr = NULL;
		return (ERR_OK);
	}
	appr_name = route->appr->name;

	apprtr = find_navproc(arr, NAVPROC_TYPE_FINAL_TRANS, appr_name,
	    tr_name);
	if (apprtr == NULL)
		return (ERR_INVALID_TRANS);

	route_delete_proc_rlg(route, NAVPROC_TYPE_FINAL_TRANS);

	/* Must go after route_delete_proc_rlg to avoid stale rlg refs */
	next_rlg = route_find_proc_rlg(route, NAVPROC_TYPE_FINAL);
	prev_rlg = (next_rlg ? rlg_prev_ndisc(route, next_rlg) :
	    rlg_tail_ndisc(route));

	apprtr_rlg = route_insert_proc_rlg(route, apprtr, prev_rlg);
	rlg_connect_neigh(route, apprtr_rlg, B_TRUE, B_FALSE);

	route->apprtr = apprtr;

	return (ERR_OK);
}

/*
 * Returns the route's departure runway or NULL if none is set.
 */
const runway_t *
route_get_dep_rwy(const route_t *route)
{
	return (route->dep_rwy);
}

/*
 * Returns the route's standard departure procedure or NULL if none is set.
 */
const navproc_t *
route_get_sid(const route_t *route)
{
	return (route->sid ? route->sid : route->sidcm);
}

/*
 * Returns the route's standard departure transition or NULL if none is set.
 */
const navproc_t *
route_get_sidtr(const route_t *route)
{
	return (route->sidtr);
}

/*
 * Returns the route's standard arrival procedure or NULL if none is set.
 */
const navproc_t *
route_get_star(const route_t *route)
{
	return (route->star ? route->star : route->starcm);
}

/*
 * Returns the route's standard arrival transition or NULL if none is set.
 */
const navproc_t *
route_get_startr(const route_t *route)
{
	return (route->startr);
}

/*
 * Returns the route's final approach procedure or NULL if none is set.
 */
const navproc_t *
route_get_appr(const route_t *route)
{
	return (route->appr);
}

/*
 * Returns the route's final approach transition or NULL if none is set.
 */
const navproc_t *
route_get_apprtr(const route_t *route)
{
	return (route->apprtr);
}

/*
 * Returns the leg groups which constitute the route. This is a read-only
 * list. To edit the route, use the route editing functions.
 */
const list_t *
route_get_leg_groups(const route_t *route)
{
	return (&route->leg_groups);
}

/*
 * Returns the individual legs which constitute the route. This is a
 * read-only list. To edit the route, use the route editing functions.
 */
const list_t *
route_get_legs(const route_t *route)
{
	return (&route->legs);
}

/*
 * Inserts an airway leg group without a terminating wpt for the moment.
 *
 * @param route The route to insert the leg group into.
 * @param awyname Airway name.
 * @param x_prev_rlg The preceding leg group after which to insert the new
 *	leg group. Can be NULL to insert at the start of the route.
 * @param new_rlgpp If not NULL, the pointer will be set to point to the
 *	newly created leg group.
 *
 * @return ERR_OK on success or an error code otherwise.
 */
err_t
route_lg_awy_insert(route_t *route, const char *awyname,
    const route_leg_group_t *x_prev_rlg, const route_leg_group_t **new_rlgpp)
{
	route_leg_group_t *prev_rlg = (route_leg_group_t *)x_prev_rlg;
	route_leg_group_t *next_rlg = rlg_next_ndisc(route, prev_rlg);
	route_leg_group_t *rlg;
	const airway_t *awy;

	awy = airway_db_lookup(route->navdb->awydb, awyname, NULL, NULL, NULL);
	if (!awy)
		return (ERR_INVALID_AWY);
	if (next_rlg != NULL && next_rlg->type == ROUTE_LEG_GROUP_TYPE_PROC &&
	    next_rlg->proc->type <= NAVPROC_TYPE_SID_TRANS)
		return (ERR_INVALID_ENTRY);
	if (prev_rlg != NULL && prev_rlg->type == ROUTE_LEG_GROUP_TYPE_PROC &&
	    prev_rlg->proc->type >= NAVPROC_TYPE_STAR)
		return (ERR_INVALID_ENTRY);

	rlg = rlg_new(ROUTE_LEG_GROUP_TYPE_AIRWAY, route);
	rlg->awy = awy;

	list_insert_after(&route->leg_groups, prev_rlg, rlg);

	rlg_update_awy_legs(route, rlg, B_FALSE);
	rlg_connect_neigh(route, rlg, B_TRUE, B_TRUE);

	route->segs_dirty = B_TRUE;
	if (new_rlgpp)
		*new_rlgpp = rlg;

	return (ERR_OK);
}

/*
 * Sets the terminating wpt of an airway leg group previously created with
 * route_lg_awy_insert.
 *
 * @param route The route holding the airway leg group.
 * @param x_rlg The airway leg group for which to set the terminating wpt.
 * @param wptname The name of the terminating wpt.
 *
 * @return ERR_OK on success or an error code otherwise.
 */
err_t
route_lg_awy_set_end_fix(route_t *route, const route_leg_group_t *x_rlg,
    const char *wptname)
{
	route_leg_group_t *rlg = (route_leg_group_t *)x_rlg;
	const airway_t *newawy;
	const wpt_t *end_wpt;

	ASSERT(rlg->type == ROUTE_LEG_GROUP_TYPE_AIRWAY);
	ASSERT(list_link_active(&rlg->route_leg_groups_node));

	if (IS_NULL_WPT(&rlg->start_wpt))
		return (ERR_AWY_WPT_MISMATCH);

	newawy = airway_db_lookup(route->navdb->awydb, rlg->awy->name,
	    &rlg->start_wpt, wptname, &end_wpt);
	if (newawy == NULL)
		return (ERR_AWY_WPT_MISMATCH);

	rlg->awy = newawy;
	rlg->end_wpt = *end_wpt;

	rlg_connect_neigh(route, rlg, B_FALSE, B_FALSE);

	rlg_update_awy_legs(route, rlg, B_FALSE);
	route->segs_dirty = B_TRUE;

	return (ERR_OK);
}

/*
 * Inserts a direct-to-FIX leg group to a route.
 *
 * @param route The route to insert the leg group into.
 * @param fix The target fix for the direct leg group.
 * @param x_prev_rlg The preceding leg group after which to insert the new
 *	leg group. Can be NULL to insert at the start of the route.
 * @param new_rlgpp If not NULL, the pointer will be set to point to the
 *	newly created leg group.
 *
 * @return ERR_OK on success or an error code otherwise.
 */
err_t
route_lg_direct_insert(route_t *route, const wpt_t *fix,
    const route_leg_group_t *x_prev_rlg, const route_leg_group_t **new_rlgpp)
{
	route_leg_group_t *prev_rlg = (route_leg_group_t *)x_prev_rlg;
	route_leg_group_t *next_rlg = rlg_next_ndisc(route, prev_rlg);
	route_leg_group_t *rlg;

	if (next_rlg != NULL && next_rlg->type == ROUTE_LEG_GROUP_TYPE_PROC &&
	    next_rlg->proc->type <= NAVPROC_TYPE_SID_TRANS)
		return (ERR_INVALID_ENTRY);
	if (prev_rlg != NULL && prev_rlg->type == ROUTE_LEG_GROUP_TYPE_PROC &&
	    prev_rlg->proc->type >= NAVPROC_TYPE_STAR)
		return (ERR_INVALID_ENTRY);

	rlg = rlg_new(ROUTE_LEG_GROUP_TYPE_DIRECT, route);
	rlg->end_wpt = *fix;

	list_insert_after(&route->leg_groups, prev_rlg, rlg);
	rlg_update_direct_leg(route, rlg);
	rlg_connect_neigh(route, rlg, B_TRUE, B_FALSE);

	route->segs_dirty = B_TRUE;
	if (new_rlgpp)
		*new_rlgpp = rlg;

	return (ERR_OK);
}

/*
 * Deletes a route leg group and attempts to reconnect the leg group
 * components to either side of it, automatically managing addition/removal
 * of discontinuities.
 */
err_t
route_lg_delete(route_t *route, const route_leg_group_t *x_rlg)
{
	route_leg_group_t *rlg = (route_leg_group_t *)x_rlg;
	route_leg_group_t *prev_rlg = rlg_prev_ndisc(route, rlg);
	route_leg_group_t *next_rlg = rlg_next_ndisc(route, rlg);
	bool_t allow_mod = (rlg->type == ROUTE_LEG_GROUP_TYPE_DISCO);

	/*
	 * Procedures can't be deleted this way, use the appropriate
	 * procedure manipulation functions.
	 */
	if (rlg->type == ROUTE_LEG_GROUP_TYPE_PROC)
		return (ERR_INVALID_DELETE);

	rlg_destroy(route, rlg);
	rlg_connect(route, prev_rlg, next_rlg, allow_mod, B_FALSE);

	return (ERR_OK);
}

/*
 * Prepends a leg ending at `wpt' to an airway leg group `awyrlg'. The new
 * leg will be returned in `rlpp' (if not NULL). What this does is set the
 * start fix of the airway leg group to `wpt' and regenerate its legs. It
 * then generates a new direct leg group and inserts it before `awyrlg'.
 * Finally, it connects the direct rlg to the airway and also to any rlg
 * that might have preceded it.
 */
static void
rlg_prepend_direct(route_t *route, route_leg_group_t *awyrlg, const wpt_t *wpt,
    const route_leg_t **rlpp)
{
	const route_leg_group_t *dirrlg;
	route_leg_group_t *prev_rlg = rlg_prev_ndisc(route, awyrlg);

	ASSERT(awyrlg->type == ROUTE_LEG_GROUP_TYPE_AIRWAY);
	awyrlg->start_wpt = *wpt;
	rlg_update_awy_legs(route, awyrlg, B_TRUE);
	route_lg_direct_insert(route, wpt, prev_rlg, &dirrlg);
	if (rlpp)
		*rlpp = list_head(&dirrlg->legs);
}

/*
 * Appends a leg ending at `wpt' to the airway `rlg' and returns the new
 * leg in `rlpp' (if not NULL). This simply sets the airway's end wpt to
 * `wpt' and regenerates its legs, at the end attempting to connect it to
 * the rlg following it.
 */
static void
rlg_append_direct(route_t *route, route_leg_group_t *rlg, const wpt_t *wpt,
    const route_leg_t **rlpp)
{
	ASSERT(rlg->type == ROUTE_LEG_GROUP_TYPE_AIRWAY ||
	    rlg->type == ROUTE_LEG_GROUP_TYPE_PROC);
	rlg->end_wpt = *wpt;
	if (rlg->type == ROUTE_LEG_GROUP_TYPE_AIRWAY) {
		rlg_update_awy_legs(route, rlg, B_TRUE);
	} else {
		route_leg_t *rl = rl_new_direct(wpt, rlg);
		route_leg_t *rl_last = list_tail(&rlg->legs);
		list_insert_tail(&rlg->legs, rl);
		list_insert_after(&route->legs, rl_last, rl);
	}
	rlg_connect_neigh(route, rlg, B_FALSE, B_FALSE);
	if (rlpp)
		*rlpp = list_tail(&rlg->legs);
}

/*
 * Returns true if `type' describes a SID* procedure.
 */
static bool_t
is_departure_procedure(navproc_type_t type)
{
	return (type >= NAVPROC_TYPE_SID && type <= NAVPROC_TYPE_SID_TRANS);
}

/*
 * Returns true if `type' describes a STAR* or an APPR* procedure.
 */
static bool_t
is_terminal_procedure(navproc_type_t type)
{
	return (type >= NAVPROC_TYPE_STAR && type <= NAVPROC_TYPE_FINAL);
}

/*
 * Given two legs `rl1' and `rl2' on the same airway route leg group, this
 * function splits the airway rlg up into two airway rlg's. The first airway
 * is terminated at rl1's end wpt, and the second airway starts at the start
 * wpt of rl2. rl2 must follow rl1 on the airway leg group. If `join' is set
 * to B_TRUE and rl1 and rl2 are not immediately adjacent on the airway, the
 * two new airways are connected using a DIRECT leg group. If `join' is set
 * to B_FALSE and rl1 and rl2 are not immediately adjacent on the airway a
 * DISCO is inserted between them. The `join' argument is ignored if `rl1'
 * and `rl2' are adjacent.
 */
static void
awy_split(route_t *route, route_leg_group_t *awy1, route_leg_t *rl1,
    route_leg_t *rl2, bool_t join)
{
	route_leg_group_t	*dir, *awy2;
	wpt_t			awy1_start_fix, awy1_end_fix;
	wpt_t			awy2_end_fix, awy2_start_fix;

	awy1_start_fix = awy1->start_wpt;
	if (rl1 != NULL) {
		awy1_end_fix = *leg_get_end_wpt(rl1);
	} else {
		awy1_end_fix = awy1->start_wpt;
	}
	if (rl2 != NULL) {
		route_leg_t *prev_rl = list_prev(&awy1->legs, rl2);
		awy2_start_fix = *leg_get_end_wpt(prev_rl);
	} else {
		awy2_start_fix = awy1->end_wpt;
	}
	awy2_end_fix = awy1->end_wpt;

	if (!WPT_EQ(&awy2_start_fix, &awy2_end_fix)) {
		awy2 = rlg_new(ROUTE_LEG_GROUP_TYPE_AIRWAY, route);
		awy2->awy = awy1->awy;
		awy2->start_wpt = awy2_start_fix;
		awy2->end_wpt = awy2_end_fix;
		list_insert_after(&route->leg_groups, awy1, awy2);
		rlg_update_awy_legs(route, awy2, B_FALSE);
	} else {
		awy2 = NULL;
	}

	if (!WPT_EQ(&awy1_end_fix, &awy2_start_fix)) {
		if (join) {
			dir = rlg_new(ROUTE_LEG_GROUP_TYPE_DIRECT, route);
			dir->start_wpt = awy1_end_fix;
			dir->end_wpt = awy2_start_fix;
			list_insert_after(&route->leg_groups, awy1, dir);
			rlg_update_direct_leg(route, dir);
		} else {
			dir = NULL;
		}
	} else {
		dir = NULL;
	}

	if (!WPT_EQ(&awy1_start_fix, &awy1_end_fix)) {
		awy1->end_wpt = awy1_end_fix;
		rlg_update_awy_legs(route, awy1, B_FALSE);
	} else {
		rlg_bypass(route, awy1, B_TRUE, B_FALSE);
		awy1 = NULL;
	}

	if (awy1 != NULL)
		rlg_connect_neigh(route, awy1, B_TRUE, B_FALSE);
	if (dir != NULL)
		rlg_connect_neigh(route, dir, B_TRUE, B_FALSE);
	if (awy2 != NULL)
		rlg_connect_neigh(route, awy2, B_TRUE, B_FALSE);

	route->segs_dirty = B_TRUE;
}

static bool_t
leg_check_dup(const route_leg_t *rl, const wpt_t *wpt)
{
	return (rl != NULL && rl->seg.type != NAVPROC_SEG_TYPE_INIT_FIX &&
	    WPT_EQ_POS(leg_get_end_wpt(rl), wpt));
}

/*
 * Directly inserts a leg into a route (used on the LEGS page). The new leg
 * is targetted to terminate at `fix' and will be following `x_prev_rl'. If
 * `rlpp' is not NULL, the newly created leg will be returned there. The
 * rules for leg insertion are as follows:
 *
 * 1) If there are legs either side of the new leg:
 *	a) If each side-leg belongs to a different route leg groups:
 *		i) If the previous leg group is an airway and the newly
 *		   added leg immediately follows the airway's last wpt on
 *		   the same airway, then the airway is simply extended to
 *		   end at the new leg's wpt.
 *		ii) If the previous and next leg group are procedures that
 *		   belong to the same airport, then the new leg is appended
 *		   to the end of the previous leg group.
 *		iii) If the next leg group is an airway and the newly
 *		   added leg immediately precedes the airway's first wpt
 *		   on the same airway, then the airway is simply extended
 *		   to start at the new leg's wpt.
 *		iv) If all else fails then insert a new direct leg group
 *		   containing the new leg in between the extant leg groups.
 */
err_t
route_l_insert(route_t *route, const wpt_t *fix, const route_leg_t *x_prev_rl,
    const route_leg_t **rlpp)
{
	route_leg_t *prev_rl = (route_leg_t *)x_prev_rl;
	route_leg_t *next_rl = (prev_rl != NULL ?
	    list_next(&route->legs, prev_rl) : list_head(&route->legs));

	ASSERT(!IS_NULL_WPT(fix));

		/* Check for dups */
	if (leg_check_dup(prev_rl, fix) || leg_check_dup(next_rl, fix))
		return (ERR_DUPLICATE_LEG);

	if (prev_rl != NULL && next_rl != NULL) {
		/* Both legs exist */
		route_leg_group_t *prev_rlg = prev_rl->rlg;
		route_leg_group_t *next_rlg = next_rl->rlg;

		if (prev_rlg != next_rlg) {
			const route_leg_group_t *rlg;
			/*
			 * Parents not shared, figure out what to do based
			 * on rlg type.
			 */

			/* extend airway if the new fix immediately follows */
			if (prev_rlg->type == ROUTE_LEG_GROUP_TYPE_AIRWAY &&
			    chk_awy_fix_adjacent(prev_rlg, fix, B_FALSE)) {
				rlg_append_direct(route, prev_rlg, fix, rlpp);
				goto out;
			/*
			 * Extend a procedure if next_rlg is also a procedure
			 * from the same airport (since we don't want to
			 * split sequential procedures).
			 */
			} else if (prev_rlg->type == ROUTE_LEG_GROUP_TYPE_PROC
			    && next_rlg->type == ROUTE_LEG_GROUP_TYPE_PROC &&
			    prev_rlg->proc->arpt == next_rlg->proc->arpt) {
				rlg_append_direct(route, prev_rlg, fix, rlpp);
				goto out;
			}
			/*
			 * No success with modifying the prev_rlg, try
			 * the next_rlg.
			 */
			if (next_rlg->type == ROUTE_LEG_GROUP_TYPE_AIRWAY &&
			    chk_awy_fix_adjacent(next_rlg, fix, B_TRUE)) {
				rlg_prepend_direct(route, next_rlg, fix, rlpp);
				goto out;
			}
			/* last resort: new direct leg group */
			route_lg_direct_insert(route, fix, prev_rlg, &rlg);
			if (rlpp != NULL)
				*rlpp = list_head(&rlg->legs);
		} else {
			/* Same rlg */
			ASSERT(prev_rlg->type == ROUTE_LEG_GROUP_TYPE_AIRWAY ||
			    prev_rlg->type == ROUTE_LEG_GROUP_TYPE_PROC);
			if (prev_rlg->type == ROUTE_LEG_GROUP_TYPE_AIRWAY) {
				/* Airways need to be split */
				const route_leg_group_t *rlg;
				awy_split(route, prev_rl->rlg, prev_rl,
				    next_rl, B_FALSE);
				route_lg_direct_insert(route, fix, prev_rlg,
				    &rlg);
				if (rlpp != NULL)
					*rlpp = list_head(&rlg->legs);
			} else {
				/* Procedures must be internally expanded */
				route_leg_t *rl = rl_new_direct(fix, prev_rlg);
				list_insert_after(&route->legs, prev_rl, rl);
				list_insert_after(&prev_rlg->legs, prev_rl, rl);
				if (rlpp != NULL)
					*rlpp = rl;
			}
		}
	} else if (prev_rl != NULL) {
		route_leg_group_t *prev_rlg = prev_rl->rlg;
		const route_leg_group_t *rlg;

		/* extend airway if the new fix immediately follows */
		if (prev_rlg->type == ROUTE_LEG_GROUP_TYPE_AIRWAY &&
		    chk_awy_fix_adjacent(prev_rlg, fix, B_FALSE)) {
			rlg_append_direct(route, prev_rlg, fix, rlpp);
			goto out;
		/*
		 * Extend a procedure iff it's a terminal procedure, i.e.
		 * it needs to be last in the route leg group sequence.
		 */
		} else if (prev_rlg->type == ROUTE_LEG_GROUP_TYPE_PROC &&
		    is_terminal_procedure(prev_rlg->proc->type)) {
			rlg_append_direct(route, prev_rlg, fix, rlpp);
			goto out;
		}
		/* last resort: new direct leg group */
		route_lg_direct_insert(route, fix, prev_rlg, &rlg);
		if (rlpp != NULL)
			*rlpp = list_head(&rlg->legs);
	} else if (next_rl != NULL) {
		route_leg_group_t *next_rlg = next_rl->rlg;
		const route_leg_group_t *rlg;

		/* extend airway if the new fix immediately precedes it */
		if (next_rlg->type == ROUTE_LEG_GROUP_TYPE_AIRWAY &&
		    chk_awy_fix_adjacent(next_rlg, fix, B_TRUE)) {
			rlg_prepend_direct(route, next_rlg, fix, rlpp);
			goto out;
		/* Don't allow modifying the first departure procedure. */
		} else if (next_rlg->type == ROUTE_LEG_GROUP_TYPE_PROC &&
		    is_departure_procedure(next_rlg->proc->type)) {
			return (ERR_INVALID_ENTRY);
		}
		/* last resort: new direct leg group */
		route_lg_direct_insert(route, fix, NULL, &rlg);
		if (rlpp != NULL)
			*rlpp = list_head(&rlg->legs);
	} else {
		const route_leg_group_t *rlg;
		route_lg_direct_insert(route, fix, NULL, &rlg);
		if (rlpp != NULL)
			*rlpp = list_head(&rlg->legs);
	}
out:
	route->segs_dirty = B_TRUE;
	return (ERR_OK);
}

/*
 * Given a route leg `x_source_rl', moves it to replace `x_target_rl' in
 * `route'. `x_target_rl' must occur before `x_source_rl' in the route.
 * Any intervening route legs and leg groups are deleted.
 */
err_t
route_l_move(route_t *route, const route_leg_t *x_target_rl,
    const route_leg_t *x_source_rl)
{
	route_leg_t		*prev_rl;
	route_leg_t		*next_rl;
	route_leg_group_t	*prev_rlg;
	route_leg_group_t	*next_rlg;

	ASSERT(x_target_rl != NULL);
	ASSERT(x_source_rl != NULL);

	prev_rl = rl_prev_ndisc(route, (route_leg_t *)x_target_rl);
	next_rl = (route_leg_t *)x_source_rl;

	prev_rlg = (prev_rl != NULL ? prev_rl->rlg : NULL);
	next_rlg = next_rl->rlg;

#define	NEXT_OR_HEAD(list, elem) \
	((elem) != NULL ? list_next((list), (elem)) : list_head((list)))

	ASSERT(prev_rl != next_rl);
	if (prev_rlg != next_rlg) {
		/* Find all intervening rlgs and remove them */
		for (route_leg_group_t *rlg = NEXT_OR_HEAD(&route->leg_groups,
		    prev_rlg); rlg != next_rlg;
		    rlg = NEXT_OR_HEAD(&route->leg_groups, prev_rlg)) {
			ASSERT(rlg != NULL);
			rlg_destroy(route, rlg);
		}
		if (prev_rl != NULL) {
			switch(prev_rlg->type) {
			case ROUTE_LEG_GROUP_TYPE_AIRWAY:
				awy_split(route, prev_rlg, prev_rl, NULL,
				    B_FALSE);
				break;
			case ROUTE_LEG_GROUP_TYPE_PROC:
				rlg_shorten_proc(route, prev_rl, B_FALSE);
				break;
			case ROUTE_LEG_GROUP_TYPE_DIRECT:
				rlg_connect_neigh(route, next_rl->rlg,
				    B_TRUE, B_FALSE);
				break;
			default:
				
				break;
			}
		}
		switch(next_rlg->type) {
		case ROUTE_LEG_GROUP_TYPE_AIRWAY:
			awy_split(route, next_rlg, NULL,
			    list_next(&next_rlg->legs, next_rl), B_TRUE);
			break;
		case ROUTE_LEG_GROUP_TYPE_PROC:
			rlg_shorten_proc(route, next_rl, B_TRUE);
			break;
		case ROUTE_LEG_GROUP_TYPE_DIRECT:
			rlg_connect_neigh(route, next_rl->rlg, B_TRUE,
			    B_FALSE);
			break;
		default:
			break;
		}
	} else {
		/* None of these can contain more than one leg */
		ASSERT(prev_rlg->type != ROUTE_LEG_GROUP_TYPE_DISCO &&
		    prev_rlg->type != ROUTE_LEG_GROUP_TYPE_DIRECT);

		if (prev_rlg->type == ROUTE_LEG_GROUP_TYPE_AIRWAY) {
			awy_split(route, prev_rl->rlg, prev_rl,
			    list_next(&next_rlg->legs, next_rl), B_TRUE);
		} else {
			/*
			 * Procedures legs are simply removed. The segment
			 * generator will be able to deal with any BS we
			 * throw at it and the pilot is responsible for
			 * checking the sanity of the result.
			 */
			for (route_leg_t *rl = list_next(&route->legs, prev_rl);
			    rl != next_rl; rl = list_next(&route->legs,
			    prev_rl)) {
				ASSERT(rl != NULL);
				list_remove(&route->legs, rl);
				list_remove(&prev_rlg->legs, rl);
				free(rl);
			}
			route->segs_dirty = B_TRUE;
		}
	}

#undef	NEXT_OR_HEAD

	return (ERR_OK);
}

/*
 * Deletes the route leg `x_rl' from `route'.
 */
void
route_l_delete(route_t *route, const route_leg_t *x_rl)
{
	route_leg_t *rl = (route_leg_t *)x_rl;
	route_leg_group_t *rlg = rl->rlg;
	route_leg_group_t *rlg2;
	route_leg_t *prev_rl = list_prev(&rlg->legs, rl);
	route_leg_t *next_rl = list_next(&rlg->legs, rl);

	switch (rlg->type) {
	case ROUTE_LEG_GROUP_TYPE_AIRWAY:
		if (prev_rl != NULL && next_rl != NULL) {
			/* Split the airway and insert a disco */
			awy_split(route, prev_rl->rlg, prev_rl, next_rl,
			    B_FALSE);
		} else if (prev_rl != NULL && next_rl == NULL) {
			/* Shorten the airway from the right */
			rlg->end_wpt = *leg_get_end_wpt(prev_rl);
			rlg_update_awy_legs(route, rlg, B_FALSE);
			rlg2 = rlg_next_ndisc(route, rlg);
			rlg_connect(route, rlg, rlg2, B_FALSE, B_FALSE);
		} else if (prev_rl == NULL && next_rl != NULL) {
			/* Shorten the airway from the left */
			rlg->start_wpt = *leg_get_end_wpt(rl);
			rlg_update_awy_legs(route, rlg, B_FALSE);
			rlg2 = rlg_prev_ndisc(route, rlg);
			rlg_connect(route, rlg2, rlg, B_FALSE, B_FALSE);
		} else {
			/* Get rid of the whole thing */
			rlg_bypass(route, rl->rlg, B_FALSE, B_FALSE);
		}
		break;
	case ROUTE_LEG_GROUP_TYPE_PROC:
		/* We're the last one */
		if (prev_rl == NULL && next_rl == NULL) {
			rlg_bypass(route, rl->rlg, B_FALSE, B_FALSE);
		} else if (prev_rl == NULL) {
			/* First leg, check if we can adjust start_wpt. */
			wpt_t start_wpt;

			list_remove(&rlg->legs, rl);
			list_remove(&route->legs, rl);
			free(rl);
			start_wpt = rlg_find_start_fix(rlg);
			if (!IS_NULL_WPT(&start_wpt)) {
				rlg->start_wpt = start_wpt;
				rlg_connect_neigh(route, rlg, B_FALSE, B_FALSE);
			}
		} else if (next_rl == NULL) {
			/* Last leg, check if we can adjust end_wpt. */
			wpt_t end_wpt;

			list_remove(&rlg->legs, rl);
			list_remove(&route->legs, rl);
			free(rl);
			end_wpt = rlg_find_end_wpt(rlg);
			if (!IS_NULL_WPT(&end_wpt)) {
				rlg->end_wpt = end_wpt;
				rlg_connect_neigh(route, rlg, B_FALSE, B_FALSE);
			}
		} else {
			/* Internal delete, just remove it */
			list_remove(&rlg->legs, rl);
			list_remove(&route->legs, rl);
			free(rl);
		}
		break;
	case ROUTE_LEG_GROUP_TYPE_DIRECT:
		rlg_bypass(route, rl->rlg, B_FALSE, B_FALSE);
		break;
	case ROUTE_LEG_GROUP_TYPE_DISCO:
		ASSERT(list_prev(&route->legs, rl) != NULL &&
		    list_next(&route->legs, rl) != NULL);
		rlg_bypass(route, rl->rlg, B_TRUE, B_FALSE);
		break;
	default:
		assert(0);
	}
	route->segs_dirty = B_TRUE;
}

/*
 * Overrides the leg's navproc seg's altitude constraint with `l'.
 */
void
route_l_set_alt_lim(route_t *route, const route_leg_t *x_rl, alt_lim_t l)
{
	route_leg_t *rl = (route_leg_t *)x_rl;
	if (!rl->alt_lim_ovrd || memcmp(&rl->alt_lim, &l, sizeof (l)) != 0) {
		rl->alt_lim = l;
		rl->alt_lim_ovrd = B_TRUE;
		route->segs_dirty = B_TRUE;
	}
}

/*
 * Returns the active alt constraint on the route leg. If the route leg has
 * no overriding constraint, the leg's navproc seg's constraint is returned.
 */
alt_lim_t route_l_get_alt_lim(const route_leg_t *rl)
{
	if (rl->alt_lim_ovrd)
		return (rl->alt_lim);
	else
		return (rl->seg.alt_lim);
}

/*
 * Overrides the leg's navproc seg's speed constraint with `l'.
 */
void
route_l_set_spd_lim(route_t *route, const route_leg_t *x_rl, spd_lim_t l)
{
	route_leg_t *rl = (route_leg_t *)x_rl;
	if (!rl->spd_lim_ovrd || memcmp(&rl->spd_lim, &l, sizeof (l)) != 0) {
		rl->spd_lim = l;
		rl->spd_lim_ovrd = B_TRUE;
		route->segs_dirty = B_TRUE;
	}
}

/*
 * Returns the active speed constraint on the route leg. If the route leg has
 * no overriding constraint, the leg's navproc seg's constraint is returned.
 */
spd_lim_t
route_l_get_spd_lim(const route_leg_t *rl)
{
	if (rl->spd_lim_ovrd)
		return (rl->spd_lim);
	else
		return (rl->seg.spd_lim);
}

/*
 * Calculates the radius of a flight arc.
 *
 * @param speed Speed of flight through the arc in knots.
 * @param turn_rate Rate of turn in degrees per second.
 *
 * @return Arc radius in meters.
 */
static double
calc_arc_radius(double speed, double turn_rate)
{
	return (((360 / turn_rate) * KT2MPS(speed)) / (2 * M_PI));
}

#define	ARC_JOIN_THR	1
#define	STD_RATE_TURN	3
#define	STD_INCTP_ANGLE	30
#define	INTCP_SRCH_DIST	1e9

static route_seg_t *rs_join_dir_reintcp_trk(list_t *seglist, route_seg_t *rs1,
    route_seg_t *rs2, const fpp_t *fpp, double r, double rnp, vect2_t p1,
    vect2_t p2, vect2_t p3, vect2_t leg1_dir, vect2_t leg2, double rhdg,
    bool_t cw);
static route_seg_t *rs_join_dir_reintcp_dir(list_t *seglist, route_seg_t *rs1,
    route_seg_t *rs2, const fpp_t *fpp, double r, double rnp, vect2_t p1,
    vect2_t p2, vect2_t p3, double rhdg, bool_t cw);

static bool_t
point_is_on_arc(vect2_t p, vect2_t c, vect2_t s, vect2_t e, bool_t cw)
{
	double p_angle, angle1, angle2;

	p_angle = dir2hdg(vect2_sub(p, c));
	angle1 = dir2hdg(vect2_sub(s, c));
	angle2 = dir2hdg(vect2_sub(e, c));
	return (is_on_arc(p_angle, angle1, angle2, cw));
}

/*
 * Joins a leg segment to a subsequent leg segment of type
 * ROUTE_SEG_TYPE_DIRECT. This function performs several different
 * types of joins automatically depending on the specific geometry
 * of the required join:
 *
 * 1) If the difference between the outbound course from the first segment
 *	and the inbound course to the second segment is less than
 *	ARC_JOIN_THR, we simply mark the first segment as connected and
 *	exit, because the connection is too shallow to require construction
 *	of a connection arc.
 *
 * 2) If the course difference is greater than ARC_JOIN_THR, we attempt to
 *	construct an arc which connects the first and second segments in
 *	one turn. If such an arc deviates too far (above `rnp') from the
 *	p2 point (the point where the two segments meet), we construct a
 *	reintercept join instead. For a diagram of this join type, see
 *	doc/join_types/dir2dir_arc.png and doc/join_types/arc2dir_arc.png.
 *
 * 3) Reintercept joins come in one of two varieties:
 *
 *	a) A track-join consists of one arc which intersects the second
 *	   segment while approaching p2 up to a distance of `rnp' (in order
 *	   meet the rnp requirement). It then turns back to the second
 *	   segment an attempts to intersect with it at an angle of
 *	   STD_INTCP_ANGLE. Then at the intersection point with the second
 *	   segment we construct a second arc which smoothly rejoins the
 *	   outbound second segment leg. For a diagram of this join type, see
 *	   doc/join_types/dir2dir_reintcp_trk.png and
 *	   doc/join_types/arc2dir_reintcp_trk.png.
 *
 *	   If the intersection of the reintercept portion with the second
 *	   segment occurs beyond a point where a smooth rejoin of the
 *	   second segment is possible, we instead continue the turn on the
 *	   first arc and attempt to construct as sharp a turn as possible
 *	   to rejoin the second leg segment before reaching its end. For
 *	   the diagrams, see doc/join_types/dir2dir_reintcp_trk_fast.png and
 *	   doc/join_types/arc2dir_reintcp_trk_fast.png.
 *
 *	   If there isn't enough room even for a `fast' join as described
 *	   above, we degrade to a direct-to join (as described below).
 *
 *	b) A direct-to-join consists of one arc which intersects the
 *	   second segment while approaching p2 up to a distance of `rnp'.
 *	   It then turns directly towards the second segment's end point,
 *	   instead of attempting to reintercept the original track of the
 *	   second leg segment.
 *
 *	   If the second segment's endpoint is too close to the arc to be
 *	   able to meet it in the turn, we do not attempt to create the
 *	   arc and simply link the two segments together.
 *
 * @param seglist The segment list containing the route segments `rs1' and
 *	`rs2'.
 * @param rs1 The first route segment to connect.
 * @param rs2 The second route segment to connect. It must immediately
 *	follow `rs1' in `seglist'.
 * @param r The turn radius of any applicable maneuvers in meters.
 * @param rnp The maximum allowable deviation from the intersection of rs1
 *	and rs2. Passing zero here means we must overfly the intersection.
 * @param follow_track In case a simple arc to transition from rs1 to rs2
 *	deviates too far from p2 (it would violate the `rnp' constraint),
 *	this flag tells us whether we should construct a track-join which
 *	rejoins rs2's track, or a direct-to-join.
 */
static route_seg_t *
rs_join_dir(list_t *seglist, route_seg_t *rs1, route_seg_t *rs2,
    double r, double rnp, bool_t follow_track)
{
	vect2_t p1, p2, p3, leg1_dir, leg2, dp1, dp2, c, i1, i2, p2_i2;
	double rhdg, p2_i2_len, leg2_len;
	geo_pos2_t i1_pos, i2_pos, c_pos;
	route_seg_t *rs_arc;
	fpp_t fpp;

	ASSERT(list_next(seglist, rs1) == rs2);
	ASSERT(rs2->type == ROUTE_SEG_TYPE_DIRECT);

	/*
	 * In this function we use the following variables:
	 *
	 * *) p1, p2, p3: for dir2dir joins, p1-p2 is the first segment
	 *	leg and p2-p3 is the second segment leg. For arc2dir joins,
	 *	p1 and p2 define the first arc's center and endpoint (and
	 *	their distance is its radius). p2-p3 again define the second
	 *	segment's direct leg.
	 * *) leg1_dir: the outbound direction from the first segment.
	 * *) leg2: the second leg between p2 and p3. Since the second
	 *	segment is always a direct, this represents the leg exactly
	 *	(in the gnomonic projection space).
	 * *) rs_arc: is the arc which joins the first and second segment.
	 */
	fpp = gnomo_fpp_init(GEO3_TO_GEO2(rs2->direct.start), 0, NULL, B_TRUE);
	p2 = geo2fpp(GEO3_TO_GEO2(rs2->direct.start), &fpp);
	p3 = geo2fpp(GEO3_TO_GEO2(rs2->direct.end), &fpp);
	if (rs1->type == ROUTE_SEG_TYPE_DIRECT) {
		p1 = geo2fpp(GEO3_TO_GEO2(rs1->direct.start), &fpp);
		leg1_dir = vect2_set_abs(vect2_sub(p2, p1), 1);
	} else {
		ASSERT(rs1->type == ROUTE_SEG_TYPE_ARC);
		p1 = geo2fpp(rs1->arc.center, &fpp);
		leg1_dir = vect2_set_abs(vect2_norm(vect2_sub(p2, p1),
		    rs1->arc.cw), 1);
	}
	leg2 = vect2_sub(p3, p2);
	rhdg = rel_hdg(dir2hdg(leg1_dir), dir2hdg(leg2));

	if (ABS(rhdg) < ARC_JOIN_THR) {
		/* turn is shallow enough to do a simple join */
		rs1->join_type = ROUTE_SEG_JOIN_SIMPLE;
		return (rs1);
	}
	if (ABS(rhdg) > 180 - ARC_JOIN_THR)
		/* Almost complete course reversal, direct turn not possible */
		goto reintcp;

	/* dp2 displaces leg2 in parallel towards the join's inner angle */
	dp2 = vect2_set_abs(vect2_norm(leg2, rhdg >= 0), r);
	if (rs1->type == ROUTE_SEG_TYPE_DIRECT) {
		vect2_t leg1 = vect2_sub(p2, p1);

		/*
		 * dp1 displaces leg1 in the same manner as dp2. Their
		 * intersection gives the centerpoint `c' for the arc
		 * which smoothly connects leg1 and leg2.
		 */
		dp1 = vect2_set_abs(vect2_norm(leg1_dir, rhdg >= 0), r);
		c = vect2vect_isect(vect2_sub(p2, p1), vect2_add(p1, dp1),
		    leg2, vect2_add(p2, dp2), B_FALSE);
		ASSERT(!IS_NULL_VECT(c));
		i1 = vect2vect_isect(dp1, c, leg1, p1, B_FALSE);

		/* Intersection is past our source point, do a reintcp join. */
		if (vect2_dist(p1, p2) - vect2_dist(i1, p2) <= 0)
			goto reintcp;
	} else {
		bool_t outer = (rs1->arc.cw && rhdg < 0) ||
		    (!rs1->arc.cw && rhdg > 0);
		double g = vect2_dist(p2, p1);
		unsigned n;
		vect2_t p0, vs[2];

		/* Check the turn isn't too tight to execute */
		if (!outer && g <= r)
			goto reintcp;

		n = vect2circ_isect(leg2, vect2_add(p2, dp2), p1,
		    outer ? g + r : g - r, B_FALSE, vs);
		if (n == 0)
			goto reintcp;
		if (n == 2 && vect2_dist(vs[0], p2) > vect2_dist(vs[1], p2))
			vs[0] = vs[1];
		c = vs[0];
		n = vect2circ_isect(vect2_set_abs(vect2_sub(c, p1),
		    INTCP_SRCH_DIST), p1, p1, g, B_TRUE, vs);
		ASSERT(n == 1);
		i1 = vs[0];

		/* Check the intersection is on our arc or do a reinctp join. */
		p0 = geo2fpp(GEO3_TO_GEO2(rs1->arc.start), &fpp);
		if (VECT2_EQ(i1, p0) ||
		    !point_is_on_arc(i1, p1, p0, p2, rs1->arc.cw))
			goto reintcp;
	}
	if (vect2_dist(c, p2) - r > rnp)
		/* Arc deviates too far from `p2', do arc back-track. */
		goto reintcp;

	i2 = vect2vect_isect(dp2, c, leg2, p2, B_FALSE);
	ASSERT(!IS_NULL_VECT(i2) && !IS_NULL_VECT(i1));

	p2_i2 = vect2_sub(i2, p2);
	p2_i2_len = vect2_abs(p2_i2);
	leg2_len = vect2_abs(leg2);

	if (vect2_dist(i2, p2) >= leg2_len)
		/* arc would join outside of our legs */
		goto reintcp;

	/*
	 * Calculate contact point positions and interpolate leg altitudes
	 * and speeds.
	 */
	if (rs1->type == ROUTE_SEG_TYPE_DIRECT)
		i1_pos = fpp2geo(i1, &fpp);
	else
		i1_pos = fpp2geo(i1, &fpp);
	i2_pos = fpp2geo(i2, &fpp);
	c_pos = fpp2geo(c, &fpp);

	/* Create new arc segment to join them and insert it */
	rs_arc = rs_new_arc(i1_pos, i2_pos, c_pos, rhdg >= 0,
	    ROUTE_SEG_JOIN_SIMPLE);
	list_insert_after(seglist, rs1, rs_arc);

	/* Adjust rs1 and rs2 start/end points and speeds */
	if (rs1->type == ROUTE_SEG_TYPE_DIRECT)
		rs1->direct.end = i1_pos;
	else
		rs1->arc.end = i1_pos;
	rs1->join_type = ROUTE_SEG_JOIN_SIMPLE;

	rs2->direct.start = i2_pos;

	return (rs1);
reintcp:
	if (follow_track)
		return (rs_join_dir_reintcp_trk(seglist, rs1, rs2, &fpp, r, rnp,
		    p1, p2, p3, leg1_dir, leg2, rhdg, rhdg >= 0));
	else
		return (rs_join_dir_reintcp_dir(seglist, rs1, rs2, &fpp, r, rnp,
		    p1, p2, p3, rhdg, rhdg >= 0));
}

/*
 * This is a utility function called from rs_join_dir to perform
 * a track-reintercept-join in case any of the simple-arc join constraints
 * are violated. For a description of the arguments, see the internals
 * of rs_join_dir.
 */
static route_seg_t *
rs_join_dir_reintcp_trk(list_t *seglist, route_seg_t *rs1, route_seg_t *rs2,
    const fpp_t *fpp, double r, double rnp, vect2_t p1, vect2_t p2, vect2_t p3,
    vect2_t leg1_dir, vect2_t leg2, double rhdg, bool_t cw)
{
	vect2_t i1, c1, t, t_i2_dir, i2, c1_t;
	double leg2_len, p2_c_len, p2_i1_len, smooth_len;
	geo_pos2_t i1_pos, c1_pos;
	bool_t rs1_remove = B_FALSE;
	route_seg_t *rs_arc1, *rs_arc2;

	ASSERT(rs2->type == ROUTE_SEG_TYPE_DIRECT);

	/* Figure out where the first arc's centerpoint will be. */
	p2_c_len = rnp + r;
	p2_i1_len = sqrt(POW2(p2_c_len) - POW2(r));
	leg2 = vect2_sub(p3, p2);
	leg2_len = vect2_abs(leg2);
	if (rs1->type == ROUTE_SEG_TYPE_DIRECT) {
		vect2_t leg1 = vect2_sub(p2, p1);
		double leg1_len, p1_i1_len;

		leg1_len = vect2_abs(leg1);
		p1_i1_len = leg1_len - p2_i1_len;
		if (p1_i1_len < 0) {
			/* leg1 is being shortened to nothing */
			p1_i1_len = 0;
			rs1_remove = B_TRUE;
		}
		i1 = vect2_add(p1, vect2_set_abs(leg1, p1_i1_len));
		c1 = vect2_add(i1, vect2_set_abs(vect2_norm(leg1, cw), r));
		leg1_dir = vect2_set_abs(vect2_sub(p2, p1), 1);
	} else {
		bool_t outer = (rs1->arc.cw && rhdg < 0) ||
		    (!rs1->arc.cw && rhdg > 0);
		double srch_g, g, srch_r;
		unsigned n;
		vect2_t p0, vs[2];

		g = vect2_dist(p2, p1);
		if (!outer && g < r)
			goto errout;
		srch_g = (outer ? g + r : g - r);
		srch_r = MIN(srch_g, r + rnp);
		n = circ2circ_isect(p1, srch_g, p2, srch_r, vs);
		ASSERT(n != 0);
		if (n == 2) {
			double rhdg1 = rel_hdg(dir2hdg(vect2_sub(p2, p1)),
			    dir2hdg(vect2_sub(vs[0], p2)));

			if ((rs1->arc.cw && rhdg1 <= 0) ||
			    (!rs1->arc.cw && rhdg1 >= 0))
				c1 = vs[0];
			else
				c1 = vs[1];
		} else {
			c1 = vs[0];
		}
		n = vect2circ_isect(vect2_set_abs(vect2_sub(c1, p1),
		    INTCP_SRCH_DIST), p1, p1, g, B_TRUE, vs);
		ASSERT(n == 1);
		i1 = vs[0];

		/* Check the intersection is on our arc or do a reinctp join. */
		p0 = geo2fpp(GEO3_TO_GEO2(rs1->arc.start), fpp);
		if (VECT2_EQ(i1, p0) ||
		    !point_is_on_arc(i1, p1, p0, p2, rs1->arc.cw)) {
			i1 = p0;
			rs1_remove = B_TRUE;
		}
		leg1_dir = vect2_set_abs(vect2_norm(vect2_sub(p2, p1),
		    rs1->arc.cw), 1);
	}
	i1_pos = fpp2geo(i1, fpp);
	c1_pos = fpp2geo(c1, fpp);

	/* `t' is where the re-intercept line touches the first arc. */
	c1_t = vect2_set_abs(vect2_rot(leg2, cw ? STD_INCTP_ANGLE - 90 :
	    90 - STD_INCTP_ANGLE), r);
	t = vect2_add(c1, c1_t);

	smooth_len = tan(DEG2RAD(STD_INCTP_ANGLE / 2)) * r;
	/* `i2' is where the re-intercept line crosses leg2 */
	t_i2_dir = vect2_set_abs(vect2_norm(c1_t, cw), INTCP_SRCH_DIST);
	i2 = vect2vect_isect(t_i2_dir, t, leg2, p2, B_TRUE);
	/*
	 * Check that the intercept happens before reaching p3 and there
	 * is enough space there to put in another arc to smooth out the
	 * intercept.
	 */
	if (!IS_NULL_VECT(i2) && vect2_dist(i2, t) > smooth_len &&
	    vect2_dist(i2, p2) + smooth_len + rnp < vect2_abs(leg2)) {
		vect2_t i3, i4, c3, t_i2;
		geo_pos2_t i3_pos, i4_pos, t_pos, c3_pos;
		route_seg_t *rs_dir;

		t_i2 = vect2_sub(i2, t);
		i3 = vect2_add(t, vect2_set_abs(t_i2, vect2_abs(t_i2) -
		    smooth_len));
		i4 = vect2_add(p2, vect2_set_abs(leg2, vect2_dist(i2, p2) +
		    smooth_len));
		c3 = vect2_add(i4, vect2_set_abs(vect2_norm(leg2, !cw), r));

		t_pos = fpp2geo(t, fpp);
		i3_pos = fpp2geo(i3, fpp);
		i4_pos = fpp2geo(i4, fpp);
		c3_pos = fpp2geo(c3, fpp);

		rs_arc1 = rs_new_arc(i1_pos, t_pos, c1_pos, cw,
		    ROUTE_SEG_JOIN_SIMPLE);
		rs_dir = rs_new_direct(t_pos, i3_pos, ROUTE_SEG_JOIN_SIMPLE);
		rs_arc2 = rs_new_arc(i3_pos, i4_pos, c3_pos, !cw,
		    ROUTE_SEG_JOIN_SIMPLE);

		list_insert_after(seglist, rs1, rs_arc1);
		list_insert_after(seglist, rs_arc1, rs_dir);
		list_insert_after(seglist, rs_dir, rs_arc2);

		rs2->direct.start = i4_pos;
	} else {
		vect2_t c2, t2, t3, c1_c2, p2m, vs[2];
		unsigned n;
		geo_pos2_t t2_pos, t3_pos, c2_pos;

		/* Make the intercept as sharply as possible */
		p2m = vect2_add(p2, vect2_set_abs(vect2_norm(leg2, !cw), r));
		n = vect2circ_isect(leg2, p2m, c1, 2 * r, B_FALSE, vs);
		if (n == 0) {
			/*
			 * Final resort, try placing c1 and i1 onto p2. If
			 * we can't reintcp from there, no help.
			 */
			if (rnp != 0) {
				return (rs_join_dir_reintcp_trk(seglist, rs1,
				    rs2, fpp, r, 0, p1, p2, p3, leg1_dir, leg2,
				    rhdg, cw));
			} else {
				goto errout;
			}
		}
		if (n == 2 && vect2_dist(vs[0], p3) > vect2_dist(vs[1], p3))
			vs[0] = vs[1];
		c2 = vs[0];
		c2_pos = fpp2geo(c2, fpp);
		c1_c2 = vect2_sub(c2, c1);
		n = vect2circ_isect(c1_c2, c1, c1, r, B_TRUE, vs);
		ASSERT(n == 1);
		t2 = vs[0];
		t3 = vect2vect_isect(vect2_norm(leg2, cw), c2, leg2, p2,
		    B_TRUE);
		if (!IS_NULL_VECT(t3)) {
			t2_pos = fpp2geo(t2, fpp);
			t3_pos = fpp2geo(t3, fpp);

			rs_arc1 = rs_new_arc(i1_pos, t2_pos, c1_pos, cw,
			    ROUTE_SEG_JOIN_SIMPLE);
			rs_arc2 = rs_new_arc(t2_pos, t3_pos, c2_pos, !cw,
			    ROUTE_SEG_JOIN_SIMPLE);

			list_insert_after(seglist, rs1, rs_arc1);
			list_insert_after(seglist, rs_arc1, rs_arc2);

			rs2->direct.start = t3_pos;
		} else {
			vect2_t c1_p3, c1_t2;

			c1_p3 = vect2_sub(p3, c1);
			if (vect2_abs(c1_p3) <= r) {
				n = vect2circ_isect(vect2_set_abs(c1_p3, 2 * r),
				    c1, c1, r, B_TRUE, vs);
				ASSERT(n == 1);
				t2 = vs[0];
				t3 = vs[0];
			} else {
				double p3_c1_t2_angle = RAD2DEG(acos(r /
				    vect2_abs(c1_p3)));
				c1_t2 = vect2_set_abs(vect2_rot(c1_p3,
				    cw ? -p3_c1_t2_angle : p3_c1_t2_angle), r);
				t2 = vect2_add(c1, c1_t2);
			}

			if (!point_is_on_arc(p2, c1, i1, t2, cw))
				goto errout;

			t2_pos = fpp2geo(t2, fpp);
			t3_pos = fpp2geo(t3, fpp);
			rs_arc1 = rs_new_arc(i1_pos, t2_pos, c1_pos, cw,
			    ROUTE_SEG_JOIN_SIMPLE);
			list_insert_after(seglist, rs1, rs_arc1);

			if (!VECT2_EQ(t2, t3)) {
				rs2->direct.start = t2_pos;
			} else {
				list_remove(seglist, rs2);
				free(rs2);
			}
		}
	}
	if (rs1_remove) {
		list_remove(seglist, rs1);
		free(rs1);
		rs1 = rs_arc1;
	} else {
		if (rs1->type == ROUTE_SEG_TYPE_DIRECT)
			rs1->direct.end = i1_pos;
		else
			rs1->arc.end = i1_pos;
	}

	return (rs1);
errout:
	rs1->join_type = ROUTE_SEG_JOIN_SIMPLE;
	return (rs1);
}

/*
 * This is a utility function called from rs_join_dir to perform a
 * direct-to-join in case any of the simple-arc join constraints are violated.
 * For a description of the arguments, see the internals of rs_join_dir.
 */
static route_seg_t *
rs_join_dir_reintcp_dir(list_t *seglist, route_seg_t *rs1, route_seg_t *rs2,
    const fpp_t *fpp, double r, double rnp, vect2_t p1, vect2_t p2, vect2_t p3,
    double rhdg, bool_t cw)
{
	vect2_t c, i1, i2, vs[2], p3_c;
	double p3_c_dist;
	bool_t rs1_remove = B_FALSE;
	route_seg_t *rs_arc;
	geo_pos2_t i1_pos, i2_pos;
	geo_pos2_t c_pos;
	unsigned n;

	ASSERT(rs2->type == ROUTE_SEG_TYPE_DIRECT);

	if (ABS(rhdg) < ARC_JOIN_THR)
		goto errout;

	if (rs1->type == ROUTE_SEG_TYPE_DIRECT) {
		vect2_t leg1 = vect2_sub(p2, p1);
		vect2_t dc = vect2_set_abs(vect2_norm(leg1, cw), r);

		n = vect2circ_isect(leg1, vect2_add(p1, dc), p2, r + rnp,
		    B_TRUE, vs);
		if (n == 0) {
			i1 = p2;
			c = vect2_add(i1, dc);
		} else {
			if (n == 2 &&
			    vect2_dist(vs[0], p1) > vect2_dist(vs[1], p1))
				vs[0] = vs[1];
			c = vs[0];
		}
		i1 = vect2vect_isect(vect2_neg(dc), c, p1, leg1, B_FALSE);
		if (!SAME_DIR(i1, p1)) {
			i1 = p2;
			c = vect2_add(i1, dc);
		}
	} else {
		bool_t outer = ((rs1->arc.cw && !cw) || (!rs1->arc.cw && cw));
		double g = vect2_abs(vect2_sub(p2, p1));
		double srch_g = (outer ? g + r : g - r);
		double c_rhdg;
		vect2_t p0 = geo2fpp(GEO3_TO_GEO2(rs1->arc.start), fpp);

		if (srch_g <= 0)
			goto errout;
		n = circ2circ_isect(p1, srch_g, p2, r + rnp, vs);
		if (n == 0)
			goto errout;
		c_rhdg = rel_hdg(dir2hdg(vect2_sub(p2, p1)),
		    dir2hdg(vect2_sub(vs[0], p1)));
		if (n == 2 && ((rs1->arc.cw && c_rhdg > 0) ||
		    (!rs1->arc.cw && c_rhdg < 0)))
			vs[0] = vs[1];
		c = vs[0];
		if (!point_is_on_arc(c, p1, p0, p2, rs1->arc.cw)) {
			i1 = p0;
			c = vect2_add(i1, vect2_set_abs(vect2_sub(i1, p1),
			    outer ? r : -r));
		} else {
			n = vect2circ_isect(vect2_set_abs(vect2_sub(c, p1),
			    INTCP_SRCH_DIST), p1, p1, g, B_TRUE, vs);
			ASSERT(n == 1);
			i1 = vs[0];
		}
	}

	p3_c = vect2_sub(c, p3);
	p3_c_dist = vect2_abs(p3_c);
	if (p3_c_dist < r) {
		goto errout;
	} else if (p3_c_dist == r) {
		list_remove(seglist, rs2);
		free(rs2);
		rs2 = NULL;
		i2 = p3;
	} else {
		double theta = RAD2DEG(asin(r / p3_c_dist));
		double p3_c_hdg = dir2hdg(p3_c);
		double p3_i2_hdg = p3_c_hdg - theta;
		double p3_i2_dist = sqrt(POW2(p3_c_dist) - POW2(r));

		ASSERT(!isnan(p3_i2_dist));
		i2 = vect2_add(p3, vect2_set_abs(hdg2dir(p3_i2_hdg),
		    p3_i2_dist));
	}

	i1_pos = fpp2geo(i1, fpp);
	c_pos = fpp2geo(c, fpp);
	i2_pos = fpp2geo(i2, fpp);
	rs_arc = rs_new_arc(i1_pos, i2_pos, c_pos, cw, ROUTE_SEG_JOIN_SIMPLE);
	list_insert_after(seglist, rs1, rs_arc);

	if (rs1_remove) {
		list_remove(seglist, rs1);
		free(rs1);
		rs1 = rs_arc;
	} else {
		if (rs1->type == ROUTE_SEG_TYPE_DIRECT) {
			rs1->direct.end = i1_pos;
		} else {
			rs1->arc.end = i1_pos;
		}
	}
	rs1->join_type = ROUTE_SEG_JOIN_SIMPLE;
	if (rs2 != NULL)
		rs2->direct.start = i2_pos;

	return (rs1);
errout:
	rs1->join_type = ROUTE_SEG_JOIN_SIMPLE;
	return (rs1);
}

static bool_t
rs_join_arc_find_c1_i1(const fpp_t *fpp, vect2_t p1, vect2_t p2, double r,
    double g, bool_t outer, double rnp, double rhdg, const route_seg_t *rs1,
    vect2_t *c1p, vect2_t *i1p, bool_t *rs1_removep)
{
	unsigned n;
	vect2_t vs[2];
	double g1 = vect2_dist(p2, p1);
	bool_t outer1 = (rhdg > 180 - ARC_JOIN_THR) ||
	    (rs1->arc.cw && rhdg < 0) ||
	    (!rs1->arc.cw && rhdg > 0);
	double srch_g1, srch_g, c1_rhdg;
	vect2_t p0 = geo2fpp(GEO3_TO_GEO2(rs1->arc.start), fpp);

	srch_g1 = (outer1 ? g1 + r : g1 - r);
	srch_g = (outer ? g + r : g - r);
	if (srch_g <= 0)
		return (B_FALSE);
	n = circ2circ_isect(p2, r + rnp, p1, srch_g1, vs);
	*c1p = NULL_VECT2;
	if (n != 0) {
		c1_rhdg = rel_hdg(dir2hdg(vect2_sub(p2, p1)),
		    dir2hdg(vect2_sub(vs[0], p1)));
		/* Check that the point actually lies on the arc ahead of p2. */
		if (n == 2 && ((rs1->arc.cw && c1_rhdg > 0) ||
		    (!rs1->arc.cw && c1_rhdg < 0)))
			vs[0] = vs[1];
		*c1p = vs[0];
		n = vect2circ_isect(vect2_set_abs(vect2_sub(*c1p, p1),
		    INTCP_SRCH_DIST), p1, p1, g1, B_TRUE, vs);
		ASSERT(n != 0);
		*i1p = vs[0];
		if (!point_is_on_arc(*i1p, p1, p0, p2, rs1->arc.cw))
			*c1p = NULL_VECT2;
	}
	if (IS_NULL_VECT(*c1p)) {
		*i1p = p0;
		*rs1_removep = B_TRUE;
		*c1p = vect2_add(*i1p, vect2_set_abs(vect2_sub(p1, *i1p), r));
	}

	return (B_TRUE);
}

static route_seg_t *
rs_join_arc(list_t *seglist, route_seg_t *rs1, route_seg_t *rs2,
    double r, double rnp)
{
	fpp_t fpp;
	vect2_t leg1_dir, p1, p2, p3, c, c1, i1, vs[2];
	bool_t cw, outer;
	double rhdg, g;
	route_seg_t *rs_arc1, *rs_arc2;
	unsigned n;
	bool_t rs1_remove = B_FALSE;
	geo_pos2_t i1_pos;

	/*
	 * The grand logic of this function works as follows:
	 *
	 * 1) We attempt to make a direct turn from rs1 onto rs2. Regardless
	 *	if it is an arc or a direct, we are constrained by two factors:
	 *	a) our turn must be of radius `r' (dictated by flight speed)
	 *	b) we mustn't deviate from the crossover point.
	 * 2) If the direct turn is not possible (due to deviating too far
	 *	from p2), we instead calculate an arc (rs_arc1) that gets
	 *	just close enough to p2 to meet the rnp constraint. Since we
	 *	already know that a direct turn onto the outbound arc is not
	 *	possible, we know that rs_arc1 will intersect it. We then
	 *	construct a second arc (rs_arc2) which turns in the opposite
	 *	direction to rs_arc1 to rejoin the outbound route segment.
	 *
	 * This function defines the following generally used variables:
	 *
	 * *) p1, p2, p3: are key points along the join being calculated.
	 *	The meaning of p1 and p2 depend on the type of rs1. For
	 *	DIRECT segments, they are the starting and ending points
	 *	of that segment. For ARC segments, p1 is the arc's center
	 *	and p2 is the arc's end point. p3 is always the outbound
	 *	arc's endpoint. Point p2 is where rs1 and rs2 meet and is
	 *	this each respective segments start/end point.
	 * *) c: is the outbound (rs2) arc's center point.
	 * *) c1: is the center point of rs_arc1.
	 * *) i1: is the first intersection point where rs_arc1 touches
	 *	the inbound route segment. The line from i1 to c1 is always
	 *	perpendicular to the inbound route segment at point i1.
	 * *) cw: indicates whether the outbound arc segment is clockwise.
	 * *) outer: indicates whether the join onto the outbound arc
	 *	segment is performed from outside the outbound arc or inside
	 *	(this depends on the inbound leg course at point p2).
	 * *) rhdg: the relative heading of the inbound-to-outbound course
	 *	at point p2. This tells us the direction and how much to turn
	 *	to transition from rs1 onto rs2.
	 * *) g: the radius of outbound arc in rs2.
	 * *) rs1_remove: a flag which if raised indicates that we can remove
	 *	rs1 from the seglist and completely destroy it. This happens
	 *	when while calculating the join from rs1 onto rs2, rs1 got
	 *	shortented to zero length.
	 * *) leg1_dir: a unit vector pointing in the direction of travel
	 *	on the inbound route segment at point p2.
	 */

	/* initial variable setup */
	fpp = gnomo_fpp_init(GEO3_TO_GEO2(rs2->arc.start), 0, NULL, B_TRUE);
	p2 = geo2fpp(GEO3_TO_GEO2(rs2->arc.start), &fpp);
	p3 = geo2fpp(GEO3_TO_GEO2(rs2->arc.end), &fpp);
	if (rs1->type == ROUTE_SEG_TYPE_DIRECT) {
		p1 = geo2fpp(GEO3_TO_GEO2(rs1->direct.start), &fpp);
		leg1_dir = vect2_set_abs(vect2_sub(p2, p1), 1);
	} else {
		p1 = geo2fpp(rs1->arc.center, &fpp);
		leg1_dir = vect2_set_abs(vect2_norm(vect2_sub(p2, p1),
		    rs1->arc.cw), 1);
	}
	c = geo2fpp(rs2->arc.center, &fpp);
	g = vect2_dist(c, p2);
	cw = rs2->arc.cw;
	rhdg = rel_hdg(dir2hdg(leg1_dir), dir2hdg(vect2_norm(vect2_sub(p2, c),
	    cw)));
	if (ABS(rhdg) < ARC_JOIN_THR) {
		/* If the join is too shallow to bother, we're done */
		rs1->join_type = ROUTE_SEG_JOIN_SIMPLE;
		return (rs1);
	}
	/*
	 * We are joining the outbound arc from the outside if:
	 * a) the relative angle of the inbound-to-outbound direction is
	 *	nearly 180 degrees in either direction
	 * b) relative heading is negative (left turn) if the outbound arc is
	 *	clockwise
	 * c) relative heading is positive (right turn) if the outbound arc
	 *	is counter-clockwise.
	 */
	outer = (ABS(rhdg) > 180 - ARC_JOIN_THR) || (cw ? rhdg < 0 : rhdg > 0);

	/*
	 * First attempt to create a direct join onto the outbound arc. This
	 * is primarily limited by how far this join takes us from p2. If
	 * this distance is greater than rnp, we'll have to figure out a
	 * rejoin of the outbound arc.
	 */
	c1 = NULL_VECT2;
	if (rs1->type == ROUTE_SEG_TYPE_DIRECT) {
		vect2_t dp1, leg1;

		leg1 = vect2_sub(p2, p1);
		dp1 = vect2_set_abs(vect2_norm(leg1, outer ? !cw : cw), r);
		if (outer || g > r) {
			/*
			 * Direct-to-arc joins are calculated by attempting to
			 * have the direct leg1 (offset to the side by `r'
			 * towards the arc join direction) intersect the
			 * outbound arc with the radius adjusted by `r'
			 * (either up or down depending on outer/inner). The
			 * point closer to p2 is our rs_arc1 center c1.
			 */
			n = vect2circ_isect(leg1, vect2_add(p1, dp1), c,
			    outer ? g + r : g - r, B_TRUE, vs);
			if (n == 2 &&
			    vect2_dist(vs[0], p2) > vect2_dist(vs[1], p2))
				vs[0] = vs[1];
			if (n > 0)
				c1 = vs[0];
			if (!IS_NULL_VECT(c1) &&
			    vect2_dist(c1, p2) - r <= rnp) {
				/*
				 * Take the normal from the join arc center c1
				 * towards leg1. Where it intersects is i1.
				 */
				i1 = vect2vect_isect(vect2_neg(dp1), c1, leg1,
				    p1, B_FALSE);
				ASSERT(!IS_NULL_VECT(i1));
			} else {
				/*
				 * Simple join doesn't exist or takes us too
				 * far from p2. Do a reintercept.
				 */
				c1 = NULL_VECT2;
			}
		}
	} else {
		double g1;
		unsigned n;
		vect2_t vs[2];
		/* Same principle as `outer' */
		bool_t outer1 = (ABS(rhdg) > 180 - ARC_JOIN_THR) ||
		    (rs1->arc.cw && rhdg < 0) ||
		    (!rs1->arc.cw && rhdg > 0);

		/*
		 * Direct intercept from the inbound arc onto the outbound
		 * arc is calculated by having each arc's radius adjusted
		 * (either up or down depending on respective outer flag).
		 */
		g1 = vect2_dist(p2, p1);
		if ((outer1 || g1 > r) && (outer || g > r)) {
			double c1_rhdg;

			/*
			 * Check that the candidate point doesn't stray too
			 * far from p2.
			 */
			n = circ2circ_isect(p1, outer1 ? g1 + r : g1 - r,
			    c, outer ? g + r : g - r, vs);
			/*
			 * Check that the point actually lies on the arc
			 * just ahead of p2.
			 */
			c1_rhdg = rel_hdg(dir2hdg(vect2_sub(p2, p1)),
			    dir2hdg(vect2_sub(vs[0], p1)));
			if (n == 2 && ((rs1->arc.cw && c1_rhdg > 0) ||
			    (!rs1->arc.cw && c1_rhdg < 0) ||
			    (outer == outer1 && vect2_dist(vs[0], p2) >
			    vect2_dist(vs[1], p2)) ||
			    (outer != outer1 && vect2_dist(vs[0], p2) <
			    vect2_dist(vs[1], p2))))
				vs[0] = vs[1];
			if (n > 0)
				c1 = vs[0];
			if (!IS_NULL_VECT(c1) &&
			    vect2_dist(c1, p2) - r <= rnp) {
				/*
				 * Draw a ray out from the rs1's center p1
				 * to rs_arc1's center c1. Where it intersects
				 * rs1 is the i1 point (rs_arc1's start).
				 */
				n = vect2circ_isect(vect2_set_abs(vect2_sub(c1,
				    p1), INTCP_SRCH_DIST), p1, p1, g1, B_TRUE,
				    vs);
				ASSERT(n == 1);
				i1 = vs[0];
			} else {
				/* rs_arc1 is too far from p2 to satisfy rnp */
				c1 = NULL_VECT2;
			}
		}
	}

	if (!IS_NULL_VECT(c1)) {
		/*
		 * Direct join onto arc possible. We will only need one arc
		 * (rs_arc1) and won't be reintercepting the outbound arc.
		 */
		vect2_t i2;
		geo_pos2_t i2_pos, c1_pos;

		/*
		 * Draw a ray from the outbound arc center c to the join arc
		 * rs_arc1's center c1. Where it intersects the outbound arc
		 * is the point i2, rs_arc1's end point.
		 */
		n = vect2circ_isect(vect2_set_abs(vect2_sub(c1, c),
		    INTCP_SRCH_DIST), c, c, g, B_TRUE, vs);
		ASSERT(n == 1);
		i2 = vs[0];

		i1_pos = fpp2geo(i1, &fpp);
		i2_pos = fpp2geo(i2, &fpp);
		c1_pos = fpp2geo(c1, &fpp);
		rs_arc1 = rs_new_arc(i1_pos, i2_pos, c1_pos, outer ? !cw : cw,
		    ROUTE_SEG_JOIN_SIMPLE);

		list_insert_after(seglist, rs1, rs_arc1);
		rs2->arc.start = i2_pos;
	} else if (outer || g > r) {
		/*
		 * Direct join onto outbound arc rs2 not possible. We'll have
		 * to cross it and reintercept.
		 */
		vect2_t c2, i4, i5;
		geo_pos2_t i4_pos, i5_pos, c1_pos, c2_pos;
		double c_p2_angle, c_p3_angle, c_i5_angle, intcp_angle;

		/* Locate rs_arc1's center and starting points c1 and i1 */
		if (rs1->type == ROUTE_SEG_TYPE_DIRECT) {
			/*
			 * Try placing the first join arc rs_arc1 as far back
			 * from point p2 so that rnp is satisfied just as it
			 * passes near it.
			 */
			double p2_i1_len = sqrt(POW2(rnp + r) - POW2(r));
			vect2_t leg1 = vect2_sub(p2, p1);
			double leg1_len = vect2_abs(leg1);

			if (p2_i1_len > leg1_len) {
				/*
				 * Join would be outside of leg1, adjust
				 * rs_arc1's starting point to the starting
				 * point of leg1 and remove leg1.
				 */
				rs1_remove = B_TRUE;
				p2_i1_len = vect2_abs(leg1);
			}
			i1 = vect2_add(p2, vect2_set_abs(vect2_neg(leg1),
			    p2_i1_len));
			c1 = vect2_add(i1, vect2_set_abs(vect2_norm(leg1,
			    outer ? !cw : cw), r));
		} else {
			if (!rs_join_arc_find_c1_i1(&fpp, p1, p2, r, g, outer,
			    rnp, rhdg, rs1, &c1, &i1, &rs1_remove))
				goto errout;
		}
		/*
		 * The second join arc rs_arc2 reintercepts the outbound arc
		 * from rs_arc1.
		 */
		n = circ2circ_isect(c1, 2 * r, c, outer ? g - r : g + r, vs);
		if (n == 0)
			goto errout;

		if (n == 2 && vect2_dist(vs[0], p2) < vect2_dist(vs[1], p2))
			vs[0] = vs[1];
		c2 = vs[0];
		ASSERT(!IS_NULL_VECT(c2));
		n = vect2circ_isect(vect2_set_abs(vect2_sub(c2, c1),
		    INTCP_SRCH_DIST), c1, c1, r, B_TRUE, vs);
		ASSERT(n == 1);
		i4 = vs[0];
		n = vect2circ_isect(vect2_set_abs(vect2_sub(c2, c),
		    INTCP_SRCH_DIST), c, c, g, B_TRUE, vs);
		ASSERT(n == 1);
		i5 = vs[0];
		/*
		 * Check that the intercept is going to happen without having
		 * to reloop around. We expect that the second loop is pointing
		 * towards the rs2 arc.
		 */
		intcp_angle = rel_hdg(dir2hdg(vect2_norm(vect2_sub(i4, c2),
		    cw)), dir2hdg(vect2_norm(vect2_sub(i4, c), cw)));
		if ((!cw && intcp_angle >= 0) || (cw && intcp_angle <= 0))
			goto errout;

		c_p2_angle = dir2hdg(vect2_sub(p2, c));
		c_p3_angle = dir2hdg(vect2_sub(p3, c));
		c_i5_angle = dir2hdg(vect2_sub(i5, c));
		/*
		 * Check that the i5 outbound arc rejoin doesn't lie outside
		 * of the outbound arc.
		 */
		if (!point_is_on_arc(i5, c, p2, p3, cw))
			goto errout;

		i1_pos = fpp2geo(i1, &fpp);
		i4_pos = fpp2geo(i4, &fpp);
		i5_pos = fpp2geo(i5, &fpp);
		c1_pos = fpp2geo(c1, &fpp);
		c2_pos = fpp2geo(c2, &fpp);

		rs_arc1 = rs_new_arc(i1_pos, i4_pos, c1_pos, outer ? !cw : cw,
		    ROUTE_SEG_JOIN_SIMPLE);
		rs_arc2 = rs_new_arc(i4_pos, i5_pos, c2_pos, outer ? cw : !cw,
		    ROUTE_SEG_JOIN_SIMPLE);
		list_insert_after(seglist, rs1, rs_arc1);
		list_insert_after(seglist, rs_arc1, rs_arc2);

		rs2->arc.start = i5_pos;
	} else {
		goto errout;
	}

	if (rs1_remove) {
		list_remove(seglist, rs1);
		free(rs1);
		rs1 = rs_arc1;
	} else {
		if (rs1->type == ROUTE_SEG_TYPE_DIRECT)
			rs1->direct.end = i1_pos;
		else
			rs1->arc.end = i1_pos;
		rs1->join_type = ROUTE_SEG_JOIN_SIMPLE;
	}

	return (rs1);
errout:
	rs1->join_type = ROUTE_SEG_JOIN_SIMPLE;
	return (rs1);
}

/*
 * Creates a smooth joint between two route segments. The joint type is
 * determined by the type of each route segment and the first segment's
 * join_type.
 *
 * @param seglist List of route segments which will be modified to contain
 *	any new necessary new intermediate route segments to complete the
 *	join.
 * @param rs1 First segment to join.
 * @param rs2 Second segment to join.
 */
route_seg_t *
route_seg_join(list_t *seglist, route_seg_t *rs1, route_seg_t *rs2,
    double wpt_rnp, double spd)
{
	double		r;

	ASSERT(list_next(seglist, rs1) == rs2);

	if (rs1->join_type == ROUTE_SEG_JOIN_SIMPLE)
		return (rs1);
	r = calc_arc_radius(spd, STD_RATE_TURN);

	if (rs2->type == ROUTE_SEG_TYPE_DIRECT) {
		return (rs_join_dir(seglist, rs1, rs2, r, wpt_rnp,
		    rs1->join_type == ROUTE_SEG_JOIN_TRACK));
	} else {
		return (rs_join_arc(seglist, rs1, rs2, r, wpt_rnp));
	}
}

/*
 * Constructs a new ROUTE_SEG_TYPE_DIRECT route segment and returns it.
 */
static route_seg_t *
rs_new_direct(geo_pos2_t start, geo_pos2_t end, route_seg_join_type_t join_type)
{
	route_seg_t *rs = calloc(sizeof (*rs), 1);

	rs->type = ROUTE_SEG_TYPE_DIRECT;
	rs->direct.start = start;
	rs->direct.end = end;
	rs->join_type = join_type;

	return (rs);
}

/*
 * Constructs a new ROUTE_SEG_TYPE_ARC route segment and returns it.
 */
static route_seg_t *
rs_new_arc(geo_pos2_t start, geo_pos2_t end, geo_pos2_t center, bool_t cw,
    route_seg_join_type_t join_type)
{
	route_seg_t *rs = calloc(sizeof (*rs), 1);

	rs->type = ROUTE_SEG_TYPE_ARC;
	rs->arc.start = start;
	rs->arc.end = end;
	rs->arc.center = center;
	rs->arc.cw = cw;
	rs->join_type = join_type;

	return (rs);
}

/*
 * Given a route segment of type ROUTE_SEG_TYPE_ARC, calculates the arc radius.
 */
static double
arc_seg_get_radius(const route_seg_t *rs)
{
	vect3_t ctr_v = geo2ecef(GEO2_TO_GEO3(rs->arc.center, 0), &wgs84);
	vect3_t start_v = geo2ecef(GEO2_TO_GEO3(GEO3_TO_GEO2(rs->arc.start),
	    0), &wgs84);
	ASSERT(rs->type == ROUTE_SEG_TYPE_ARC);
	return (vect3_abs(vect3_sub(ctr_v, start_v)));
}
