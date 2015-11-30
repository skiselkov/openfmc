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

#include <stdlib.h>
#include <stddef.h>
#include <string.h>

#include "route.h"

static void rlg_connect(route_t *route, route_leg_group_t *prev_rlg,
    route_leg_group_t *next_rlg, bool_t allow_mod);
static void rlg_bypass(route_t *route, route_leg_group_t *rlg,
    bool_t allow_mod);
static route_leg_t * last_leg_before_rlg(route_t *route,
    route_leg_group_t *rlg);

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
rlg_new(route_leg_group_type_t type)
{
	route_leg_group_t *rlg = calloc(sizeof (*rlg), 1);

	rlg->type = type;
	rlg->start_fix = null_fix;
	rlg->end_fix = null_fix;
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
	route_leg_group_t *rlg = rlg_new(ROUTE_LEG_GROUP_TYPE_DISCO);
	route_leg_t *rl = calloc(sizeof (*rl), 1);

	/* A disco can't be first or last in the list */
	ASSERT(prev_rlg != NULL);
	ASSERT(list_tail(&route->leg_groups) != prev_rlg);
	list_insert_after(&route->leg_groups, prev_rlg, rlg);
	rl->disco = B_TRUE;
	rl->parent = rlg;
	list_insert_head(&rlg->legs, rl);
	list_insert_after(&route->legs, last_leg_before_rlg(route, rlg), rl);
}

/*
 * Returns a pointer to the end fix of a particular route leg. If the
 * route leg doesn't end in a fix, returns a pointer to `null_fix' instead.
 */
static const fix_t *
leg_get_end_fix(const route_leg_t *leg)
{
	return (!leg->disco ? navproc_seg_get_end_fix(&leg->seg) : &null_fix);
}

/*
 * Sets the end fix of `leg' to `fix'. If the leg is of a type that doesn't
 * take fixes, causes an assertion failure.
 */
static void
leg_set_end_fix(route_leg_t *leg, const fix_t *fix)
{
	navproc_seg_set_end_fix(&leg->seg, fix);
}

static bool_t
chk_awy_fix_adjacent(route_leg_group_t *rlg, const fix_t *fix, bool_t head)
{
	unsigned i;

	ASSERT(rlg->type == ROUTE_LEG_GROUP_TYPE_AIRWAY);
	if (IS_NULL_FIX(&rlg->start_fix) || IS_NULL_FIX(&rlg->end_fix))
		return (B_FALSE);
	for (i = 0; i < rlg->awy->num_segs &&
	    !FIX_EQ(&rlg->awy->segs[i].endpt[0], head ? fix : &rlg->end_fix);
	    i++)
		;
	return (i < rlg->awy->num_segs &&
	    FIX_EQ(&rlg->awy->segs[i].endpt[1], head ? &rlg->start_fix : fix));
}

/*
 * Creates a new DF (direct-to-fix) leg and returns it.
 */
static route_leg_t *
rl_new_direct(const fix_t *fix, route_leg_group_t *rlg)
{
	route_leg_t *rl = calloc(sizeof (*rl), 1);
	rl->seg.type = NAVPROC_SEG_TYPE_DIR_TO_FIX;
	rl->seg.term_cond.fix = *fix;
	rl->parent = rlg;
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
 * fix.
 *
 * @param route The route to which the leg group belongs.
 * @param rlg The route leg group to which the leg is supposed to belong.
 * @param rl The route leg to check. If you pass NULL here, the leg will be
 *	created (as a DF leg) and will be inserted into the parent leg group
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
    const fix_t *end_fix, route_leg_t *prev_rlg_rl, route_leg_t *prev_route_rl)
{
	if (rl == NULL) {
		/* Route leg doesn't exist, recreate & reinsert. */
		rl = rl_new_direct(end_fix, rlg);
		list_insert_after(&rlg->legs, prev_rlg_rl, rl);
		list_insert_after(&route->legs, prev_route_rl, rl);
	} else {
		/* Route leg exists, check settings & position in leg list. */
		ASSERT(rl->seg.type == NAVPROC_SEG_TYPE_DIR_TO_FIX);
		ASSERT(rl != prev_rlg_rl);
		ASSERT(rl != prev_route_rl);
		if (!FIX_EQ(leg_get_end_fix(rl), end_fix)) {
			/* End fix incorrect, reset */
			leg_set_end_fix(rl, end_fix);
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
		const fix_t *endfix;
		const airway_t *awy;
		awy = airway_db_lookup(route->navdb->awydb, rlg->awy->name,
		    !IS_NULL_FIX(&rlg->start_fix) ? &rlg->start_fix : NULL,
		    !IS_NULL_FIX(&rlg->end_fix) ? rlg->end_fix.name : NULL,
		    &endfix);
		VERIFY(awy != NULL);
		ASSERT(IS_NULL_FIX(&rlg->end_fix) ||
		    FIX_EQ(&rlg->end_fix, endfix));
		rlg->awy = awy;
	}

	if (IS_NULL_FIX(&rlg->start_fix) || IS_NULL_FIX(&rlg->end_fix)) {
		/*
		 * If we're missing either the start/end fix, we can't
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
		    !FIX_EQ(&rlg->start_fix, &rlg->awy->segs[i].endpt[0]); i++)
			;
		ASSERT(i < rlg->awy->num_segs);
		/* Pass over all airway segments in order & adapt our legs */
		for (; i < rlg->awy->num_segs &&
		    !FIX_EQ(&rlg->end_fix, &rlg->awy->segs[i].endpt[0]); i++) {
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
	(void) rlg_update_leg(route, rlg, list_head(&rlg->legs), &rlg->end_fix,
	    NULL, last_leg_before_rlg(route, rlg));
}

/*
 * Returns the first non-DISCO route leg group following `ref'.
 */
static route_leg_group_t *
rlg_next_ndisc(route_t *route, route_leg_group_t *ref)
{
	for (route_leg_group_t *rlg = list_next(&route->leg_groups, ref); rlg;
	    rlg = list_next(&route->leg_groups, rlg)) {
		if (rlg->type != ROUTE_LEG_GROUP_TYPE_DISCO)
			return (rlg);
	}
	return (NULL);
}

/*
 * Returns the first non-DISCO route leg group preceding `ref'.
 */
static route_leg_group_t *
rlg_prev_ndisc(route_t *route, route_leg_group_t *ref)
{
	for (route_leg_group_t *rlg = list_prev(&route->leg_groups, ref); rlg;
	    rlg = list_prev(&route->leg_groups, rlg)) {
		if (rlg->type != ROUTE_LEG_GROUP_TYPE_DISCO)
			return (rlg);
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
		route_leg_group_t *rlg_next = list_next(&route->leg_groups,
		    rlg);
		if (rlg->type == ROUTE_LEG_GROUP_TYPE_PROC &&
		    rlg->proc->arpt == arpt) {
			rlg_bypass(route, rlg, B_FALSE);
		}
		rlg = rlg_next;
	}

	route->segs_dirty = B_TRUE;
}

static bool_t
is_intc_leg(const route_leg_t *rl)
{
	ASSERT(rl != NULL);
	switch (rl->seg.type) {
	case NAVPROC_SEG_TYPE_CRS_TO_INTCP:
	case NAVPROC_SEG_TYPE_HDG_TO_INTCP:
		return (B_TRUE);
	default:
		return (B_FALSE);
	}
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
 *		*) If the two airways already share an end/start fix, no-op.
 *		*) Attempts to locate a suitable intersection point between
 *		   the airways and set that as the end/start fix combo. If no
 *		   intersection point was found, returns ERR_AWY_AWY_MISMATCH.
 *	b) If the second leg group is a DIRECT:
 *		*) If the preceding airway already had an endpoint set, will
 *		   simply connect the second direct leg group's fix to the
 *		   airway's endpoint.
 *		*) If the preceding airway had no endpoint set but does have
 *		   a start fix, will attempt locate the subsequent direct leg
 *		   group's end fix on the airway. If successful, will delete
 *		   the direct leg group and instead set the airway's end point
 *		   to that fix. If not, will return ERR_AWY_WPT_MISMATCH.
 *		*) If the preceding airway is lacking a start_fix, will always
 *		   return ERR_AWY_WPT_MISMATCH.
 *	c) If the second leg group is a PROC:
 *		*) If the airway's end fix is identical to the procedure's
 *		   start fix, no-op.
 *		*) Otherwise tries to look for the procedure's start fix on
 *		   the airway and attempts to find a way to extend the airway
 *		   segment up to the procedure's start. If there is no fix
 *		   on the airway that ends at the procedure, returns
 *		   ERR_AWY_PROC_MISMATCH.
 *
 * 3) If the first leg group is a DIRECT or PROC:
 *	a) If the second leg group is an AIRWAY, attempts to locate the
 *	   first LG's end fix on the airway. If the airway already has an end
 *	   fix defined, will take that into consideration so that we maintain
 *	   correct airway orientation. If the first LG's end fix is not present
 *	   on the airway, returns ERR_AWY_WPT_MISMATCH.
 *	b) If the second leg group is a DIRECT, sets the second LG's start
 *	   fix to the first LG's end fix.
 *	c) If the second leg group is a PROC, checks if the first LG's end fix
 *	   is identical to the PROC's start fix. If it is, no-op, otherwise
 *	   returns ERR_WPT_PROC_MISMATCH.
 */
static err_t
rlg_try_connect(route_t *route, route_leg_group_t *prev_rlg,
    route_leg_group_t **next_rlgpp, bool_t allow_mod)
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
			const fix_t *isect;
			bool_t awy_end_overlap;

			if (FIX_EQ(&prev_rlg->end_fix, &next_rlg->start_fix))
				break;

			if (!allow_mod)
				return (ERR_AWY_AWY_MISMATCH);

			/* locate: AWY x AWY */
			isect = airway_db_lookup_awy_intersection(
			    route->navdb->awydb, prev_rlg->awy->name,
			    prev_rlg->start_fix.name, next_rlg->awy->name);
			if (isect == NULL)
				return (ERR_AWY_AWY_MISMATCH);
			prev_rlg->end_fix = *isect;
			next_rlg->start_fix = *isect;
			/*
			 * This might have shortened the airway to where its
			 * start/end starts to overlap. Resolve this by deleting
			 * the awy's end fix and reconnecting it.
			 */
			awy_end_overlap = FIX_EQ(&next_rlg->end_fix,
			    &next_rlg->start_fix);
			if (awy_end_overlap)
				next_rlg->end_fix = null_fix;
			rlg_update_awy_legs(route, prev_rlg, B_TRUE);
			rlg_update_awy_legs(route, next_rlg, B_TRUE);
			if (awy_end_overlap) {
				rlg_connect(route, next_rlg,
				    rlg_next_ndisc(route, next_rlg), allow_mod);
			}
			route->segs_dirty = B_TRUE;
			break;
		}
		case ROUTE_LEG_GROUP_TYPE_DIRECT:
			/* AWY -> DIRECT */
			if (!IS_NULL_FIX(&prev_rlg->end_fix)) {
				/*
				 * AWY and DIRECT both end at same fix, we
				 * can't have that.
				 */
				if (FIX_EQ(&prev_rlg->end_fix,
				    &next_rlg->end_fix))
					return (ERR_AWY_WPT_MISMATCH);
				if (!IS_NULL_FIX(&next_rlg->start_fix) &&
				    !allow_mod)
					return (ERR_AWY_WPT_MISMATCH);
				next_rlg->start_fix = prev_rlg->end_fix;
				rlg_update_direct_leg(route, next_rlg);

				route->segs_dirty = B_TRUE;
			} else {
				/*
				 * Preceding airway is missing the endpoint,
				 * so try to locate the next DIRECT on the
				 * airway and remove the DIRECT leg group.
				 * If airway has null start_fix, this will
				 * never succeed.
				 */
				const fix_t *newendfix;
				const airway_t *newawy;

				if (!allow_mod)
					return (ERR_AWY_WPT_MISMATCH);
				newawy = airway_db_lookup(route->navdb->awydb,
				    prev_rlg->awy->name, &prev_rlg->start_fix,
				    next_rlg->end_fix.name, &newendfix);
				if (newawy == NULL ||
				    !FIX_EQ(newendfix, &next_rlg->end_fix))
					return (ERR_AWY_WPT_MISMATCH);
				prev_rlg->awy = newawy;
				prev_rlg->end_fix = next_rlg->end_fix;
				rlg_update_awy_legs(route, prev_rlg, B_FALSE);
				rlg_destroy(route, next_rlg);

				route->segs_dirty = B_TRUE;
			}
			break;
		case ROUTE_LEG_GROUP_TYPE_PROC: {
			/* AWY -> PROC */
			const airway_t *newawy;

			ASSERT(!IS_NULL_FIX(&next_rlg->start_fix));
			if (FIX_EQ(&prev_rlg->end_fix, &next_rlg->start_fix))
				/* already connected */
				break;
			if (!allow_mod)
				return (ERR_AWY_PROC_MISMATCH);
			newawy = airway_db_lookup(route->navdb->awydb,
			    prev_rlg->awy->name, &prev_rlg->start_fix,
			    next_rlg->start_fix.name, NULL);
			if (newawy == NULL)
				return (ERR_AWY_PROC_MISMATCH);
			prev_rlg->awy = newawy;
			prev_rlg->end_fix = next_rlg->start_fix;
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
			const fix_t *newendfix;
			const airway_t *newawy;

			if (FIX_EQ(&prev_rlg->end_fix, &next_rlg->start_fix))
				/* already connected */
				break;
			if (!allow_mod)
				return (ERR_AWY_PROC_MISMATCH);
			newawy = airway_db_lookup(route->navdb->awydb,
			    next_rlg->awy->name, &prev_rlg->end_fix,
			    IS_NULL_FIX(&next_rlg->end_fix) ? NULL :
			    next_rlg->end_fix.name, &newendfix);
			if (newawy != NULL &&
			    (IS_NULL_FIX(&next_rlg->end_fix) ||
			    FIX_EQ(&next_rlg->end_fix, newendfix))) {
				next_rlg->awy = newawy;
				next_rlg->start_fix = prev_rlg->end_fix;
				rlg_update_awy_legs(route, next_rlg, B_FALSE);

				route->segs_dirty = B_TRUE;
			} else {
				return (ERR_AWY_PROC_MISMATCH);
			}
			break;
		}
		case ROUTE_LEG_GROUP_TYPE_DIRECT:
			/* [DIRECT|PROC] -> DIRECT */
			ASSERT(!IS_NULL_FIX(&prev_rlg->end_fix) ||
			    prev_rlg->type == ROUTE_LEG_GROUP_TYPE_PROC);
			if (IS_NULL_FIX(&prev_rlg->end_fix))
				return (ERR_WPT_PROC_MISMATCH);
			if (FIX_EQ(&prev_rlg->end_fix, &next_rlg->end_fix)) {
				if (allow_mod) {
					/* Kill duplicates if allowed to */
					route_leg_group_t *new_next_rlg =
					    rlg_next_ndisc(route, next_rlg);
					rlg_destroy(route, next_rlg);
					rlg_connect(route, prev_rlg,
					    new_next_rlg, B_TRUE);
					*next_rlgpp = NULL;
					break;
				} else {
					return (ERR_DUPLICATE_LEG);
				}
			}
			if (!IS_NULL_FIX(&next_rlg->start_fix) && !allow_mod)
				return (ERR_WPT_PROC_MISMATCH);
			next_rlg->start_fix = prev_rlg->end_fix;
			break;
		case ROUTE_LEG_GROUP_TYPE_PROC:
			/* [DIRECT|PROC] -> PROC */
			/* If the fixes are equal, we're done */
			if (FIX_EQ(&prev_rlg->end_fix, &next_rlg->start_fix))
				break;
			/*
			 * As a special case, we always consider sequenced
			 * procedures to continuous PROVIDED they end in a
			 * suitable (INTC) leg.
			 */
			if (((prev_rlg->proc->type <= NAVPROC_TYPE_SID_TRANS &&
			    next_rlg->proc->type <= NAVPROC_TYPE_SID_TRANS) ||
			    (prev_rlg->proc->type >= NAVPROC_TYPE_STAR &&
			    next_rlg->proc->type >= NAVPROC_TYPE_STAR)) &&
			    is_intc_leg(list_tail(&prev_rlg->legs))) {
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
    route_leg_group_t *next_rlg, bool_t allow_mod)
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
	} else if (rlg_try_connect(route, prev_rlg, &next_rlg, allow_mod) ==
	    ERR_OK) {
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
rlg_connect_neigh(route_t *route, route_leg_group_t *rlg, bool_t allow_mod)
{
	rlg_connect(route, rlg_prev_ndisc(route, rlg), rlg, allow_mod);
	rlg_connect(route, rlg, rlg_next_ndisc(route, rlg), allow_mod);
}

/*
 * Given a route leg, deletes its entire route leg group and reconnects
 * its former neighbors with each other, entirely bypassing the deleted
 * route leg group.
 */
static void
rlg_bypass(route_t *route, route_leg_group_t *rlg, bool_t allow_mod)
{
	route_leg_group_t *prev_rlg = rlg_prev_ndisc(route, rlg);
	route_leg_group_t *next_rlg = rlg_next_ndisc(route, rlg);

	rlg_destroy(route, rlg);
	rlg_connect(route, prev_rlg, next_rlg, allow_mod);
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

	if (rlg != NULL) {
		route_leg_group_t *prev_rlg, *next_rlg;
		prev_rlg = rlg_prev_ndisc(route, rlg);
		next_rlg = rlg_next_ndisc(route, rlg);
		rlg_bypass(route, rlg, B_FALSE);
	}
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
	route_leg_group_t *rlg = rlg_new(ROUTE_LEG_GROUP_TYPE_PROC);
	route_leg_t *prev_rl;

	rlg->proc = proc;
	list_insert_after(&route->leg_groups, prev_rlg, rlg);
	prev_rl = last_leg_before_rlg(route, rlg);

	/* Start inserting individual procedure legs */
	for (unsigned i = 0; i < proc->num_segs; i++) {
		route_leg_t *rl = calloc(sizeof (*rl), 1);
		rl->seg = proc->segs[i];
		rl->parent = rlg;
		list_insert_tail(&rlg->legs, rl);
		list_insert_after(&route->legs, prev_rl, rl);
		prev_rl = rl;
	}
	ASSERT(!list_is_empty(&rlg->legs));
	rlg->start_fix = navproc_get_start_fix(proc);
	rlg->end_fix = *leg_get_end_fix(list_tail(&rlg->legs));

	return (rlg);
}

static const navproc_t *
find_navproc(const airport_t *arpt, navproc_type_t type, const char *procname,
    const char *tr_or_rwy)
{
	ASSERT(tr_or_rwy != NULL || type == NAVPROC_TYPE_SID_COMMON ||
	    type == NAVPROC_TYPE_STAR_COMMON || type == NAVPROC_TYPE_FINAL);
	for (unsigned i = 0; i < arpt->num_procs; i++) {
		if (arpt->procs[i].type != type ||
		    strcmp(arpt->procs[i].name, procname) != 0)
			continue;
		if ((type == NAVPROC_TYPE_SID_TRANS ||
		    type == NAVPROC_TYPE_STAR_TRANS ||
		    type == NAVPROC_TYPE_FINAL_TRANS) &&
		    strcmp(tr_or_rwy, arpt->procs[i].tr_name) != 0) {
			continue;
		} else if ((type == NAVPROC_TYPE_SID ||
		    type == NAVPROC_TYPE_STAR) &&
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
		return (ERR_INVALID_INSERT);
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
		rlg_connect_neigh(route, sid_rlg, B_TRUE);
	}
	if (sidcm != NULL) {
		sidcm_rlg = route_insert_proc_rlg(route, sidcm, sid_rlg);
		rlg_connect_neigh(route, sidcm_rlg, B_TRUE);
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
		return (ERR_INVALID_INSERT);
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
	rlg_connect_neigh(route, sidtr_rlg, B_TRUE);

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
	if (route->appr != NULL) {
		star = find_navproc(arr, NAVPROC_TYPE_STAR, star_name,
		    route->appr->rwy->ID);
	}
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
		rlg_connect_neigh(route, starcm_rlg, B_TRUE);
	}
	if (star != NULL) {
		route_leg_group_t *prev_rlg = (appr_rlg != NULL ?
		    rlg_prev_ndisc(route, appr_rlg) : rlg_tail_ndisc(route));

		star_rlg = route_insert_proc_rlg(route, star, prev_rlg);
		rlg_connect_neigh(route, star_rlg, B_TRUE);
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
		return (ERR_INVALID_INSERT);
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
	rlg_connect_neigh(route, startr_rlg, B_TRUE);

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
	rlg_connect_neigh(route, appr_rlg, B_TRUE);

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
		return (ERR_INVALID_INSERT);
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
	rlg_connect_neigh(route, apprtr_rlg, B_TRUE);

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
 * Inserts an airway leg group without a terminating fix for the moment.
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
	route_leg_group_t *rlg;
	const airway_t *awy;

	awy = airway_db_lookup(route->navdb->awydb, awyname, NULL, NULL, NULL);
	if (!awy)
		return (ERR_INVALID_AWY);
	rlg = rlg_new(ROUTE_LEG_GROUP_TYPE_AIRWAY);
	rlg->awy = awy;

	if (prev_rlg != NULL && prev_rlg->type == ROUTE_LEG_GROUP_TYPE_PROC) {
		const navproc_type_t proctype = prev_rlg->proc->type;
		ASSERT(proctype < NAVPROC_TYPE_STAR);
	}
	list_insert_after(&route->leg_groups, prev_rlg, rlg);

	rlg_update_awy_legs(route, rlg, B_FALSE);
	rlg_connect_neigh(route, rlg, B_TRUE);

	route->segs_dirty = B_TRUE;
	if (new_rlgpp)
		*new_rlgpp = rlg;

	return (ERR_OK);
}

/*
 * Sets the terminating fix of an airway leg group previously created with
 * route_lg_awy_insert.
 *
 * @param route The route holding the airway leg group.
 * @param x_rlg The airway leg group for which to set the terminating fix.
 * @param fixname The name of the terminating fix.
 *
 * @return ERR_OK on success or an error code otherwise.
 */
err_t
route_lg_awy_set_end_fix(route_t *route, const route_leg_group_t *x_rlg,
    const char *fixname)
{
	route_leg_group_t *rlg = (route_leg_group_t *)x_rlg;
	const airway_t *newawy;
	const fix_t *end_fix;

	ASSERT(rlg->type == ROUTE_LEG_GROUP_TYPE_AIRWAY);
	ASSERT(list_link_active(&rlg->route_leg_groups_node));

	if (IS_NULL_FIX(&rlg->start_fix))
		return (ERR_AWY_WPT_MISMATCH);

	newawy = airway_db_lookup(route->navdb->awydb, rlg->awy->name,
	    &rlg->start_fix, fixname, &end_fix);
	if (newawy == NULL)
		return (ERR_AWY_WPT_MISMATCH);

	rlg->awy = newawy;
	rlg->end_fix = *end_fix;

	rlg_connect_neigh(route, rlg, B_FALSE);

	rlg_update_awy_legs(route, rlg, B_FALSE);
	route->segs_dirty = B_TRUE;

	return (ERR_OK);
}

/*
 * Inserts a direct leg group to a route.
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
void
route_lg_direct_insert(route_t *route, const fix_t *fix,
    const route_leg_group_t *x_prev_rlg, const route_leg_group_t **new_rlgpp)
{
	route_leg_group_t *prev_rlg = (route_leg_group_t *)x_prev_rlg;
	route_leg_group_t *rlg;

	rlg = rlg_new(ROUTE_LEG_GROUP_TYPE_DIRECT);
	rlg->end_fix = *fix;

	list_insert_after(&route->leg_groups, prev_rlg, rlg);
	rlg_update_direct_leg(route, rlg);
	rlg_connect_neigh(route, rlg, B_TRUE);

	route->segs_dirty = B_TRUE;
	if (new_rlgpp)
		*new_rlgpp = rlg;
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
	rlg_connect(route, prev_rlg, next_rlg, allow_mod);

	return (ERR_OK);
}

/*
 * Prepends a leg ending at `fix' to an airway leg group `awyrlg'. The new
 * leg will be returned in `rlpp' (if not NULL). What this does is set the
 * start fix of the airway leg group to `fix' and regenerate its legs. It
 * then generates a new direct leg group and inserts it before `awyrlg'.
 * Finally, it connects the direct rlg to the airway and also to any rlg
 * that might have preceded it.
 */
static void
rlg_prepend_direct(route_t *route, route_leg_group_t *awyrlg, const fix_t *fix,
    const route_leg_t **rlpp)
{
	const route_leg_group_t *dirrlg;
	route_leg_group_t *prev_rlg = rlg_prev_ndisc(route, awyrlg);

	ASSERT(awyrlg->type == ROUTE_LEG_GROUP_TYPE_AIRWAY);
	awyrlg->start_fix = *fix;
	rlg_update_awy_legs(route, awyrlg, B_TRUE);
	route_lg_direct_insert(route, fix, prev_rlg, &dirrlg);
	if (rlpp)
		*rlpp = list_head(&dirrlg->legs);
}

/*
 * Appends a leg ending at `fix' to the airway `rlg' and returns the new
 * leg in `rlpp' (if not NULL). This simply sets the airway's end fix to
 * `fix' and regenerates its legs, at the end attempting to connect it to
 * the rlg following it.
 */
static void
rlg_append_direct(route_t *route, route_leg_group_t *rlg, const fix_t *fix,
    const route_leg_t **rlpp)
{
	ASSERT(rlg->type == ROUTE_LEG_GROUP_TYPE_AIRWAY ||
	    rlg->type == ROUTE_LEG_GROUP_TYPE_PROC);
	rlg->end_fix = *fix;
	rlg_update_awy_legs(route, rlg, B_TRUE);
	rlg_connect_neigh(route, rlg, B_FALSE);
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
 * is terminated at rl1's end fix, and the second airway starts at the start
 * fix of rl2. rl2 must follow rl1 on the airway leg group. If `join' is set
 * to B_TRUE and rl1 and rl2 are not immediately adjacent on the airway, the
 * two new airways are connected using a DIRECT leg group. If `join' is set
 * to B_FALSE and rl1 and rl2 are not immediately adjacent on the airway a
 * DISCO is inserted between them. The `join' argument is ignored if `rl1'
 * and `rl2' are adjacent.
 */
static void
awy_split(route_t *route, route_leg_t *rl1, route_leg_t *rl2, bool_t join)
{
	route_leg_group_t	*awy1 = rl1->parent;
	route_leg_group_t	*awy2;
	route_leg_t		*rl_last = list_prev(&awy1->legs, rl2);
	fix_t			awy1_end_fix = *leg_get_end_fix(rl1);
	fix_t			awy2_end_fix = *leg_get_end_fix(rl2);
	fix_t			awy2_start_fix = *leg_get_end_fix(rl_last);

	ASSERT(rl1->parent == rl2->parent);
	ASSERT(rl1 != rl2);

	awy2 = rlg_new(ROUTE_LEG_GROUP_TYPE_AIRWAY);
	awy2->awy = awy1->awy;
	awy2->start_fix = awy2_start_fix;
	awy2->end_fix = awy2_end_fix;

	awy1->end_fix = awy1_end_fix;

	list_insert_after(&route->leg_groups, awy1, awy2);
	rlg_update_awy_legs(route, awy1, B_FALSE);
	rlg_update_awy_legs(route, awy2, B_FALSE);

	if (!FIX_EQ(&awy1_end_fix, &awy2_start_fix)) {
		if (join) {
			route_leg_group_t *dir =
			    rlg_new(ROUTE_LEG_GROUP_TYPE_DIRECT);
			dir->start_fix = awy1->end_fix;
			dir->end_fix = awy2->start_fix;
			list_insert_after(&route->leg_groups, awy1, dir);
			rlg_update_direct_leg(route, dir);
		} else {
			rlg_new_disco(route, awy1);
		}
	}
	route->segs_dirty = B_TRUE;
}

static bool_t
leg_check_dup(const route_leg_t *rl, const fix_t *fix)
{
	return (rl != NULL && rl->seg.type != NAVPROC_SEG_TYPE_INIT_FIX &&
	    FIX_EQ_POS(leg_get_end_fix(rl), fix));
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
 *		   added leg immediately follows the airway's last fix on
 *		   the same airway, then the airway is simply extended to
 *		   end at the new leg's fix.
 *		ii) If the previous and next leg group are procedures that
 *		   belong to the same airport, then the new leg is appended
 *		   to the end of the previous leg group.
 *		iii) If the next leg group is an airway and the newly
 *		   added leg immediately precedes the airway's first fix
 *		   on the same airway, then the airway is simply extended
 *		   to start at the new leg's fix.
 *		iv) If all else fails then insert a new direct leg group
 *		   containing the new leg in between the extant leg groups.
 */
err_t
route_l_insert(route_t *route, const fix_t *fix, const route_leg_t *x_prev_rl,
    const route_leg_t **rlpp)
{
	route_leg_t *prev_rl = (route_leg_t *)x_prev_rl;
	route_leg_t *next_rl = (prev_rl != NULL ?
	    list_next(&route->legs, prev_rl) : list_head(&route->legs));

	ASSERT(!IS_NULL_FIX(fix));

		/* Check for dups */
	if (leg_check_dup(prev_rl, fix) || leg_check_dup(next_rl, fix))
		return (ERR_DUPLICATE_LEG);

	if (prev_rl != NULL && next_rl != NULL) {
		/* Both legs exist */
		route_leg_group_t *prev_rlg = prev_rl->parent;
		route_leg_group_t *next_rlg = next_rl->parent;

		if (prev_rlg != next_rlg) {
			const route_leg_group_t *rlg;
			/*
			 * Parents not shared, figure out what to do based
			 * on parent type.
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
			/* Same parent */
			ASSERT(prev_rlg->type == ROUTE_LEG_GROUP_TYPE_AIRWAY ||
			    prev_rlg->type == ROUTE_LEG_GROUP_TYPE_PROC);
			if (prev_rlg->type == ROUTE_LEG_GROUP_TYPE_AIRWAY) {
				/* Airways need to be split */
				const route_leg_group_t *rlg;
				awy_split(route, prev_rl, next_rl, B_FALSE);
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
		route_leg_group_t *prev_rlg = prev_rl->parent;
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
		route_leg_group_t *next_rlg = next_rl->parent;
		const route_leg_group_t *rlg;

		/* extend airway if the new fix immediately precedes it */
		if (next_rlg->type == ROUTE_LEG_GROUP_TYPE_AIRWAY &&
		    chk_awy_fix_adjacent(next_rlg, fix, B_TRUE)) {
			rlg_prepend_direct(route, next_rlg, fix, rlpp);
			goto out;
		/* Don't allow modifying the first departure procedure. */
		} else if (next_rlg->type == ROUTE_LEG_GROUP_TYPE_PROC &&
		    is_departure_procedure(next_rlg->proc->type)) {
			return (ERR_INVALID_INSERT);
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

	prev_rl = list_prev(&route->legs, x_target_rl);
	next_rl = (route_leg_t *)x_source_rl;

	prev_rlg = (prev_rl != NULL ? prev_rl->parent : NULL);
	next_rlg = next_rl->parent;

	/*
	 * Airways can simply get shortened, but procedures can contain
	 * non-fixed legs, so refuse to terminate on those.
	 */
	if ((prev_rl != NULL && IS_NULL_FIX(leg_get_end_fix(prev_rl)) &&
	    !prev_rl->disco) ||
	    (IS_NULL_FIX(leg_get_end_fix(next_rl)) && !next_rl->disco))
		return (ERR_INVALID_DELETE);

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
			ASSERT(prev_rlg->type != ROUTE_LEG_GROUP_TYPE_DISCO ||
			    list_next(&prev_rlg->legs, prev_rl) == NULL);
			/*
			 * Shorten airways and procedures. We already know
			 * we'll be terminating on a fix.
			 */
			if (prev_rlg->type == ROUTE_LEG_GROUP_TYPE_AIRWAY ||
			    prev_rlg->type == ROUTE_LEG_GROUP_TYPE_PROC) {
				prev_rlg->end_fix = *leg_get_end_fix(prev_rl);
				for (route_leg_t *rl =
				    list_next(&prev_rlg->legs, prev_rl); rl;
				    rl = list_next(&prev_rlg->legs, prev_rl)) {
					list_remove(&route->legs, rl);
					list_remove(&prev_rlg->legs, rl);
					free(rl);
				}
			}
		}
		/*
		 * If both sides are disco, kill one and we're done.
		 * This also prevents discos at the start of the route.
		 */
		if ((prev_rl == NULL ||
		    prev_rlg->type == ROUTE_LEG_GROUP_TYPE_DISCO) &&
		    next_rlg->type == ROUTE_LEG_GROUP_TYPE_DISCO) {
			rlg_destroy(route, next_rlg);
		}
		if (next_rlg->type == ROUTE_LEG_GROUP_TYPE_AIRWAY ||
		    next_rlg->type == ROUTE_LEG_GROUP_TYPE_DIRECT ||
		    next_rlg->type == ROUTE_LEG_GROUP_TYPE_PROC) {
			for (route_leg_t *rl = list_head(&next_rlg->legs);
			    rl != next_rl; rl = list_head(&prev_rlg->legs)) {
				ASSERT(rl != NULL);
				list_remove(&route->legs, rl);
				list_remove(&next_rlg->legs, rl);
				free(rl);
			}
			if (prev_rl != NULL)
				/* can be null_fix if prev_rl is a disco */
				next_rlg->start_fix = *leg_get_end_fix(prev_rl);
			else
				next_rlg->start_fix = null_fix;
		}
	} else {
		/* None of these can contain more than one leg */
		ASSERT(prev_rlg->type != ROUTE_LEG_GROUP_TYPE_DISCO &&
		    prev_rlg->type != ROUTE_LEG_GROUP_TYPE_DIRECT);

		if (prev_rlg->type == ROUTE_LEG_GROUP_TYPE_AIRWAY) {
			awy_split(route, prev_rl, next_rl, B_TRUE);
		} else {
			/* Procedures are simply shortened */
			for (route_leg_t *rl = list_next(&route->legs, prev_rl); rl;
			    rl = list_next(&route->legs, prev_rl)) {
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
	route_leg_group_t *rlg = rl->parent;
	route_leg_group_t *rlg2;
	route_leg_t *prev_rl = list_prev(&rlg->legs, rl);
	route_leg_t *next_rl = list_next(&rlg->legs, rl);

	switch (rlg->type) {
	case ROUTE_LEG_GROUP_TYPE_AIRWAY:
		if (prev_rl != NULL && next_rl != NULL) {
			/* Split the airway and insert a disco */
			awy_split(route, prev_rl, next_rl, B_FALSE);
		} else if (prev_rl != NULL && next_rl == NULL) {
			/* Shorten the airway from the right */
			rlg->end_fix = *leg_get_end_fix(prev_rl);
			rlg_update_awy_legs(route, rlg, B_FALSE);
			rlg2 = rlg_next_ndisc(route, rlg);
			rlg_connect(route, rlg, rlg2, B_FALSE);
		} else if (prev_rl == NULL && next_rl != NULL) {
			/* Shorten the airway from the left */
			rlg->start_fix = *leg_get_end_fix(rl);
			rlg_update_awy_legs(route, rlg, B_FALSE);
			rlg2 = rlg_prev_ndisc(route, rlg);
			rlg_connect(route, rlg2, rlg, B_FALSE);
		} else {
			/* Get rid of the whole thing */
			rlg_bypass(route, rl->parent, B_FALSE);
		}
		break;
	case ROUTE_LEG_GROUP_TYPE_PROC:
		/* We're the last one */
		if (prev_rl == NULL && next_rl == NULL) {
			rlg_bypass(route, rl->parent, B_FALSE);
		} else if (prev_rl == NULL) {
			/* First leg, check if we can adjust start_fix. */
			fix_t end_fix = *leg_get_end_fix(rl);
			list_remove(&rlg->legs, rl);
			list_remove(&route->legs, rl);
			free(rl);
			if (!IS_NULL_FIX(&end_fix)) {
				rlg->start_fix = end_fix;
				rlg_connect(route, rlg_prev_ndisc(route, rlg),
				    rlg, B_FALSE);
			}
		} else if (next_rl == NULL) {
			/* Last leg, check if we can adjust end_fix. */
			fix_t end_fix;
			ASSERT(prev_rl != NULL);
			end_fix = *leg_get_end_fix(prev_rl);
			list_remove(&rlg->legs, rl);
			list_remove(&route->legs, rl);
			free(rl);
			if (!IS_NULL_FIX(&end_fix)) {
				rlg->end_fix = end_fix;
				rlg_connect(route, rlg, rlg_next_ndisc(route,
				    rlg), B_FALSE);
			}
		} else {
			/* Internal delete, just remove it */
			list_remove(&rlg->legs, rl);
			list_remove(&route->legs, rl);
			free(rl);
		}
		break;
	case ROUTE_LEG_GROUP_TYPE_DIRECT:
		rlg_bypass(route, rl->parent, B_FALSE);
		break;
	case ROUTE_LEG_GROUP_TYPE_DISCO:
		ASSERT(list_prev(&route->legs, rl) != NULL &&
		    list_next(&route->legs, rl) != NULL);
		rlg_bypass(route, rl->parent, B_TRUE);
		break;
	default:
		assert(0);
	}
	route->segs_dirty = B_TRUE;
}
