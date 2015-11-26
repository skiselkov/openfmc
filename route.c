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
 * Destroys and frees a route_leg_group_t.
 */
static void
rlg_destroy(route_t *route, route_leg_group_t *rlg)
{
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
			rlg_destroy(route, rlg);
		}
		rlg = rlg_next;
	}

	route->segs_dirty = B_TRUE;
}

static err_t
rlg_connect(route_t *route, route_leg_group_t *prev_rlg,
    route_leg_group_t *next_rlg)
{
	ASSERT(prev_rlg != NULL);
	ASSERT(next_rlg != NULL);

	switch (prev_rlg->type) {
	case ROUTE_LEG_GROUP_TYPE_AIRWAY:
		/* AWY -> [something] */
		switch (next_rlg->type) {
		case ROUTE_LEG_GROUP_TYPE_AIRWAY: {
			/* AWY -> AWY */
			const fix_t *isect;

			if (FIX_EQ(&prev_rlg->end_fix, &next_rlg->start_fix)
				break;

			/* locate: AWY x AWY */
			isect = airway_db_lookup_awy_intersection(route->navdb,
			    prev_rlg->awy->name, prev_rlg->start_fix.name,
			    next_rlg->awy->name);
			if (isect == NULL)
				return (ERR_AWY_AWY_MISMATCH);
			prev_rlg->end_fix = *isect;
			next_rlg->start_fix = *isect;
			update_awy_rlg_legs(route, prev_rlg);
			update_awy_rlg_legs(route, next_rlg);
			route->segs_dirty = B_TRUE;
			break;
		}
		case ROUTE_LEG_GROUP_TYPE_DIRECT:
			/* AWY -> DIRECT */
			if (!IS_NULL_FIX(&prev_rlg->end_fix) {
				/*
				 * If we had an endpoint, then it must not
				 * have been the same as the next DIRECT,
				 * otherwise there would have been no disco.
				 */
				ASSERT(!FIX_EQ(&prev_rlg->end_fix,
				    &next_rlg->end_fix));
				next_rlg->start_fix = prev_rlg->end_fix;

				route->segs_dirty = B_TRUE;
			} else {
				/*
				 * Preceding airway is missing an endpoint,
				 * so try to locate the next DIRECT on the
				 * airway and remove the DIRECT leg group.
				 */
				const airway_t *newawy;

				airway_db_lookup(route->navdb->awydb,
				    prev_rlg->awy->name,
				    prev_rlg->start_fix.name,
				    next_rlg->end_fix.name, &newawy, NULL);
				if (newawy == NULL)
					return (ERR_AWY_WPT_MISMATCH);
				prev_rlg->awy = newawy;
				prev_rlg->end_fix = next_rlg->end_fix;
				update_awy_rlg_legs(route, prev_rlg);
				rlg_destroy(next_rlg);

				route->segs_dirty = B_TRUE;
			}
			break;
		case ROUTE_LEG_GROUP_TYPE_PROC: {
			/* AWY -> PROC */
			const airway_t *newawy;

			ASSERT(!IS_NULL_FIX(&next_rlg->start_fix));
			/* lack of start_fix means we don't know where to go */
			if (IS_NULL_FIX(&prev_rlg->start_fix))
				return (ERR_AWY_WPT_MISMATCH);
			airway_db_lookup(route->navdb,
			    prev_rlg->awy->name, prev_rlg->start_fix.name,
			    next_rlg->start_fix.name, &newawy, NULL);
			if (nawy == NULL)
				return (ERR_AWY_PROC_MISMATCH);
			prev_rlg->awy = nawy;
			prev_rlg->end_fix = next_rlg->start_fix;
			update_awy_rlg_legs(route, prev_rlg);
			route->segs_dirty = B_TRUE;
			break;
		}
		case ROUTE_LEG_GROUP_TYPE_DISCO:
			/* AWY -> (disco) */
			break;
		default:
			assert(0);
		}
		break;
	case ROUTE_LEG_GROUP_TYPE_DIRECT:
		/* DIRECT -> [something] */
		switch (next_rlg->type) {
		case ROUTE_LEG_GROUP_TYPE_AIRWAY: {
			/* DIRECT -> AWY */
			const airway_t *newawy;

			airway_db_lookup(route->navdb, next_rlg->awy->name,
			    prev_rlg->start_fix.name, next_rlg->end_fix.name,
			    &newawy, NULL);
			if (newawy != NULL) {
				next_rlg->awy = newawy;
				next_rlg->start_fix = prev_rlg->end_fix;
				update_awy_rlg(route, next_rlg);

				route->segs_dirty = B_TRUE;
			} else {
				return (ERR_AWY_WPT_MISMATCH);
			}
			break;
		}
		case ROUTE_LEG_GROUP_TYPE_DIRECT:
			/* DIRECT -> DIRECT */
			ASSERT(!IS_NULL_FIX(&prev_rlg->end_fix));
			next_rlg->start_fix = prev_rlg->end_fix;
			break;
		case ROUTE_LEG_GROUP_TYPE_PROC:
			/* DIRECT -> PROC */
			if (!FIX_EQ(&prev_rlg->end_fix, &next_rlg->start_fix))
				return (ERR_WPT_PROC_MISMATCH);
			break;
		case ROUTE_LEG_GROUP_TYPE_DISCO:
			/* DIRECT -> (disco) */
			break;
		default:
			assert(0);
		}
		break;
	case ROUTE_LEG_GROUP_TYPE_PROC:
		/* PROC -> [something] */
		switch (next_rlg->type) {
		case ROUTE_LEG_GROUP_TYPE_AIRWAY: {
			/* PROC -> AWY */
			const airway_t *newawy;

			airway_db_lookup(route->navdb, next_rlg->awy->name,
			    prev_rlg->end_fix.name, next_rlg->end_fix.name,
			    &newawy, NULL);
			if (newawy != NULL) {
				next_rlg->awy = newawy;
				next_rlg->start_fix = prev_rlg->end_fix;
				update_awy_rlg_legs(route, next_rlg);

				route->segs_dirty = B_TRUE;
			} else {
				return (ERR_AWY_WPT_MISMATCH);
			}
			break;
		}
		case ROUTE_LEG_GROUP_TYPE_DIRECT:
			/* PROC -> DIRECT */
			next_rlg->start_fix = prev_rlg->end_fix;
			break;
		case ROUTE_LEG_GROUP_TYPE_PROC:
			/* PROC -> PROC */
			if (!FIX_EQ(&prev_rlg->end_fix, &next_rlg->start_fix))
				return (ERR_PROC_PROC_MISMATCH);
			break;
		case ROUTE_LEG_GROUP_TYPE_DISCO:
			/* PROC -> (disco) */
			break;
		default:
			assert(0);
		}
		break;
	case ROUTE_LEG_GROUP_TYPE_DISCO:
		VERIFY(next_rlg->type != ROUTE_LEG_GROUP_TYPE_DISCO);
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
disco_between(route_t *route, route_leg_group_t *rlg1, route_leg_group_t *rlg2)
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

static route_leg_group_t *
rlg_new(route_leg_group_type_t type)
{
	route_leg_group_t *rlg = calloc(sizeof (*rlg), 1);

	rlg->type = type;
	list_create(&rlg->legs, sizeof (route_leg_t),
	    offsetof(route_leg_t, leg_group_legs_node));

	return (rlg);
}

static void
rlg_new_disco(route_t *route, route_leg_group_t *prev_rlg)
{
	route_leg_group_t *rlg = rlg_new(ROUTE_LEG_GROUP_TYPE_DISCO);

	/* A disco can't be first or last in the list */
	ASSERT(prev_rlg != NULL);
	ASSERT(list_tail(&route->leg_groups) != prev_rlg);
	list_insert_after(&route->leg_groups, prev_rlg, rlg);
}

static void
rlg_connect_force(route_t *route, route_leg_group_t *prev_rlg,
    route_leg_group_t *next_rlg)
{
	if (rlg_connect(route, prev_rlg, next_rlg) != ERR_OK) {
		route_leg_group_t *rlg;

		/* connection not possible, insert disco */
		if (disco_between(route, prev_rlg, next_rlg))
			/* disco already there, no need to change it */
			return;

		/* discard all rlg's between these two */
		for (rlg = list_next(&route->leg_groups, prev_rlg);
		    rlg != next_rlg;
		    rlg = list_next(&route->leg_groups, prev_rlg)) {
			ASSERT(rlg != NULL);
			rlg_destroy(rlg);
		}

		rlg_new_disco(route, prev_rlg);
	}
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
		rlg_destroy(rlg);
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
 * to it from the route) and replaces it with a newly opened one.
 */
static err_t
route_set_arpt(route_t *route, airport_t **arptp, const char *icao)
{
	airport_t *narpt;

	/* Don't replace the same airport - just return OK */
	if (*arptp != NULL && strcmp((*arptp)->icao, icao) == 0)
		return (ERR_OK);

	/* Try to open new airport */
	narpt = airport_open(icao, route->navdb->navdata_dir,
	    route->navdb->wptdb, route->navdb->navaiddb);
	if (narpt == NULL)
		return (ERR_ARPT_NOT_FOUND);

	/* Replace the old one */
	if (*arptp != NULL) {
		route_remove_arpt_links(route, *arptp);
		airport_close(*arptp);
	}
	*arptp = narpt;
	route->segs_dirty = B_TRUE;
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
 * Returns the leg groups which constitute the route. This is a read-only
 * list. To edit the route, use the route editing functions.
 */
const list_t *
route_leg_groups(const route_t *route)
{
	return (&route->leg_groups);
}

/*
 * Returns the individual legs which constitute the route. This is a
 * read-only list. To edit the route, use the route editing functions.
 */
const list_t *
route_legs(const route_t *route)
{
	return (&route->legs);
}

const route_leg_group_t *route_lg_awy_insert(route_t *route,
    const char *awyname, const route_leg_group_t *prev_rlg)
{
	route_leg_group_t *rlg;

	rlg = calloc(sizeof (*rlg), 1);
	if (prev_rlg != NULL) {
		route_leg_group_t *next_rlg = list_next(&route->leg_groups,
		    prev_rlg);

		switch (prev_rlg->type) {
		case ROUTE_LEG_GROUP_TYPE_AIRWAY:
			break;
		case ROUTE_LEG_GROUP_TYPE_DIRECT:
		case ROUTE_LEG_GROUP_TYPE_PROC:
			break;
		case ROUTE_LEG_GROUP_TYPE_DISCO:
			break;
		}
	}
}

err_t
route_lg_awy_set_end_fix(route_t *route, const route_leg_group_t *rlg_const,
    const char *fixname)
{
	route_leg_group_t *rlg = (route_leg_group_t *)rlg_const;
	const airway_t *newawy;
	const fix_t *end_fix;
	route_leg_group_t *next_rlg;

	ASSERT(rlg->type == ROUTE_LEG_GROUP_TYPE_AIRWAY);
	if (IS_NULL_FIX(rlg->start_fix))
		return (ERR_AWY_WPT_MISMATCH);

	airway_db_lookup(route->navdb->awydb, rlg->awy->name, &rlg->start_fix,
	    fixname, &newawy, &end_fix);
	if (newawy == NULL)
		return (ERR_AWY_WPT_MISMATCH);

	rlg->awy = newawy;
	rlg->end_fix = *end_fix;
	update_awy_rlg_legs(route, rlg);

	next_rlg = list_next(&route->leg_groups, rlg);
	if (next_rlg != NULL)
		rlg_connect_force(route, rlg, next_rlg);

	route->segs_dirty = B_TRUE;
}

err_t
route_lg_delete(route_t *route, const route_leg_group_t *rlg)
{
	const route_leg_group_t *prev_rlg = list_prev(&route->leg_groups, rlg);
	const route_leg_group_t *next_rlg = list_next(&route->leg_groups, rlg);
	err_t err;

	/*
	 * Procedures can't be deleted this way, use the appropriate
	 * procedure manipulation functions.
	 */
	if (rlg->type == ROUTE_LEG_GROUP_TYPE_PROC)
		return (ERR_INVALID_DELETE);

	/*
	 * Try to remove the disco - only valid if the prev leg group
	 * has an end_fix set.
	 */
	switch (rlg->type) {
	case ROUTE_LEG_GROUP_TYPE_DISCO:
		err = route_lg_delete_disco(route, rlg, prev_rlg, next_rlg);
		break;
	}
	if (err != ERR_OK)
		return (err);

	list_remove(&route->leg_groups, rlg);
	rlg_destroy(rlg);
}
