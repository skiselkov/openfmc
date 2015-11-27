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
 * Destroys and frees a route_leg_group_t. This also remove all of the
 * leg group's associated legs. It does not handle reconnecting the adjacent
 * leg groups.
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
 * Creates a new route leg group of `type' and returns it.
 */
static route_leg_group_t *
rlg_new(route_leg_group_type_t type)
{
	route_leg_group_t *rlg = calloc(sizeof (*rlg), 1);

	rlg->type = type;
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

	/* A disco can't be first or last in the list */
	ASSERT(prev_rlg != NULL);
	ASSERT(list_tail(&route->leg_groups) != prev_rlg);
	list_insert_after(&route->leg_groups, prev_rlg, rlg);
}

/*
 * Updates the route legs of `rlg' to correspond to the rlg settings.
 * This adds/removes route legs as necessary to complete the airway.
 */
static void
rlg_update_awy_legs(route_t *route, route_leg_group_t *rlg)
{
	ASSERT(rlg->type == ROUTE_LEG_GROUP_TYPE_AIRWAY);
	UNUSED(route);
	// TODO: complete this
}

/*
 * Updates the route leg of `rlg' to correspond to the rlg settings.
 */
static void
rlg_update_direct_leg(route_t *route, route_leg_group_t *rlg)
{
	ASSERT(rlg->type == ROUTE_LEG_GROUP_TYPE_DIRECT);
	UNUSED(route);
	// TODO: complete this
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
rlg_connect(route_t *route, route_leg_group_t *prev_rlg,
    route_leg_group_t *next_rlg)
{
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

			if (FIX_EQ(&prev_rlg->end_fix, &next_rlg->start_fix))
				break;

			/* locate: AWY x AWY */
			isect = airway_db_lookup_awy_intersection(
			    route->navdb->awydb, prev_rlg->awy->name,
			    prev_rlg->start_fix.name, next_rlg->awy->name);
			if (isect == NULL)
				return (ERR_AWY_AWY_MISMATCH);
			prev_rlg->end_fix = *isect;
			next_rlg->start_fix = *isect;
			rlg_update_awy_legs(route, prev_rlg);
			rlg_update_awy_legs(route, next_rlg);
			route->segs_dirty = B_TRUE;
			break;
		}
		case ROUTE_LEG_GROUP_TYPE_DIRECT:
			/* AWY -> DIRECT */
			if (!IS_NULL_FIX(&prev_rlg->end_fix)) {
				/*
				 * If we had an endpoint, then it must not
				 * have been the same as the next DIRECT,
				 * otherwise there would have been no disco.
				 */
				ASSERT(!FIX_EQ(&prev_rlg->end_fix,
				    &next_rlg->end_fix));
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
				const airway_t *newawy = airway_db_lookup(
				    route->navdb->awydb, prev_rlg->awy->name,
				    &prev_rlg->start_fix,
				    next_rlg->end_fix.name, &newendfix);
				if (newawy == NULL ||
				    !FIX_EQ(newendfix, &next_rlg->end_fix))
					return (ERR_AWY_WPT_MISMATCH);
				prev_rlg->awy = newawy;
				prev_rlg->end_fix = next_rlg->end_fix;
				rlg_update_awy_legs(route, prev_rlg);
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
			newawy = airway_db_lookup(route->navdb->awydb,
			    prev_rlg->awy->name, &prev_rlg->start_fix,
			    next_rlg->start_fix.name, NULL);
			if (newawy == NULL)
				return (ERR_AWY_PROC_MISMATCH);
			prev_rlg->awy = newawy;
			prev_rlg->end_fix = next_rlg->start_fix;
			rlg_update_awy_legs(route, prev_rlg);
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
			const airway_t *newawy = airway_db_lookup(
			    route->navdb->awydb, next_rlg->awy->name,
			    &prev_rlg->end_fix,
			    IS_NULL_FIX(&next_rlg->end_fix) ? NULL :
			    next_rlg->end_fix.name, &newendfix);

			if (newawy != NULL &&
			    (IS_NULL_FIX(&next_rlg->end_fix) ||
			    FIX_EQ(&next_rlg->end_fix, newendfix))) {
				next_rlg->awy = newawy;
				next_rlg->start_fix = prev_rlg->end_fix;
				rlg_update_awy_legs(route, next_rlg);

				route->segs_dirty = B_TRUE;
			} else {
				return (ERR_AWY_WPT_MISMATCH);
			}
			break;
		}
		case ROUTE_LEG_GROUP_TYPE_DIRECT:
			/* [DIRECT|PROC] -> DIRECT */
			ASSERT(!IS_NULL_FIX(&prev_rlg->end_fix));
			next_rlg->start_fix = prev_rlg->end_fix;
			break;
		case ROUTE_LEG_GROUP_TYPE_PROC:
			/* [DIRECT|PROC] -> PROC */
			if (!FIX_EQ(&prev_rlg->end_fix, &next_rlg->start_fix))
				return (ERR_WPT_PROC_MISMATCH);
			break;
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
	if (list_next(&route->leg_groups, prev_rlg) == next_rlg)
		return;

	for (route_leg_group_t *rlg = list_next(&route->leg_groups, prev_rlg);
	    rlg != next_rlg; rlg = list_next(&route->leg_groups, prev_rlg)) {
		ASSERT(rlg != NULL);
		rlg_destroy(route, rlg);
	}
	route->segs_dirty = B_TRUE;
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
rlg_force_connect(route_t *route, route_leg_group_t *prev_rlg,
    route_leg_group_t *next_rlg)
{
	if (rlg_connect(route, prev_rlg, next_rlg) == ERR_OK) {
		/* Connection succeeded, delete any intervening legs */
		rlg_bring_together(route, prev_rlg, next_rlg);
	} else if (!only_disco_between(route, prev_rlg, next_rlg)) {
		/* Connection failed and we need to insert new disco */
		rlg_bring_together(route, prev_rlg, next_rlg);
		rlg_new_disco(route, prev_rlg);
		route->segs_dirty = B_TRUE;
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
	route_leg_group_t *rlg, *next_rlg;
	const airway_t *awy;

	awy = airway_db_lookup(route->navdb->awydb, awyname, NULL, NULL, NULL);
	if (!awy)
		return (ERR_AWY_NOT_FOUND);
	rlg = rlg_new(ROUTE_LEG_GROUP_TYPE_AIRWAY);
	rlg->awy = awy;

	if (prev_rlg != NULL)
		list_insert_after(&route->leg_groups, prev_rlg, rlg);
	else
		list_insert_head(&route->leg_groups, rlg);

	next_rlg = list_next(&route->leg_groups, rlg);
	if (prev_rlg != NULL)
		rlg_force_connect(route, prev_rlg, rlg);
	if (next_rlg != NULL)
		rlg_force_connect(route, rlg, next_rlg);

	rlg_update_awy_legs(route, rlg);
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
	route_leg_group_t *next_rlg;

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

	next_rlg = list_next(&route->leg_groups, rlg);
	if (next_rlg != NULL)
		rlg_force_connect(route, rlg, next_rlg);

	rlg_update_awy_legs(route, rlg);
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
	route_leg_group_t *rlg, *next_rlg;

	rlg = rlg_new(ROUTE_LEG_GROUP_TYPE_DIRECT);
	rlg->end_fix = *fix;

	if (prev_rlg != NULL)
		list_insert_after(&route->leg_groups, prev_rlg, rlg);
	else
		list_insert_head(&route->leg_groups, rlg);

	next_rlg = list_next(&route->leg_groups, rlg);
	if (prev_rlg != NULL)
		rlg_force_connect(route, prev_rlg, rlg);
	if (next_rlg != NULL)
		rlg_force_connect(route, rlg, next_rlg);

	rlg_update_direct_leg(route, rlg);
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
	route_leg_group_t *prev_rlg = list_prev(&route->leg_groups, rlg);
	route_leg_group_t *next_rlg = list_next(&route->leg_groups, rlg);

	/*
	 * Procedures can't be deleted this way, use the appropriate
	 * procedure manipulation functions.
	 */
	if (rlg->type == ROUTE_LEG_GROUP_TYPE_PROC)
		return (ERR_INVALID_DELETE);

	if (prev_rlg != NULL && next_rlg != NULL)
		rlg_force_connect(route, prev_rlg, next_rlg);

	rlg_destroy(route, rlg);

	return (ERR_OK);
}
