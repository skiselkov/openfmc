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

static void
route_seg_destroy(route_seg_t *seg)
{
	ASSERT(!list_link_active(&seg->route_segs_node));
	free(seg);
}

static void
route_leg_destroy(route_leg_t *rl)
{
	ASSERT(!list_link_active(&rl->leg_group_legs_node));
	ASSERT(!list_link_active(&rl->route_legs_node));
	free(rl);
}

static void
route_leg_group_destroy(route_leg_group_t *rlg)
{
	ASSERT(list_head(&rlg->legs) == NULL);
	ASSERT(!list_link_active(&rlg->route_leg_groups_node));
	list_destroy(&rlg->legs);
	free(rlg);
}

static void
route_remove_arpt_links(route_t *route, const airport_t *arpt)
{
#define	REM_BACKREF(obj) \
	do { \
		if (route->obj != NULL && route->obj->arpt == arpt) \
			route->obj = NULL; \
	} while (0)
	/* First check all runways & procedures */
	REM_BACKREF(dep_rwy);
	REM_BACKREF(arr_rwy);
	/* Next check direct procedure links */
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
		if (rlg->type == ROUTE_LEG_GROUP_PROC &&
		    rlg->proc->arpt == arpt) {
			for (route_leg_t *rl = list_head(&rlg->legs); rl;) {
				route_leg_t *rl_next = list_next(&rlg->legs,
				    rl);
				list_remove(&rlg->legs, rl);
				list_remove(&route->legs, rl);
				route_leg_destroy(rl);
				rl = rl_next;
			}
		}
		list_remove(&route->leg_groups, rlg);
		route_leg_group_destroy(rlg);
		rlg = rlg_next;
	}

	route->dirty = B_TRUE;
}

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
		route_leg_group_destroy(rlg);
	for (route_leg_t *rl = list_head(&route->legs); rl;
	    rl = list_head(&route->legs))
		route_leg_destroy(rl);
	for (route_seg_t *rs = list_head(&route->segs); rs;
	    rs = list_head(&route->segs))
		route_seg_destroy(rs);

	list_destroy(&route->leg_groups);
	list_destroy(&route->legs);
	list_destroy(&route->segs);

	free(route);
}

static err_t
route_set_arpt_impl(route_t *route, airport_t **arptp, const char *icao)
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
	return (ERR_OK);
}

err_t
route_set_dep_arpt(route_t *route, const char *icao)
{
	return (route_set_arpt_impl(route, &route->dep, icao));
}

err_t
route_set_arr_arpt(route_t *route, const char *icao)
{
	return (route_set_arpt_impl(route, &route->arr, icao));
}

err_t
route_set_altn1_arpt(route_t *route, const char *icao)
{
	return (route_set_arpt_impl(route, &route->altn1, icao));
}

err_t
route_set_altn2_arpt(route_t *route, const char *icao)
{
	return (route_set_arpt_impl(route, &route->altn2, icao));
}
