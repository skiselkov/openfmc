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

#ifndef	_OPENFMC_ROUTE_H_
#define	_OPENFMC_ROUTE_H_

#include "helpers.h"
#include "list.h"
#include "fms.h"
#include "airac.h"
#include "err.h"

typedef enum {
	ROUTE_LEG_GROUP_AIRWAY,
	ROUTE_LEG_GROUP_DIRECT,
	ROUTE_LEG_GROUP_PROC
} route_leg_group_type_t;

/*
 * Leg groups are very high level route elements that encapsulate several
 * route legs, an entire procedure or just even a single DIRECT-TO leg. This
 * corresponds to the RTE page in a typical Boeing FMC.
 */
typedef struct {
	route_leg_group_type_t	type;
	union {
		airway_t	*awy;
		navproc_t	*proc;
	};
	fix_t			end_fix;
	list_t			legs;

	list_node_t		route_leg_groups_node;
} route_leg_group_t;

typedef struct {
	bool_t		disco;
	navproc_seg_t		seg;
	alt_lim_t		alt_lim;
	spd_lim_t		spd_lim;

	list_node_t		leg_group_legs_node;
	list_node_t		route_legs_node;
} route_leg_t;

/*
 * Route segments are actual individual pieces of a route that consititute
 * a single maneuver.
 */
typedef enum {
	ROUTE_SEG_TYPE_STRAIGHT,
	ROUTE_SEG_TYPE_ARC,
	ROUTE_SEG_TYPE_DEP_RWY,
	ROUTE_SEG_TYPES
} route_seg_type_t;

typedef struct {
	route_seg_type_t		type;
	union {
		struct {
			geo_pos3_t	start;
			geo_pos3_t	end;
		} straight;
		struct {
			geo_pos3_t	start;
			geo_pos3_t	end;
			geo_pos2_t	center;
		} arc;
	};

	list_node_t			route_segs_node;
} route_seg_t;

typedef struct {
	const fms_navdb_t	*navdb;

	airport_t		*dep;
	airport_t		*arr;
	airport_t		*altn1;
	airport_t		*altn2;

	/*
	 * Departure procedure related info:
	 *	1) dep_rwy: The route starts here with a special conditional
	 *	   segment that gets auto-placed on wheels-off. Before that
	 *	   it is calculated from the runway threshold.
	 *	2) sid: Initial runway-specific portion of SID, leading up
	 *	   to the common SID/SID transition/enroute structure.
	 *	3) sidcm: Common SID segment that links the runway-specific
	 *	   SID portion to the SID transition or enroute structure.
	 *	4) sidtr: SID transition from SID/SID common segment to the
	 *	   enroute structure.
	 */
	runway_t		*dep_rwy;
	navproc_t		*sid;
	navproc_t		*sidcm;
	navproc_t		*sidtr;

	/*
	 * Arrival procedure related info (in reverse order to departure):
	 *	1) startr: STAR transition from enroute structure to STAR/
	 *	   common STAR segment.
	 *	2) startcm: Common STAR segment that links the enroute
	 *	   structure or STAR transition to a STAR.
	 *	3) star: Specific STAR.
	 *	4) apprtr: Approach transition from STAR to approach proc.
	 *	5) appr: Approach procedure to specific runway.
	 *	6) arr_rwy: Target arrival runway.
	 * Arrival procedures can be selected from either the `arr', `altn1'
	 * or `altn2' airports.
	 */
	navproc_t		*startr;
	navproc_t		*starcm;
	navproc_t		*star;
	navproc_t		*apprtr;
	navproc_t		*appr;
	runway_t		*arr_rwy;

	list_t			leg_groups;
	list_t			legs;

	bool_t			dirty;
	list_t			segs;
	route_seg_t		*active_seg;
} route_t;

route_t *route_create(const fms_navdb_t *navdb);
void route_destroy(route_t *route);

err_t route_set_dep_arpt(route_t *route, const char *icao);
err_t route_set_arr_arpt(route_t *route, const char *icao);
err_t route_set_altn1_arpt(route_t *route, const char *icao);
err_t route_set_altn2_arpt(route_t *route, const char *icao);

#endif	/* _OPENFMC_ROUTE_H_ */
