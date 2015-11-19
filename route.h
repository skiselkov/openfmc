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

#include "list.h"
#include "airac.h"

typedef enum {
	ROUTE_LEG_GROUP_AIRWAY,
	ROUTE_LEG_GROUP_DIRECT,
	ROUTE_LEG_GROUP_PROC
} route_leg_group_type_t;

typedef struct {
	route_leg_group_type_t	type;
	union {
		airway_t	*awy;
		navproc_t	*proc;
	};
	fix_t			end_fix;
	list_t			legs;
} route_leg_group_t;

typedef struct {
	navproc_seg_t		seg;
	alt_lim_t		alt_lim;
	spd_lim_t		spd_lim;
	list_node_t		leg_group_node;
	list_node_t		legs_node;
} route_leg_t;

typedef struct {
	route_seg_type_t	type;
	list_node_t		segs_node;
} route_seg_t;

typedef struct {
	airport_t	*dep;
	airport_t	*arr;
	airport_t	*altn1;
	airport_t	*altn2;

	list_t		leg_groups;
	list_t		legs;
	list_t		segs;
	route_seg_t	*active_seg;
} route_t;

#endif	/* _OPENFMC_ROUTE_H_ */
