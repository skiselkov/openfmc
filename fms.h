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

#ifndef	_OPENFMC_FMS_H_
#define	_OPENFMC_FMS_H_

#include "airac.h"
#include "wmm.h"

typedef struct {
	time_t		valid_from;
	time_t		valid_to;
	unsigned	airac_cycle;

	char		*navdata_dir;
	airway_db_t	*awydb;
	waypoint_db_t	*wptdb;
	navaid_db_t	*navaiddb;

	char		*wmm_file;
	wmm_t		*wmm;
} fms_navdb_t;

fms_navdb_t *navdb_open(const char *navdata_dir, const char *wmm_file);
void navdb_close(fms_navdb_t *navdb);

bool_t navdb_is_current(const fms_navdb_t *navdb);
bool_t navdata_is_current(const char *navdata_dir);

#endif	/* _OPENFMC_FMS_H_ */
