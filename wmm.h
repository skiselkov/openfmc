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

#ifndef	_OPENFMC_WMM_H_
#define	_OPENFMC_WMM_H_

#include <math.h>

#include "geom.h"

#ifdef	__cplusplus
extern "C" {
#endif

typedef struct wmm_s wmm_t;

wmm_t *wmm_open(const char *filename, double year);
void wmm_close(wmm_t *wmm);

double wmm_get_start(const wmm_t *wmm);
double wmm_get_end(const wmm_t *wmm);

double wmm_mag2true(const wmm_t *wmm, double m, geo_pos3_t pos);
double wmm_true2mag(const wmm_t *wmm, double t, geo_pos3_t pos);

#ifdef	__cplusplus
}
#endif

#endif	/* _OPENFMC_WMM_H_ */
