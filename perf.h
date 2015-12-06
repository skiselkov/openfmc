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

#ifndef	_OPENFMC_PERF_H_
#define	_OPENFMC_PERF_H_

#include "geom.h"

#ifdef	__cplusplus
extern "C" {
#endif

typedef struct {
	int	spd;
	double	Cd;
} drag_coeff_t;

typedef struct {
	char	*acft_type;
	char	*eng_type;

	/* max thrust in Newtons, 15C @ 1013 hPa (SL ISA) */
	double	eng_max_thr;

	/* max thrust fraction (y) depending on alt (x) in thousands of feet */
	size_t	n_thr_alt_curve_pts;
	vect2_t	*thr_alt_curve_pts;

	/* max thrust fraction (y) depending on temp deviation (x) in C */
	size_t	n_thr_temp_curve_pts;
	vect2_t	*thr_temp_curve_pts;
} acft_perf_t;

typedef struct {
	double	crz_lvl;
	double	crz_tas;
	double	thr_derate;	/* fraction of eng_max_thr */
} flt_perf_t;

acft_perf_t *parse_acft_perf(const char *filename);
void acft_perf_destroy(acft_perf_t *perf);

double eng_max_thr_avg(const flt_perf_t *flt, acft_perf_t *acft,
    double alt1, double temp1, double alt2, double temp2, double tp_alt);

#ifdef	__cplusplus
}
#endif

#endif	/* _OPENFMC_PERF_H_ */
