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

#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <string.h>

#include "log.h"
#include "helpers.h"
#include "perf.h"

/*
 * Interpolates a linear trend defined at reference points x0 and x1 by
 * values y0 and y1 to point 'x'.
 */
static inline double
interpolate_linear(double x0, double y0, double x1, double y1, double x)
{
	return (y0 + ((x - x0) * ((y1 - y0) / (x1 - x0))));
}

static double
thr_temp_coeff(double isa_dev, const acft_perf_t *acft)
{
	ASSERT(acft->n_thr_temp_curve_pts >= 2);

	for (size_t i = 0; i < acft->n_thr_temp_curve_pts; i++) {
		vect2_t p0, p1;

		if (i + 1 == acft->n_thr_temp_curve_pts) {
			/* extrapolate linearly from last segment */
			p0 = acft->thr_temp_curve_pts[i - 1];
			p1 = acft->thr_temp_curve_pts[i];
		} else if (isa_dev < acft->thr_temp_curve_pts[i + 1].x) {
			p0 = acft->thr_temp_curve_pts[i];
			p1 = acft->thr_temp_curve_pts[i + 1];
		} else {
			continue;
		}
		return (interpolate_linear(p0.x, p0.y, p1.x, p1.y, isa_dev));
	}
	assert(0);
}

static inline double
calc_isa_dev(double alt, double temp)
{
#define	ISA_TEMP		15.0	/* ISA temp at SL in degC */
#define	ISA_ELR_PER_1000	1.98	/* Environmental lapse rate */
	return (temp - (ISA_TEMP - ((alt / 1000.0) * ISA_ELR_PER_1000)));
}

static bool_t
parse_curves(FILE *fp, size_t *num, vect2_t **curvep, size_t numpoints,
    size_t *line_num)
{
	vect2_t	*curve;
	char	*line = NULL;
	size_t	line_cap = 0;
	ssize_t	line_len = 0;

	ASSERT(*curvep == NULL);
	curve = calloc(sizeof (vect2_t), numpoints);

	for (size_t i = 0; i < numpoints; i++) {
		char *comps[2];

		line_len = parser_get_next_line(fp, &line, &line_cap, line_num);
		if (line_len <= 0)
			goto errout;
		if (explode_line(line, ',', comps, 2) != 2)
			goto errout;
		curve[i] = VECT2(atof(comps[0]), atof(comps[1]));
		if (i > 0 && curve[i - 1].x >= curve[i].x)
			goto errout;
	}

	*curvep = curve;
	*num = numpoints;
	free(line);

	return (B_TRUE);
errout:
	free(line);
	free(curve);
	return (B_FALSE);
}

acft_perf_t *
parse_acft_perf(const char *filename)
{
#define	ACFT_PERF_MIN_VERSION	1
#define	ACFT_PERF_MAX_VERSION	1
#define	MAX_LINE_COMPS		10
	acft_perf_t	*acft = calloc(sizeof (*acft), 1);
	FILE		*fp = fopen(filename, "r");
	char		*line = NULL;
	size_t		line_cap = 0, line_num = 0;
	ssize_t		line_len = 0;
	char		*comps[MAX_LINE_COMPS];
	bool_t		version_check_completed = B_FALSE;

	if (fp == NULL)
		goto errout;
	while ((line_len = parser_get_next_line(fp, &line, &line_cap,
	    &line_num)) != -1) {
		ssize_t ncomps = explode_line(line, ',', comps, MAX_LINE_COMPS);

		if (ncomps < 0) {
			openfmc_log(OPENFMC_LOG_ERR, "Error parsing acft "
			    "perf file %s:%lu: malformed line, too many "
			    "line components.", filename, line_num);
			goto errout;
		}
		ASSERT(ncomps > 0);
		if (strcmp(comps[0], "VERSION") == 0) {
			int vers;

			if (version_check_completed) {
				openfmc_log(OPENFMC_LOG_ERR, "Error parsing "
				    "acft perf file %s:%lu: duplicate VERSION "
				    "line.", filename, line_num);
				goto errout;
			}
			if (ncomps != 2) {
				openfmc_log(OPENFMC_LOG_ERR, "Error parsing "
				    "acft perf file %s:%lu: malformed VERSION "
				    "line.", filename, line_num);
				goto errout;
			}
			vers = atoi(comps[1]);
			if (vers < ACFT_PERF_MIN_VERSION ||
			    vers > ACFT_PERF_MAX_VERSION) {
				openfmc_log(OPENFMC_LOG_ERR, "Error parsing "
				    "acft perf file %s:%lu: unsupported file "
				    "version %d.", filename, line_num, vers);
				goto errout;
			}
			version_check_completed = B_TRUE;
		}
		if (!version_check_completed) {
			openfmc_log(OPENFMC_LOG_ERR, "Error parsing acft "
			    "perf file %s:%lu: first line was not VERSION.",
			    filename, line_num);
			goto errout;
		}
		if (strcmp(comps[0], "ACFTTYPE") == 0) {
			if (ncomps != 2 || acft->acft_type != NULL) {
				openfmc_log(OPENFMC_LOG_ERR, "Error parsing "
				    "acft perf file %s:%lu: malformed or "
				    "duplicate ACFTTYPE line.", filename,
				    line_num);
				goto errout;
			}
			acft->acft_type = strdup(comps[1]);
		} else if (strcmp(comps[0], "ENGTYPE") == 0) {
			if (ncomps != 2 || acft->eng_type != NULL) {
				openfmc_log(OPENFMC_LOG_ERR, "Error parsing "
				    "acft perf file %s:%lu: malformed or "
				    "duplicate ENGTYPE line.", filename,
				    line_num);
				goto errout;
			}
			acft->eng_type = strdup(comps[1]);
		} else if (strcmp(comps[0], "MAXTHR") == 0) {
			if (ncomps != 2 || atof(comps[1]) <= 0.0 ||
			    acft->eng_max_thr != 0.0) {
				openfmc_log(OPENFMC_LOG_ERR, "Error parsing "
				    "acft perf file %s:%lu: malformed or "
				    "duplicate MAXTHR line.", filename,
				    line_num);
				goto errout;
			}
			acft->eng_max_thr = atof(comps[1]);
		} else if (strcmp(comps[0], "THRALT") == 0) {
			if (ncomps != 2 || atoi(comps[1]) < 2 ||
			    acft->thr_alt_curve_pts != NULL) {
				openfmc_log(OPENFMC_LOG_ERR, "Error parsing "
				    "acft perf file %s:%lu: malformed or "
				    "duplicate THRALT line.", filename,
				    line_num);
				goto errout;
			}
			if (!parse_curves(fp, &acft->n_thr_alt_curve_pts,
			    &acft->thr_alt_curve_pts, atoi(comps[1]),
			    &line_num)) {
				openfmc_log(OPENFMC_LOG_ERR, "Error parsing "
				    "acft perf file %s:%lu: malformed or "
				    "missing lines.", filename, line_num);
				goto errout;
			}
		}
	}

	fclose(fp);
	free(line);

	return (acft);
errout:
	if (fp)
		fclose(fp);
	if (acft)
		acft_perf_destroy(acft);
	free(line);
	return (NULL);
}

void
acft_perf_destroy(acft_perf_t *acft)
{
	if (acft->acft_type)
		free(acft->acft_type);
	if (acft->eng_type)
		free(acft->eng_type);
	if (acft->thr_alt_curve_pts)
		free(acft->thr_alt_curve_pts);
	if (acft->thr_temp_curve_pts)
		free(acft->thr_temp_curve_pts);
	free(acft);
}

double
eng_max_thr_avg(const flt_perf_t *flt, acft_perf_t *acft,
    double alt1, double temp1, double alt2, double temp2, double tp_alt)
{
	double	thr_sum = 0;

	ASSERT(alt1 <= alt2);
	alt1 /= 1000.0;
	alt2 /= 1000.0;
	tp_alt /= 1000.0;

	for (size_t i = 0; i < acft->n_thr_alt_curve_pts; i++) {
		vect2_t p0, p1;
		double lower, upper;
		double val;

		p0 = acft->thr_alt_curve_pts[i];
		p1 = (i + 1 < acft->n_thr_alt_curve_pts ?
		    acft->thr_alt_curve_pts[i + 1] :
		    acft->thr_alt_curve_pts[i]);
		lower = MAX(alt1, p0.x);
		upper = MIN(alt2, p1.x);
		if (upper < lower)
			continue;
		if (i + 1 == acft->n_thr_alt_curve_pts)
			upper = alt2;
		val = ((p0.y + p1.y) / 2) * (upper - lower);

		if (lower >= tp_alt) {
			val *= thr_temp_coeff(calc_isa_dev(tp_alt, temp2),
			    acft);
		} else {
			double temp_alt = AVG(lower, MIN(upper, tp_alt));
			double loc_temp = interpolate_linear(alt1, temp1,
			    alt2, temp2, temp_alt);
			val *= thr_temp_coeff(calc_isa_dev(loc_temp,
			    temp_alt), acft);
		}
		thr_sum += val;
	}

	return (acft->eng_max_thr * flt->thr_derate *
	    (thr_sum / (alt2 - alt1)));
}
