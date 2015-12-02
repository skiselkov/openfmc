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

#include "wmm.h"
#include "helpers.h"
#include "GeomagnetismHeader.h"

struct wmm_s {
	/* time-modified model according to year passed to wmm_open */
	MAGtype_MagneticModel	*model;
	MAGtype_Ellipsoid	ellip;
};

static void
wmm_set_earth_ellip(MAGtype_Ellipsoid *ellip)
{
	/* Sets WGS-84 parameters */
	/* semi-major axis of the ellipsoid in meters */
	ellip->a = 6378.137;
	/* semi-minor axis of the ellipsoid in meters */
	ellip->b = 6356.7523142;
	/* flattening */
	ellip->fla = 1 / 298.257223563;
	/* first eccentricity */
	ellip->eps = sqrt(1 - (ellip->b * ellip->b) / (ellip->a * ellip->a));
	/*first eccentricity squared */
	ellip->epssq = (ellip->eps * ellip->eps);
	/* Earth's radius */
	ellip->re = 6371.2;
}

wmm_t *
wmm_open(const char *filename, double year)
{
	wmm_t			*wmm;
	int			n_max, n_terms;
	MAGtype_Date		date = { .DecimalYear = year };
	MAGtype_MagneticModel	*fixed_model;

	if (!MAG_robustReadMagModels(filename, &fixed_model))
		return (NULL);
	if (year < fixed_model->epoch || year > fixed_model->epoch + 5) {
		/* Magnetic model not applicable */
		MAG_FreeMagneticModelMemory(fixed_model);
		return (NULL);
	}

	n_max = MAX(fixed_model->nMax, 0);
	n_terms = ((n_max + 1) * (n_max + 2) / 2);

	wmm = calloc(sizeof (*wmm), 1);
	wmm->model = MAG_AllocateModelMemory(n_terms);
	ASSERT(wmm->model != NULL);
	MAG_TimelyModifyMagneticModel(date, fixed_model, wmm->model);
	MAG_FreeMagneticModelMemory(fixed_model);

	wmm_set_earth_ellip(&wmm->ellip);

	return (wmm);
}

void
wmm_close(wmm_t *wmm)
{
	ASSERT(wmm != NULL);
	MAG_FreeMagneticModelMemory(wmm->model);
	free(wmm);
}

static double
wmm_get_decl(const wmm_t *wmm, geo_pos3_t pos)
{
	MAGtype_CoordSpherical		coord_sph;
	MAGtype_CoordGeodetic		coord_geo = {
		.lambda = pos.lon, .phi = pos.lat,
		.HeightAboveEllipsoid = pos.elev / 3281
	};
	MAGtype_GeoMagneticElements	gme;

	MAG_GeodeticToSpherical(wmm->ellip, coord_geo, &coord_sph);
	MAG_Geomag(wmm->ellip, coord_sph, coord_geo, wmm->model, &gme);
	return (gme.Decl);
}

double
wmm_mag2true(const wmm_t *wmm, double m, geo_pos3_t pos)
{
	return (m - wmm_get_decl(wmm, pos));
}

double
wmm_true2mag(const wmm_t *wmm, double t, geo_pos3_t pos)
{
	return (t + wmm_get_decl(wmm, pos));
}
