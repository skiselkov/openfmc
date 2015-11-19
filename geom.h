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

#ifndef	_OPENFMC_GEOM_H_
#define	_OPENFMC_GEOM_H_

#include "types.h"

#ifdef	__cplusplus
extern "C" {
#endif

typedef struct {
	double	lat;
	double	lon;
} geo_pos2_t;

typedef struct {
	double	lat;
	double	lon;
	double	elev;
} geo_pos3_t;

typedef struct {
	double	x;
	double	y;
	double	z;
} vect3_t;

#define	RAD_TO_DEG_RATIO	0.01745329251994329576	/* 1 rad / 180 deg */
#define	DEG_TO_RAD_RATIO	57.29577951308232087684	/* 180 deg / 1 rad */
#define	DEG_TO_RAD(d)		(d * RAD_TO_DEG_RATIO)
#define	RAD_TO_DEG(r)		(r * DEG_TO_RAD_RATIO)

#define	ZERO_VECT3		((vect3_t){0.0, 0.0, 0.0})
#define	NULL_VECT3		((vect3_t){FP_NAN, FP_NAN, FP_NAN})
#define	NULL_GEO_POS3		((geo_pos3_t){FP_NAN, FP_NAN, FP_NAN})
#define	NULL_GEO_POS2		((geo_pos2_t){FP_NAN, FP_NAN})
#define	IS_NULL_VECT3(a)	(isnan(a.x))
#define	IS_NULL_GEO_POS3(a)	(isnan(a.lat))
#define	IS_NULL_GEO_POS2(a)	IS_NULL_GEO_POS3((a))
#define	IS_ZERO_VECT3(a)	(a.x == 0.0 && a.y == 0.0 && a.z == 0.0)

#define	VECT3_SCMUL(v, l)	((vect3_t){v.x * l, v.y * l, v.z * l})

#define	EARTH_MSL		6371000		/* meters */

double vect3_abs(vect3_t a);
vect3_t vect3_unit(vect3_t a, double *l);

vect3_t vect3_add(vect3_t a, vect3_t b);
vect3_t vect3_sub(vect3_t a, vect3_t b);
vect3_t vect3_scmul(vect3_t a, double b);
double vect3_dotprod(vect3_t a, vect3_t b);
vect3_t vect3_xprod(vect3_t a, vect3_t b);

vect3_t geo2vect_coords(double lat, double lon, double alt_msl,
    double msl_radius);
void vect2geo_coords(vect3_t v, double msl_radius, double *lat,
    double *lon, double *alt_msl);

int vect_sphere_intersect(vect3_t v, vect3_t o, vect3_t c,
    double r, vect3_t i[2]);

/* geometry parser & validator helpers */
bool_t geo_pos2_from_str(const char *lat, const char *lon, geo_pos2_t *pos);
bool_t geo_pos3_from_str(const char *lat, const char *lon, const char *elev,
    geo_pos3_t *pos);

/* vector functions */

#ifdef	__cplusplus
}
#endif

#endif	/* _OPENFMC_GEOM_H_ */