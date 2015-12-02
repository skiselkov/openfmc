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

#include <math.h>
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

typedef struct {
	double	x;
	double	y;
} vect2_t;

#define	RAD_TO_DEG_RATIO	(M_PI / 180)		/* 1 rad / 180 deg */
#define	DEG_TO_RAD_RATIO	(180 / M_PI)		/* 180 deg / 1 rad */
#define	DEG_TO_RAD(d)		((d) * RAD_TO_DEG_RATIO)
#define	RAD_TO_DEG(r)		((r) * DEG_TO_RAD_RATIO)

#define	GEO_POS2(lat, lon)		((geo_pos2_t){(lat), (lon)})
#define	GEO_POS3(lat, lon, elev)	((geo_pos3_t){(lat), (lon), (elev)})
#define	VECT2(x, y)			((vect2_t){(x), (y)})
#define	VECT3(x, y, z)			((vect3_t){(x), (y), (z)})
#define	VECT2_EQ(a, b)			((a).x == (b).x && (a).y == (b).y)
#define	VECT2_PARALLEL(a, b)		(((a).x * ((a).y / (b).y)) == (b).x)

#define	ZERO_VECT2		((vect2_t){0.0, 0.0})
#define	ZERO_VECT3		((vect3_t){0.0, 0.0, 0.0})
#define	NULL_VECT2		((vect2_t){NAN, NAN})
#define	NULL_VECT3		((vect3_t){NAN, NAN, NAN})
#define	NULL_GEO_POS3		((geo_pos3_t){NAN, NAN, NAN})
#define	NULL_GEO_POS2		((geo_pos2_t){NAN, NAN})
#define	IS_NULL_VECT(a)		(isnan((a).x))
#define	IS_NULL_GEO_POS(a)	(isnan((a).lat))
#define	IS_ZERO_VECT2(a)	((a).x == 0.0 && (a).y == 0.0)
#define	IS_ZERO_VECT3(a)	((a).x == 0.0 && (a).y == 0.0 && (a).z == 0.0)

#define	GEO2_TO_GEO3(v, a)	((geo_pos3_t){(v).lat, (v).lon, (a)})
#define	GEO3_TO_GEO2(v)		((geo_pos2_t){(v).lat, (v).lon})
#define	GEO_POS2_INV(v)		((geo_pos2_t){-(v).lat, -(v).lon})

#define	EARTH_MSL		6371000		/* meters */
#ifndef	ABS
#define	ABS(x)	((x) > 0 ? (x) : -(x))
#endif

double vect3_abs(vect3_t a);
vect3_t vect3_set_abs(vect3_t a, double abs);
vect3_t vect3_unit(vect3_t a, double *l);

vect3_t vect3_add(vect3_t a, vect3_t b);
vect2_t vect2_add(vect2_t a, vect2_t b);
vect3_t vect3_sub(vect3_t a, vect3_t b);
vect3_t vect3_scmul(vect3_t a, double b);
double vect3_dotprod(vect3_t a, vect3_t b);
vect3_t vect3_xprod(vect3_t a, vect3_t b);

vect3_t geo2vect(geo_pos3_t pos);
geo_pos3_t vect2geo(vect3_t v);

int vect_sphere_intersect(vect3_t v, vect3_t o, vect3_t c, double r,
    bool_t confined, vect3_t i[2]);

vect2_t vect2_intersect(vect2_t da, vect2_t oa, vect2_t db, vect2_t ob,
    bool_t confined);

/* geometry parser & validator helpers */
bool_t geo_pos2_from_str(const char *lat, const char *lon, geo_pos2_t *pos);
bool_t geo_pos3_from_str(const char *lat, const char *lon, const char *elev,
    geo_pos3_t *pos);

/*
 * Spherical coordinate system translation.
 */
typedef struct {
	double	geo_matrix[3 * 3];
	double	rot_matrix[2 * 2];
} geo_xlate_t;

geo_xlate_t geo_xlate_init(geo_pos2_t displacement, double rotation);
geo_pos2_t geo_xlate(geo_pos2_t pos, const geo_xlate_t *xlate);

/*
 * Great circle functions.
 */
double gc_distance(geo_pos2_t start, geo_pos2_t end);
double gc_point_hdg(geo_pos2_t start, geo_pos2_t end, double arg);

/*
 * Generic flat-plane projections.
 */
typedef struct {
	geo_xlate_t	xlate;
	double		dist;
} fpp_t;

fpp_t fpp_init(geo_pos2_t center, double rot, double dist);
fpp_t ortho_fpp_init(geo_pos2_t center, double rot);
fpp_t gnomo_fpp_init(geo_pos2_t center, double rot);
fpp_t stereo_fpp_init(geo_pos2_t center, double rot);
vect2_t geo2fpp(geo_pos2_t pos, const fpp_t *fpp);
geo_pos2_t fpp2geo(vect2_t pos, const fpp_t *fpp);

/*
 * Lambert conformal conic projection
 */
typedef struct {
	double	reflat;
	double	reflon;
	double	n;
	double	F;
	double	rho0;
} lcc_t;

lcc_t lcc_init(double reflat, double reflon, double stdpar1, double stdpar2);
vect2_t geo2lcc(geo_pos2_t pos, const lcc_t *lcc);

#ifdef	__cplusplus
}
#endif

#endif	/* _OPENFMC_GEOM_H_ */
