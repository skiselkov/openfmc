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

#include <math.h>
#include <stdlib.h>
#include <string.h>
#include <stdio.h>

#include "geom.h"
#include "helpers.h"

#if	1
#include <stdio.h>
#define	PRINT_VECT2(v)	printf(#v "(%f, %f)\n", v.x, v.y)
#define	PRINT_VECT3(v)	printf(#v "(%f, %f, %f)\n", v.x, v.y, v.z)
#define	PRINT_GEO2(p)	printf(#p "(%f, %f)\n", p.lat, p.lon)
#define	PRINT_GEO3(p)	printf(#p "(%f, %f, %f)\n", p.lat, p.lon, p.elev)
#define	DEBUG_PRINT(...)	printf(__VA_ARGS__)
#else
#define	PRINT_VECT2(v)
#define	PRINT_VECT3(v)
#define	PRINT_GEO2(p)
#define	PRINT_GEO3(p)
#define	DEBUG_PRINT(...)
#endif

#define	POW3(x)	((x) * (x) * (x))
#define	POW2(x)	((x) * (x))

/*
 * The WGS 84 ellipsoid parameters.
 */
const ellip_t wgs84_ellip = {
	.a = 6378137.0,
	.b = 6356752.314245,
	.f = .00335281066474748071,
	.ecc = 0.08181919084296430238,
	.ecc2 = 0.00669437999019741354,
	.r = 6371200.0
};

/*
 * Naive implementation of matrix multiplication. We don't use this very
 * heavily and can thus afford to rely on compiler auto-vectorization to
 * get this optimized.
 * Multiplies `x' and `y' and places result in 'z'. `xrows', `ycols' and `sz'
 * have meanings as explained below:
 *                sz                     ycols               ycols
 *            |<------->|             |<------->|         |<------->|
 *            |         |             |         |         |         |
 *
 *       --- ++=       =++         .-++=       =++       ++=       =++ --
 * xrows ^   || x00 x01 || \/     /  || y00 y01 ||  ---  || z00 z01 ||  ^ xrows
 *       v   || x10 x11 || /\    /   || y10 y11 ||  ---  || z10 z11 ||  v
 *       --- ++=       =++      /  .-++=       =++       ++=       =++ --
 *                           --'  /
 *                        sz ^   /
 *                           v  /
 *                           --'
 */
static void
matrix_mul(const double *x, const double *y, double *z,
    size_t xrows, size_t ycols, size_t sz)
{
	memset(z, 0, sz * ycols * sizeof (double));
	for (size_t row = 0; row < xrows; row++) {
		for (size_t col = 0; col < ycols; col++) {
			for (size_t i = 0; i < sz; i++) {
				z[row * ycols + col] += x[row * sz + i] *
				    y[i * ycols + col];
			}
		}
	}
}

/*
 * Returns the absolute value (length) of a 3-space vector:
 * r = |a|
 */
double
vect3_abs(vect3_t a)
{
	return (sqrt(POW2(a.x) + POW2(a.y) + POW2(a.z)));
}

/*
 * Same as vect3_abs, but for 2-space vectors.
 */
double
vect2_abs(vect2_t a)
{
	return (sqrt(POW2(a.x) + POW2(a.y)));
}

/*
 * Sets the absolute value (length) of a vector without changing
 * its orientation.
 */
vect3_t
vect3_set_abs(vect3_t a, double abs)
{
	double oldval = vect3_abs(a);
	if (oldval != 0.0)
		return (vect3_scmul(a, abs / oldval));
	else
		return (ZERO_VECT3);
}

/*
 * Returns a unit  vector (vector with identical orientation but a length of 1)
 * for a given input vector. The length of the input vector is stored in `l'.
 */
vect3_t
vect3_unit(vect3_t a, double *l)
{
	double len;
	len = vect3_abs(a);
	if (len == 0)
		return (NULL_VECT3);
	if (l)
		*l = len;
	return (VECT3(a.x / len, a.y / len, a.z / len));
}

/*
 * Adds 3-space vectors `a' and `b' and returns the result:
 * _   _   _
 * r = a + b
 */
vect3_t
vect3_add(vect3_t a, vect3_t b)
{
	return (VECT3(a.x + b.x, a.y + b.y, a.z + b.z));
}

/*
 * Same as vect3_add, but for 2-space vectors.
 */
vect2_t
vect2_add(vect2_t a, vect2_t b)
{
	return (VECT2(a.x + b.x, a.y + b.y));
}

/*
 * Subtracts 3-space vector `b' from vector `a' and returns the result:
 * _   _   _
 * r = a - b
 */
vect3_t
vect3_sub(vect3_t a, vect3_t b)
{
	return (VECT3(a.x - b.x, a.y - b.y, a.z - b.z));
}

/*
 * Performs a scalar multiply of 3-space vector `a' and scalar value `b' and
 * returns the result:
 * _   _
 * r = ab
 */
vect3_t
vect3_scmul(vect3_t a, double b)
{
	return (VECT3(a.x * b, a.y * b, a.z * b));
}

/*
 * Returns the dot product of 3-space vectors `a' and `b':
 *     _   _
 * r = a . b
 */
double
vect3_dotprod(vect3_t a, vect3_t b)
{
	return (a.x * b.x + a.y * b.y + a.z * b.z);
}

/*
 * Returns the cross product of 3-space vectors `a' and `b':
 * _   _   _
 * r = a x b
 */
vect3_t
vect3_xprod(vect3_t a, vect3_t b)
{
	return (VECT3(a.y * b.z - a.z * b.y, a.z * b.x - a.x * b.z,
	    a.x * b.y - a.y * b.x));
}

/*
 * Converts surface coordinates on an Earth-sized spheroid into 3-space
 * coordinate vector in ECEF space. Please note that this considers the
 * Earth to be a perfect sphere and hence cannot be used for very precise
 * calculations. For more accurate conversions, use geo2ecef.
 *
 * @param pos The input position to convert.
 *
 * In 3-space, axes have their origins at the globe center point, are
 * perpendicular to each other and are designated as follows:
 * - x: positive & passing through lat=0, lon=0
 * - y: positive & passing through lat=0, lon=+90
 * - z: positive & passing through lat=90
 */
vect3_t
sph2ecef(geo_pos3_t pos)
{
	vect3_t result;
	double lat_rad, lon_rad, R, R0;

	lat_rad = DEG2RAD(pos.lat);
	lon_rad = DEG2RAD(pos.lon);

	/* R is total distance from center at alt_msl */
	R = pos.elev + EARTH_MSL;
	/*
	 * R0 is the radius of a circular cut parallel to the equator at the
	 * given latitude of a sphere with radius R.
	 */
	R0 = R * cos(lat_rad);
	/* Given R and R0, we can transform the geo coords into 3-space: */
	result.x = R0 * cos(lon_rad);
	result.y = R0 * sin(lon_rad);
	result.z = R * sin(lat_rad);

	return (result);
}

ellip_t
ellip_init(double semi_major, double semi_minor, double flattening)
{
	ellip_t ellip;

	ellip.a = semi_major;
	ellip.b = semi_minor;
	ellip.ecc2 = flattening * (2 - flattening);

	return (ellip);
}

geo_pos3_t
geo2sph(geo_pos3_t pos, const ellip_t *ellip)
{
	double		lat_r = DEG2RAD(pos.lat);
	double		sin_lat = sin(lat_r);
	double		p, z;
	double		Rc;	/* curvature of the prime vertical */
	geo_pos3_t	res;

	Rc = ellip->a / sqrt(1 - ellip->ecc2 * POW2(sin_lat));
	p = (Rc + pos.elev) * cos(lat_r);
	z = ((Rc * (1 - ellip->ecc2)) + pos.elev) * sin_lat;

	res.elev = sqrt(POW2(p) + POW2(z));
	res.lat = RAD2DEG(asin(z / res.elev));
	res.lon = pos.lon;

	return (res);
}

vect3_t
geo2ecef(geo_pos3_t pos, const ellip_t *ellip)
{
	double	h = pos.elev / 3.281;	/* convert to meters */
	double	lat_r = DEG2RAD(pos.lat);
	double	lon_r = DEG2RAD(pos.lon);
	double	Rc;	/* curvature of the prime vertical */
	vect3_t	res;
	double	sin_lat = sin(lat_r), cos_lat = cos(lat_r);
	double	sin_lon = sin(lon_r), cos_lon = cos(lon_r);

	Rc = ellip->a / sqrt(1 - ellip->ecc2 * POW2(sin_lat));

	res.x = (Rc + h) * cos_lat * cos_lon;
	res.y = (Rc + h) * cos_lat * sin_lon;
	res.z = (Rc * (1 - ellip->ecc2) + h) * sin_lat;

	return (res);
}

geo_pos3_t
ecef2geo(vect3_t pos, const ellip_t *ellip)
{
	geo_pos3_t	res;
	double		B;
	double		d;
	double		e;
	double		f;
	double		g;
	double		p;
	double		q;
	double		r;
	double		t;
	double		v;
	double		zlong;

	/*
	 * 1.0 compute semi-minor axis and set sign to that of z in order
	 * to get sign of Phi correct.
	 */
	B = (pos.z >= 0 ? ellip->b : -ellip->b);

	/*
	 * 2.0 compute intermediate values for latitude
	 */
	r = sqrt(POW2(pos.x) + POW2(pos.y));
	e = (B * pos.z - (POW2(ellip->a) - POW2(B))) / (ellip->a * r);
	f = (B * pos.z + (POW2(ellip->a) - POW2(B))) / (ellip->a * r);

	/*
	 * 3.0 find solution to:
	 *       t^4 + 2*E*t^3 + 2*F*t - 1 = 0
	 */
	p = (4.0 / 3.0) * (e * f + 1.0);
	q = 2.0 * (POW2(e) - POW2(f));
	d = POW3(p) + POW2(q);

	if (d >= 0.0) {
		v = pow((sqrt(d) - q), (1.0 / 3.0)) - pow((sqrt(d) + q),
		    (1.0 / 3.0));
	} else {
		v = 2.0 * sqrt(-p) * cos(acos(q / (p * sqrt(-p))) / 3.0);
	}

	/*
	 * 4.0 improve v
	 *       NOTE: not really necessary unless point is near pole
	 */
	if (POW2(v) < fabs(p))
		v = -(POW3(v) + 2.0 * q) / (3.0 * p);
	g = (sqrt(POW2(e) + v) + e) / 2.0;
	t = sqrt(POW2(g)  + (f - v * g) / (2.0 * g - e)) - g;

	res.lat = atan((ellip->a * (1.0 - POW2(t))) / (2.0 * B * t));

	/*
	 * 5.0 compute height above ellipsoid
	 */
	res.elev= (r - ellip->a * t) * cos(res.lat) + (pos.z - B) * sin(res.lat);

	/*
	 *   6.0 compute longitude east of Greenwich
	 */
	zlong = atan2(pos.y, pos.x);
	if (zlong < 0.0)
		zlong = zlong + (2 * M_PI);

	res.lon = zlong;

	/*
	 * 7.0 convert latitude and longitude to degrees & elev to feet
	 */
	res.lat = RAD2DEG(res.lat);
	res.lon = RAD2DEG(res.lon);
	res.elev = MET2FEET(res.elev);

	return (res);
}

/*
 * Converts a 3-space coordinate vector from ECEF coordinate space into
 * geocentric coordinates on an EARTH_MSL-radius spheroid.
 */
geo_pos3_t
ecef2sph(vect3_t v)
{
	geo_pos3_t pos;
	double lat_rad, lon_rad, R, R0;

	R0 = sqrt(v.x * v.x + v.y * v.y);
	R = vect3_abs(v);
	if (R0 == 0) {
		/* to prevent a div-by-zero at the poles */
		R0 = 0.000000001;
	}
	lat_rad = atan(v.z / R0);
	lon_rad = asin(v.y / R0);
	if (v.x < 0.0) {
		if (v.y >= 0.0)
			lon_rad = M_PI - lon_rad;
		else
			lon_rad = lon_rad - M_PI;
	}
	pos.elev = R - EARTH_MSL;
	pos.lat = RAD2DEG(lat_rad);
	pos.lon = RAD2DEG(lon_rad);

	return (pos);
}

/*
 * Determines whether and where a vector intersects the surface of a sphere.
 * Returns the number of intersection points (zero, one or two).
 *
 * @param v Vector for which to determine the intersection.
 * @param o Vector pointing from the coordinate origin to the origin
 *	of vector v (i.e. displacement of `v' from the origin).
 * @param c Displacement of sphere center point from the coordinate origin.
 * @param r Radius of sphere.
 * @param confined If B_TRUE, only intersects which lie between the vector's
 *	start & ending point (inclusive) are returned. Otherwise any
 *	intersect along an infinite linear extension of the vector is returned.
 * @param i If not NULL, this function stores vectors pointing to the
 *	intersection points from the coordinate origin in this array:
 *	- if 0 is returned, two null vectors are stored in the array.
 *	- if 1 is returned, one null vector and one non-null vector pointing
 *	to the intersection point are stored in the array (ordering in the
 *	array is not guarantted as described here).
 *	- if 2 is returned, two non-null vectors pointing to the
 *	intersection points are stored in the array.
 */
unsigned
vect2sph_isect(vect3_t v, vect3_t o, vect3_t c, double r, bool_t confined,
    vect3_t i[2])
{
	vect3_t l, o_min_c;
	double d, l_dot_o_min_c, sqrt_tmp, o_min_c_abs;

	/* convert v into a unit vector 'l' and scalar distance 'd' */
	l = vect3_unit(v, &d);

	/* compute (o - c) and the dot product of l.(o - c) */
	o_min_c = vect3_sub(o, c);
	l_dot_o_min_c = vect3_dotprod(l, o_min_c);

	PRINT_VECT3(l);
	PRINT_VECT3(o_min_c);
	DEBUG_PRINT("l_dot_o_min_c = %f\n", l_dot_o_min_c);

	/*
	 * The full formula for the distance along L for the intersects is:
	 * -(l.(o - c)) +- sqrt((l.(o - c))^2 - abs(o - c)^2 + r^2)
	 * The part in the sqrt may be negative, zero or positive, indicating
	 * respectively no intersection, one touching point or two points, so
	 * before we start sqrt()ing away, we check which it is. Also, this
	 * checks for a solution on an infinite line between extending along
	 * v. Before we declare victory, we check that the computed
	 * points lie on the vector.
	 */
	o_min_c_abs = vect3_abs(o_min_c);
	sqrt_tmp = POW2(l_dot_o_min_c) - POW2(o_min_c_abs) + POW2(r);
	if (sqrt_tmp > 0) {
		/* Two solutions */
		double i1_d, i2_d;
		unsigned intersects = 0;

		sqrt_tmp = sqrt(sqrt_tmp);

		i1_d = -l_dot_o_min_c - sqrt_tmp;
		if ((i1_d >= 0 && i1_d <= d) || !confined) {
			/*
			 * Solution lies on vector, store a vector to it
			 * if the caller requested it.
			 */
			if (i != NULL)
				i[0] = vect3_add(vect3_scmul(l, i1_d), o);
			intersects++;
		} else {
			/* Solution lies outside of line between o1 & o2 */
			i1_d = NAN;
			if (i != NULL)
				i[0] = NULL_VECT3;
		}

		/* ditto for the second intersect */
		i2_d = -l_dot_o_min_c + sqrt_tmp;
		if ((i2_d >= 0 && i2_d <= d) || !confined) {
			if (i != NULL)
				i[1] = vect3_add(vect3_scmul(l, i2_d), o);
			intersects++;
		} else {
			i2_d = NAN;
			if (i != NULL)
				i[1] = NULL_VECT3;
		}

		return (intersects);
	} else if (sqrt_tmp == 0) {
		/* One solution */
		double i1_d;

		if (i != NULL)
			i[1] = NULL_VECT3;

		i1_d = -l_dot_o_min_c;
		if ((i1_d >= 0 && i1_d <= d) || !confined) {
			if (i != NULL)
				i[0] = vect3_add(vect3_scmul(l, i1_d),
				    o);
			return (1);
		} else {
			if (i != NULL)
				i[0] = NULL_VECT3;
			return (0);
		}
	} else {
		/* No solution, no intersections, NaN i1 & i2 */
		if (i != NULL) {
			i[0] = NULL_VECT3;
			i[1] = NULL_VECT3;
		}
		return (0);
	}
}

/*
 * Determines whether and where a 2D vector intersects a 2D circle. The
 * meanings of the arguments and return value are exactly the same as in
 * vect2sph_isect.
 */
unsigned
vect2circ_isect(vect2_t v, vect2_t o, vect2_t c, double r, bool_t confined,
    vect2_t i[2])
{
	/*
	 * This is basically a simplified case of a vect2sph intersection,
	 * where both the vector and sphere's center lie on the xy plane.
	 * So just convert to 3D coordinates with z=0 and run vect2sph_isect.
	 * This only adds one extra coordinate to the calculation, which is
	 * generally negligible on performance.
	 */
	vect3_t v3 = VECT3(v.x, v.y, 0), o3 = VECT3(o.x, o.y, 0);
	vect3_t c3 = VECT3(c.x, c.y, 0);
	vect3_t i3[2];
	int n;

	n = vect2sph_isect(v3, o3, c3, r, confined, i3);
	if (i != NULL) {
		i[0] = VECT2(i3[0].x, i3[0].y);
		i[1] = VECT2(i3[1].x, i3[1].y);
	}

	return (n);
}

/*
 * Calculates a 2D vector/vector intersection point and returns it.
 *
 * @param a First vector.
 * @param oa Vector to origin of first vector from the coordinate origin.
 * @param b Second vector.
 * @param oa Vector to origin of second vector from the coordinate origin.
 * @param confined If B_TRUE, only intersects which lie between the vectors'
 *	start & ending points (inclusive) are considered. Otherwise any
 *	intersect along an infinite linear extension of the vectors is returned.
 *
 * @return A vector from the coordinate origin to the intersection point
 *	or NULL_VECT2 if the vectors are parallel (no intersection or inf
 *	many intersections if they're directly on top of each other).
 */
vect2_t
vect2vect_isect(vect2_t a, vect2_t oa, vect2_t b, vect2_t ob, bool_t confined)
{
	vect2_t p1, p2, p3, p4, r;
	double det;

	if (VECT2_PARALLEL(a, b))
		return (NULL_VECT2);

	if (VECT2_EQ(oa, ob))
		return (oa);

	p1 = a;
	p2 = vect2_add(oa, a);
	p3 = b;
	p4 = vect2_add(ob, b);

	det = ((p1.x * p3.y) - (p1.x * p4.y) - (p2.x * p3.y) + (p2.x * p4.y) -
	    (p1.y * p3.x) + (p1.y * p4.x) + (p2.y * p3.x) - (p2.y * p4.x));
	ASSERT(det != 0.0);
	r.x = ((p1.x * p2.y * p3.x) - (p1.x * p2.y * p4.x) -
	    (p1.y * p2.x * p3.x) + (p1.y * p2.x * p4.x) -
	    (p1.x * p3.x * p4.y) + (p1.x * p3.y * p4.x) +
	    (p2.x * p3.x * p4.y) - (p2.x * p3.y * p4.x)) / det;
	r.y = ((p1.x * p2.y * p3.y) - (p1.x * p2.y * p4.y) -
	    (p1.y * p2.x * p3.y) + (p1.y * p2.x * p4.y) -
	    (p1.y * p3.x * p4.y) + (p1.y * p3.y * p4.x) +
	    (p2.y * p3.x * p4.y) - (p2.y * p3.y * p4.x)) / det;

	if (confined) {
		if (r.x < MIN(p1.x, p2.x) || r.x > MAX(p1.x, p2.x) ||
		    r.x < MIN(p3.x, p4.x) || r.x > MAX(p3.x, p4.x) ||
		    r.y < MIN(p1.y, p2.y) || r.x > MAX(p1.y, p2.y) ||
		    r.y < MIN(p3.y, p4.y) || r.x > MAX(p3.y, p4.y))
			return (NULL_VECT2);
	}

	return (r);
}

/*
 * Given a true heading in degrees, constructs a unit vector pointing in that
 * direction. 0 degress is parallel with y axis and hdg increases clockwise.
 */
vect2_t
hdg2dir(double truehdg)
{
	truehdg = DEG2RAD(truehdg);
	return (VECT2(sin(truehdg), cos(truehdg)));
}

/*
 * Given a direction vector, returns the true heading that the vector
 * is pointing. See hdg2dir for a description of the returned heading value.
 */
double
dir2hdg(vect2_t dir)
{
	return (RAD2DEG(asin(dir.x / vect2_abs(dir))));
}

/*
 * Computes the number of latitudinal subdivisions used for tiling a spherical
 * surface. See world.c for a description of this tiling.
 */
unsigned
sphere_lat_subdiv(double radius, double partition_sz)
{
	ASSERT(radius >= partition_sz);
	return (ceil((radius * M_PI) / partition_sz) + 1);
}

/*
 * Computes the number of longitudinal subdivisions for a given latitude (given
 * in degrees, with 0 being the equator) used for tiling a spherical
 * surface. See world.c for a description of this tiling.
 */
unsigned
sphere_lon_subdiv(double radius, double lat, double partition_sz)
{
	ASSERT(lat >= -90.0 && lat <= 90.0);
	ASSERT(radius >= partition_sz);
	double r = cos(DEG2RAD(lat)) * radius;
	return (ceil((2 * M_PI * r) / partition_sz));
}

bool_t
geo_pos2_from_str(const char *lat, const char *lon, geo_pos2_t *pos)
{
	pos->lat = atof(lat);
	pos->lon = atof(lon);
	return (is_valid_lat(pos->lat) && is_valid_lon(pos->lon));
}

bool_t
geo_pos3_from_str(const char *lat, const char *lon, const char *elev,
    geo_pos3_t *pos)
{
	pos->lat = atof(lat);
	pos->lon = atof(lon);
	pos->elev = atof(elev);
	return (is_valid_lat(pos->lat) && is_valid_lon(pos->lon) &&
	    is_valid_elev(pos->elev));
}

/* Cotangent */
static inline double
cot(double x)
{
	return (1.0 / tan(x));
}

/* Secant */
static inline double
sec(double x)
{
	return (1.0 / cos(x));
}

/*
 * Prepares a set of geographical coordinate translation parameters.
 *
 * @param displacement The relative latitude & longitude (in degrees)
 *	between the origins of the two respective coordinate systems.
 *	For example, a displacement of +10 degrees of latitude (north)
 *	and +20 degrees of longitude (east) will result in an input
 *	coordinate of +5,+5 translating into -5,-15 in the target system
 *	(assuming `rotation' below is zero).
 *	Please note that these coordinates as well as all transformations
 *	are assumed to be in geocentric coordinates on the an EARTH_MSL
 *	radius spheroid.
 * @param rotation The relative rotation of the axes of the target
 *	coordinate system to the source coordinate system in degrees
 *	counter-clockwise. For example, a rotation of +90 degrees and
 *	no translation applied to an input coordinate of +5 degrees
 *	of latitude (north) and +5 degrees of longitude (east) will
 *	translate into -5,+5.
 */
geo_xlate_t
geo_xlate_init(geo_pos2_t displacement, double rotation)
{
	/*
	 * (ECEF axes:)
	 * lat - y axis - alpha
	 * lon - z axis - bravo
	 * rotation - norm to yz axis - theta
	 */
	geo_xlate_t	xlate;
	double		alpha = DEG2RAD(displacement.lat);
	double		bravo = DEG2RAD(-displacement.lon);
	double		theta = DEG2RAD(rotation);

#define	M(m, r, c)	((m)[(r) * 3 + (c)])
	double		R_a[3 * 3], R_b[3 * 3];
	double		sin_alpha = sin(alpha), cos_alpha = cos(alpha);
	double		sin_bravo = sin(bravo), cos_bravo = cos(bravo);
	double		sin_theta = sin(theta), cos_theta = cos(theta);

	/*
	 * +-                  -+
	 * | cos(a)   0  sin(a) |
	 * |    0     1     0   |
	 * | -sin(a)  0  cos(a) |
	 * +-                  -+
	 */
	memset(R_a, 0, sizeof (R_a));
	M(R_a, 0, 0) = cos_alpha;
	M(R_a, 0, 2) = sin_alpha;
	M(R_a, 1, 1) = 1;
	M(R_a, 2, 0) = -sin_alpha;
	M(R_a, 2, 2) = cos_alpha;

	/*
	 * +-                  -+
	 * | cos(g)  -sin(g)  0 |
	 * | sin(g)   cos(g)  0 |
	 * |   0        0     1 |
	 * +-                  -+
	 */
	memset(R_b, 0, sizeof (R_b));
	M(R_b, 0, 0) = cos_bravo;
	M(R_b, 0, 1) = -sin_bravo;
	M(R_b, 1, 0) = sin_bravo;
	M(R_b, 1, 1) = cos_bravo;
	M(R_b, 2, 2) = 1;

	matrix_mul(R_a, R_b, xlate.geo_matrix, 3, 3, 3);

	xlate.rot_matrix[0] = cos_theta;
	xlate.rot_matrix[1] = -sin_theta;
	xlate.rot_matrix[2] = sin_theta;
	xlate.rot_matrix[3] = cos_theta;

	return (xlate);
}

/*
 * Translates a point at `pos' using the translation specified by `xlate'.
 */
static vect3_t
geo_xlate_impl(vect3_t p, const geo_xlate_t *xlate)
{
	vect3_t	q;
	vect2_t	r, s;

	matrix_mul(xlate->geo_matrix, (double *)&p, (double *)&q, 3, 1, 3);
	/*
	 * In the final projection plane, grab y & z coords & rotate along
	 * x axis.
	 */
	r = VECT2(q.y, q.z);
	matrix_mul(xlate->rot_matrix, (double *)&r, (double *)&s, 2, 1, 2);
	q.y = s.x;
	q.z = s.y;

	return (q);
}

/*
 * Translates a point at `pos' using the geo translation specified by `xlate'.
 */
static vect3_t
geo_xlate_inv_impl(vect3_t p, const geo_xlate_t *xlate)
{
	vect3_t	q;
	vect2_t	r, s;

	/* Undo projection plane rotation along y axis. */
	r = VECT2(p.y, p.z);
	matrix_mul(xlate->rot_matrix, (double *)&r, (double *)&s, 2, 1, 2);
	p.y = s.x;
	p.z = s.y;
	/* Undo x (lat) & z (lon) rotation */
	matrix_mul(xlate->geo_matrix, (double *)&p, (double *)&q, 3, 1, 3);

	return (q);
}

/*
 * Translates a point at `pos' using the geo translation specified by `xlate'.
 */
geo_pos2_t
geo_xlate(geo_pos2_t pos, const geo_xlate_t *xlate)
{
	vect3_t		v = sph2ecef(GEO2_TO_GEO3(pos, 0));
	vect3_t		r = geo_xlate_impl(v, xlate);
	geo_pos3_t	res = ecef2sph(r);
	return (GEO_POS2(res.lat, res.lon));
}

/*
 * Translates a point at `pos' using the geo translation specified by `xlate'.
 */
geo_pos2_t
geo_xlate_inv(geo_pos2_t pos, const geo_xlate_t *xlate)
{
	vect3_t		v = sph2ecef(GEO2_TO_GEO3(pos, 0));
	vect3_t		r = geo_xlate_inv_impl(v, xlate);
	geo_pos3_t	res = ecef2sph(r);
	return (GEO_POS2(res.lat, res.lon));
}

/*
 * Returns the great circle distance between two geographical points on the
 * Earth in meters.
 */
double
gc_distance(geo_pos2_t start, geo_pos2_t end)
{
	/*
	 * Convert both coordinates into 3D vectors and calculate the angle
	 * between them. GC distance is proportional to that angle.
	 */
	vect3_t	start_v = geo2ecef(GEO2_TO_GEO3(start, 0), &wgs84_ellip);
	vect3_t	end_v = geo2ecef(GEO2_TO_GEO3(end, 0), &wgs84_ellip);
	vect3_t	s2e = vect3_sub(end_v, start_v);
	double	s2e_abs = vect3_abs(s2e);
	double	alpha = asin(s2e_abs / 2 / EARTH_MSL);
	return	(2 * alpha * EARTH_MSL);
}

double
gc_point_hdg(geo_pos2_t start, geo_pos2_t end, double arg)
{
	/* FIXME: THIS IS BROKEN !!! */
	vect3_t	start_v, end_v, norm_v, an_v, incl_v;

	start_v = geo2ecef(GEO2_TO_GEO3(start, 0), &wgs84_ellip);
	end_v = geo2ecef(GEO2_TO_GEO3(end, 0), &wgs84_ellip);
	norm_v = vect3_set_abs(vect3_xprod(end_v, start_v), EARTH_MSL);
	an_v = vect3_set_abs(vect3_xprod(norm_v, VECT3(0, 0, 1)), EARTH_MSL);
	incl_v = vect3_xprod(norm_v, an_v);
	geo_pos3_t incl = ecef2geo(incl_v, &wgs84_ellip);
	double inclination = incl.lat;

	vect3_t arg_v = {sin(DEG2RAD(arg)) * EARTH_MSL,
	    cos(DEG2RAD(arg)) * EARTH_MSL, 0};
	arg_v = VECT3(sin(DEG2RAD(inclination)) * arg_v.x, arg_v.y,
	    sin(DEG2RAD(inclination)) * arg_v.z);
	arg_v = vect3_unit(vect3_xprod(arg_v, norm_v), NULL);
	double xy = sqrt(POW2(arg_v.x) + POW2(arg_v.y));

	return (RAD2DEG(acos(xy)));
}

/*
 * Prepares a set of projection parameters for projections from a fixed
 * origin along the projection axis onto a flat projection plane. The
 * plane is centered at `center' and is rotated `rot' degrees relative to
 * the sphere's native coordinate system. The projection plane touches the
 * sphere at the center point and is parallel to its surface there. Points
 * on the sphere are projected from a fixed origin along a projection axis
 * that is perpendicular to the projection plane and passes through its
 * center. The distance of this point along the projection axis from the
 * projection plane is `dist' with positive offsets increasing away from
 * the sphere's center.
 *
 *   projection.
 *       center \ | <- projection axis (positive offsets)
 *               v|
 *  ==============+=============+======= <- projection plane
 *         - '    |    ' -     / ^-- projected point
 *       /        |        \ x <-- projecting point
 *     /          |         /\
 *    | sphere    |       /   |
 *    | center -> +     /     |
 *    |           |   /       |
 *     \          | /        /
 *       \        + <- projection origin
 *         -      |      -
 *           '----|----'
 *                |
 *                | <- projection axis (negative offsets)
 *
 * You can pass INFINITY for `dist', in which case the projection origin
 * will be centered at +INFINITY, constructing an orthographic projection.
 * N.B. ATM there is no way to position the projection point at -INFINITY.
 * If you wish to construct an inverted orthographic projection, simply
 * flip the `x' coordinate of the projected points. Also, it is illegal
 * to pass dist == 0.0.
 */
fpp_t
fpp_init(geo_pos2_t center, double rot, double dist)
{
	fpp_t fpp;

	VERIFY(dist != 0);
	fpp.xlate = geo_xlate_init(center, rot);
	fpp.dist = dist;

	return (fpp);
}

/*
 * Constructs an orthographic projection. This is a flat plane projection with
 * the projection origin at +INFINITY. See `fpp_init' for more information.
 */
fpp_t
ortho_fpp_init(geo_pos2_t center, double rot)
{
	return (fpp_init(center, rot, INFINITY));
}

/*
 * Constructs a gnomonic projection. This is a flat plane projection with
 * the origin at the Earth's center. See `fpp_init' for more information.
 */
fpp_t
gnomo_fpp_init(geo_pos2_t center, double rot)
{
	return (fpp_init(center, rot, -EARTH_MSL));
}

/*
 * Constructs a stereographic projection. This is a projection with
 * the origin at the intersection of the projection axis and the surface
 * of the Earth opposite the projection plane's center point.
 * See `fpp_init' for more information.
 */
fpp_t
stereo_fpp_init(geo_pos2_t center, double rot)
{
	return (fpp_init(center, rot, -2 * EARTH_MSL));
}

/*
 * Projects a point at `pos' according to the projection `proj' and returns
 * a 2D vector to the projected point's location on the projection plane.
 * If the specified point cannot be projected (because its projection falls
 * outside of the projection plane), NULL_VECT2 is returned instead.
 */
vect2_t
geo2fpp(geo_pos2_t pos, const fpp_t *fpp)
{
	vect3_t pos_v;
	vect2_t res_v;

	pos_v = geo2ecef(GEO2_TO_GEO3(pos, 0), &wgs84_ellip);
	pos_v = geo_xlate_impl(pos_v, &fpp->xlate);
	if (isfinite(fpp->dist)) {
		if (fpp->dist < 0.0 && pos_v.x <= fpp->dist + EARTH_MSL)
			return (NULL_VECT2);
		res_v.x = fpp->dist * (pos_v.y / (fpp->dist +
		    EARTH_MSL - pos_v.x));
		res_v.y = fpp->dist * (pos_v.z / (fpp->dist +
		    EARTH_MSL - pos_v.x));
	} else {
		res_v = VECT2(pos_v.y, pos_v.z);
	}

	return (res_v);
}

/*
 * Back-projects a point from a projection surface into spherical coordinate
 * space. N.B. since projection loses some information about the original
 * input point, back-projection is incomplete for projections where either:
 *	a) the projection origin was non-negative, or
 *	b) the projection origin was less than -EARTH_MSL
 * This means back-projection is only uniquely possible possible for
 * gnomonic, stereographic and other projections where the projection origin
 * lies "inside" the projected sphere. In case back-projection is not unique,
 * the point closer to the projection origin is returned.
 */
geo_pos2_t
fpp2geo(vect2_t pos, const fpp_t *fpp)
{
	vect3_t	v = VECT3(-fpp->dist, pos.x, pos.y);
	vect3_t	o = VECT3(EARTH_MSL + fpp->dist, 0, 0);
	vect3_t i[2];
	vect3_t r;
	int n;

	n = vect2sph_isect(v, o, ZERO_VECT3, EARTH_MSL, B_FALSE, i);
	if (n == 0) {
		/* Invalid input point, not a member of projection */
		return (NULL_GEO_POS2);
	}
	if (n == 2) {
		/*
		 * Two solutions, find the one that's closer to the projection
		 * origin while located between it and the projection plane
		 * and place it in i[0].
		 */
		if (fpp->dist >= -EARTH_MSL) {
			if (i[1].x > i[0].x)
				i[0] = i[1];
		} else {
			if (i[1].x < i[0].x)
				i[0] = i[1];
		}
	}
	/* Result is now in i[0]. Inv-xlate to global space & ecef2sph. */
	r = geo_xlate_inv_impl(i[0], &fpp->xlate);
	return (GEO3_TO_GEO2(ecef2geo(r, &wgs84_ellip)));
}

/*
 * Prepares a set of Lambert conformal conic projection parameters.
 *
 * @param reflat Reference latitude in degrees.
 * @param reflon Reference longitude in degrees.
 * @param stdpar1 First standard parallel in degrees.
 * @param stdpar2 Second standard parallel in degrees.
 *
 * @return The set of lcc parameters to pass to geo2lcc.
 */
lcc_t
lcc_init(double reflat, double reflon, double stdpar1, double stdpar2)
{
	double	phi0 = DEG2RAD(reflat);
	double	phi1 = DEG2RAD(stdpar1);
	double	phi2 = DEG2RAD(stdpar2);
	lcc_t	lcc;

	lcc.reflat = DEG2RAD(reflat);
	lcc.reflon = DEG2RAD(reflon);

	if (stdpar1 == stdpar2)
		lcc.n = sin(phi1);
	else
		lcc.n = log(cos(phi1) * sec(phi2)) / log(tan(M_PI / 4.0 +
		    phi2 / 2.0) * cot(M_PI / 4.0 + phi1 / 2.0));
	lcc.F = (cos(phi1) * pow(tan(M_PI / 4.0 * phi1 / 2.0), lcc.n)) /
	    lcc.n;
	lcc.rho0 = lcc.F * pow(cot(M_PI / 4.0 + phi0 / 2.0), lcc.n);

	return (lcc);
}

/*
 * Projects a point at `pos' using the projection `lcc'.
 */
vect2_t
geo2lcc(geo_pos2_t pos, const lcc_t *lcc)
{
	vect2_t		result;
	double		rho;
	double		lat = DEG2RAD(pos.lat);
	double		lon = DEG2RAD(pos.lon);

	rho = lcc->F * pow(cot(M_PI / 4 + lat / 2), lcc->n);
	result.x = rho * sin(lon - lcc->reflon);
	result.y = lcc->rho0 - rho * cos(lcc->n * (lat - lcc->reflat));

	return (result);
}
