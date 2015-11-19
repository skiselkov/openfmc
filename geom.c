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

#include "geom.h"
#include "helpers.h"

#if	0
#include <stdio.h>
#define	PRINT_VECT(v)		printf(#v "(%f, %f, %f)\n", v.x, v.y, v.z)
#define	DEBUG_PRINT(...)	printf(__VA_ARGS__)
#else
#define	PRINT_VECT(v)
#define	DEBUG_PRINT(...)
#endif
#define	POW2(x)	(x * x)

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
 * Returns a unit 3-space vector (vector with identical orientation but
 * a length of 1) for a given input vector. The length of the input vector
 * is stored in `l'.
 */
inline vect3_t
vect3_unit(vect3_t a, double *l)
{
	*l = vect3_abs(a);
	if (*l == 0)
		return (NULL_VECT3);
	return ((vect3_t){a.x / *l, a.y / *l, a.z / *l});
}

/*
 * Adds 3-space vectors `a' and `b' and returns the result:
 * _   _   _
 * r = a + b
 */
vect3_t
vect3_add(vect3_t a, vect3_t b)
{
	return ((vect3_t){a.x + b.x, a.y + b.y, a.z + b.z});
}

/*
 * Subtracts 3-space vector `b' from vector `a' and returns the result:
 * _   _   _
 * r = a - b
 */
vect3_t
vect3_sub(vect3_t a, vect3_t b)
{
	return ((vect3_t){a.x - b.x, a.y - b.y, a.z - b.z});
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
	return ((vect3_t){a.x * b, a.y * b, a.z * b});
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
	return ((vect3_t){a.y * b.z - a.z * b.y, a.z * b.x - a.x * b.z,
	    a.x * b.y - a.y * b.x});
}

/*
 * Converts surface coordinates into 3-space coordinate vector from the
 * Earth's center. Please note that this considers the Earth to be a perfect
 * sphere and hence cannot be used for very precise calculations.
 *
 * @param pos The input position to convert.
 *
 * In 3-space, axes have their origins at the globe center point, are
 * perpendicular to each other and are designated as follows:
 * - x: positive & passing through lat=0, lon=+90
 * - y: positive & passing through lat=0, lon=0
 * - z: positive & passing through lat=90
 */
vect3_t
geo2vect_coords(geo_pos3_t pos)
{
	vect3_t result;
	double lat_rad, lon_rad, R, R0;

	lat_rad = DEG_TO_RAD(pos.lat);
	lon_rad = DEG_TO_RAD(pos.lon);

	/* R is total distance from center at alt_msl */
	R = pos.elev + EARTH_MSL;
	/*
	 * R0 is the radius of a circular cut parallel to the equator at the
	 * given latitude of a sphere with radius R.
	 */
	R0 = R * cos(lat_rad);
	/* Given R and R0, we can transform the geo coords into 3-space: */
	result.x = R0 * sin(lon_rad);
	result.y = R0 * cos(lon_rad);
	result.z = R * sin(lat_rad);

	return (result);
}

/*
 * Converts a 3-space coordinate vector into geographical coordinates on Earth.
 * For axis alignment, see geo2vect_coords().
 */
geo_pos3_t
vect2geo_coords(vect3_t v)
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
	lon_rad = asin(v.x / R0);
	if (v.y < 0.0) {
		if (v.x >= 0.0)
			lon_rad = M_PI - lon_rad;
		else
			lon_rad = lon_rad - M_PI;
	}
	pos.elev = R - EARTH_MSL;
	pos.lat = RAD_TO_DEG(lat_rad);
	pos.lon = RAD_TO_DEG(lon_rad);

	return (pos);
}

/*
 * Determines whether and where a vector intersects the surface of a sphere.
 * Only takes into account points that lie exactly on the vector
 * (i.e. between its starting and ending points, inclusive).
 * Returns the number of intersection points (zero, one or two).
 *
 * @param v Vector for which to determine the intersection.
 * @param o Vector pointing from the coordinate origin to the origin
 *	of vector v (i.e. displacement of `v' from the origin).
 * @param c Displacement of sphere center point from the coordinate origin.
 * @param r Radius of sphere.
 * @param i If not NULL, this function stores vectors pointing to the
 *	intersection points from the coordinate origin in this array:
 *	- if 0 is returned, two null vectors are stored in the array.
 *	- if 1 is returned, one null vector and one non-null vector pointing
 *	to the intersection point are stored in the array (ordering in the
 *	array is not guarantted as described here).
 *	- if 2 is returned, two non-null vectors pointing to the
 *	intersection points are stored in the array.
 */
int
vect_sphere_intersect(vect3_t v, vect3_t o, vect3_t c,
    double r, vect3_t i[2])
{
	vect3_t l, o_min_c;
	double d, l_dot_o_min_c, sqrt_tmp, o_min_c_abs;

	/* convert v into a unit vector 'l' and scalar distance 'd' */
	l = vect3_unit(v, &d);

	/* compute (o - c) and the dot product of l.(o - c) */
	o_min_c = vect3_sub(o, c);
	l_dot_o_min_c = vect3_dotprod(l, o_min_c);

	PRINT_VECT(l);
	PRINT_VECT(o_min_c);
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
		int intersects = 0;

		sqrt_tmp = sqrt(sqrt_tmp);

		i1_d = -l_dot_o_min_c - sqrt_tmp;
		if (i1_d >= 0 && i1_d <= d) {
			/*
			 * Solution lies on vector, store a vector to it
			 * if the caller requested it.
			 */
			if (i)
				i[0] = vect3_add(vect3_scmul(l, i1_d),
				    o);
			intersects++;
		} else {
			/* Solution lies outside of line between o1 & o2 */
			i1_d = NAN;
			if (i)
				i[0] = NULL_VECT3;
		}

		/* ditto for the second intersect */
		i2_d = -l_dot_o_min_c + sqrt_tmp;
		if (i2_d >= 0 && i2_d <= d) {
			if (i)
				i[1] = vect3_add(vect3_scmul(l, i2_d),
				    o);
			intersects++;
		} else {
			i2_d = NAN;
			if (i)
				i[1] = NULL_VECT3;
		}

		return (intersects);
	} else if (sqrt_tmp == 0) {
		/* One solution */
		double i1_d;

		if (i)
			i[1] = NULL_VECT3;

		i1_d = -l_dot_o_min_c;
		if (i1_d >= 0 && i1_d <= d) {
			if (i)
				i[0] = vect3_add(vect3_scmul(l, i1_d),
				    o);
			return (1);
		} else {
			if (i)
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
	double r = cos(DEG_TO_RAD(lat)) * radius;
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
