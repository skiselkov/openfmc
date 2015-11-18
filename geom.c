#include <math.h>
#include <stdlib.h>

#include "geom.h"
#include "helpers.h"

geo_pos_2d_t null_2d_pos = {FP_NAN, FP_NAN};
geo_pos_3d_t null_3d_pos = {FP_NAN, FP_NAN, FP_NAN};

bool_t
geo_pos_2d_from_str(const char *lat, const char *lon, geo_pos_2d_t *pos)
{
	pos->lat = atof(lat);
	pos->lon = atof(lon);
	return (is_valid_lat(pos->lat) && is_valid_lon(pos->lon));
}

bool_t
geo_pos_3d_from_str(const char *lat, const char *lon, const char *elev,
    geo_pos_3d_t *pos)
{
	pos->lat = atof(lat);
	pos->lon = atof(lon);
	pos->elev = atof(elev);
	return (is_valid_lat(pos->lat) && is_valid_lon(pos->lon) &&
	    is_valid_elev(pos->elev));
}
