#include <math.h>
#include <stdio.h>

#include "geom.h"
#include "helpers.h"

geo_pos_2d_t null_2d_pos = {FP_NAN, FP_NAN};
geo_pos_3d_t null_3d_pos = {FP_NAN, FP_NAN, FP_NAN};

int
geo_pos_2d_from_str(const char *lat, const char *lon, geo_pos_2d_t *pos)
{
	return (sscanf(lat, "%lf", &pos->lat) == 1 && is_valid_lat(pos->lat) &&
	    sscanf(lon, "%lf", &pos->lon) == 1 && is_valid_lon(pos->lon));
}

int
geo_pos_3d_from_str(const char *lat, const char *lon, const char *elev,
    geo_pos_3d_t *pos)
{
	return (sscanf(lat, "%lf", &pos->lat) == 1 && is_valid_lat(pos->lat) &&
	    sscanf(lon, "%lf", &pos->lon) == 1 && is_valid_lon(pos->lon) &&
	    sscanf(elev, "%lf", &pos->elev) == 1 && is_valid_elev(pos->elev));
}
