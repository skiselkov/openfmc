#ifndef	_OPENFMC_GEOM_H_
#define	_OPENFMC_GEOM_H_

#include "types.h"

#ifdef	__cplusplus
extern "C" {
#endif

typedef struct {
	double	lat;
	double	lon;
} geo_pos_2d_t;

typedef struct {
	double	lat;
	double	lon;
	double	elev;
} geo_pos_3d_t;

extern geo_pos_2d_t null_2d_pos;
extern geo_pos_3d_t null_3d_pos;

/* geometry parser & validator helpers */
bool_t geo_pos_2d_from_str(const char *lat, const char *lon, geo_pos_2d_t *pos);
bool_t geo_pos_3d_from_str(const char *lat, const char *lon, const char *elev,
    geo_pos_3d_t *pos);

#ifdef	__cplusplus
}
#endif

#endif	/* _OPENFMC_GEOM_H_ */
