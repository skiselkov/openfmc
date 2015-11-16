#ifndef	_OPENFMC_GEOM_H_
#define	_OPENFMC_GEOM_H_

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

#endif	/* _OPENFMC_GEOM_H_ */