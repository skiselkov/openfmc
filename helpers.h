#ifndef	_OPENFMC_HELPERS_H_
#define	_OPENFMC_HELPERS_H_

#include <stdarg.h>

#include "airac.h"
#include "types.h"

#ifdef	__cplusplus
extern "C" {
#endif

#if	defined(__GNUC__) || defined(__clang__)
#define	PRINTF_ATTR(x)	__attribute__ ((format (printf, x, x + 1)))
#else	/* __GNUC */
#define	PRINTF_ATTR(x)
#endif	/* __GNUC */

/* Minimum/Maximum allowable elevation AMSL of anything */
#define	MIN_ELEV	-2000.0
#define	MAX_ELEV	30000.0

/* Minimum/Maximum allowable altitude AMSL of anything */
#define	MIN_ALT		-2000.0
#define	MAX_ALT		100000.0

/* Maximum valid speed of anything */
#define	MAX_SPD		1000.0

/* Minimum/Maximum allowable arc radius on any procedure */
#define	MIN_ARC_RADIUS	0.1
#define	MAX_ARC_RADIUS	100.0

#define	UNUSED(x)	(void)(x)

/* generic parser validator helpers */

static inline bool_t
is_valid_lat(double lat)
{
	return (lat <= 90.0 && lat >= -90.0);
}

static inline bool_t
is_valid_lon(double lon)
{
	return (lon <= 180.0 && lon >= -180.0);
}

static inline bool_t
is_valid_elev(double elev)
{
	return (elev >= MIN_ELEV && elev <= MAX_ELEV);
}

static inline bool_t
is_valid_alt(double alt)
{
	return (alt >= MIN_ALT && alt <= MAX_ALT);
}

static inline bool_t
is_valid_spd(double spd)
{
	return (spd >= 0.0 && spd <= MAX_SPD);
}

static inline bool_t
is_valid_hdg(double hdg)
{
	/* "0" is not a valid heading, "360" is */
	return (hdg > 0.0 && hdg <= 360.0);
}

static inline bool_t
is_valid_arc_radius(double radius)
{
	return (radius >= MIN_ARC_RADIUS && radius <= MAX_ARC_RADIUS);
}

bool_t is_valid_vor_freq(double freq_mhz);
bool_t is_valid_loc_freq(double freq_mhz);
bool_t is_valid_ndb_freq(double freq_khz);
bool_t is_valid_tacan_freq(double freq_mhz);
bool_t is_valid_rwy_ID(const char *rwy_ID);

/* string processing helpers */
size_t explode_line(char *line, char delim, char **comps, size_t capcity);
void strip_newline(char *line);
void append_format(char **str, size_t *sz, const char *format, ...)
    PRINTF_ATTR(3);

#if	defined(__GNUC__) || defined(__clang__)
#define	highbit64(x)	(64 - __builtin_clzll(x) - 1)
#define	highbit32(x)	(32 - __builtin_clzll(x) - 1)
#else
#error	"Compiler platform unsupported, please add highbit definition"
#endif

/*
 * return x rounded up to the nearest power-of-2.
 */
#define	P2ROUNDUP(x)	(-(-(x) & -(1 << highbit64(x))))

#ifdef	__cplusplus
}
#endif

#endif	/* _OPENFMC_HELPERS_H_ */
