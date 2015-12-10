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

#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <string.h>

#include "math.h"
#include "log.h"
#include "helpers.h"
#include "perf.h"

#define	ISA_SL_TEMP		15.0	/* ISA seal level emperature in degC */
#define	ISA_SL_TEMP_K		288.15	/* ISA seal level emperature in degC */
#define	ISA_SL_PRESS		1013.25	/* ISA sea level pressure in hPa */
#define	ISA_ELR_PER_1000	1.98	/* ISA environmental lapse rate */
#define	ISA_TLR_PER_M		0.0065	/* ISA temperature lapse rate K/m */

#define	CONST_P_SPEC_HEAT	1007	/* Constant pressure specific heat */
#define	EARTH_GRAVITY		9.80665	/* Earth surface grav. acceleration */
#define	DRY_AIR_MOL		0.0289644	/* Molar mass of dry air */

#define	GAMMA			1.4	/* Specific heat ratio for dry air */
#define	SPEED_SOUND_ISA		340.3	/* Speed of sound at 15 degrees C */
#define	R_univ			8.31447	/* Universal gas constant */
#define	R_spec			287.058	/* Specific gas constant of dry air */

#define	ACFT_PERF_MIN_VERSION	1
#define	ACFT_PERF_MAX_VERSION	1
#define	MAX_LINE_COMPS		2

/*
 * Parses a set of bezier curve points from the input CSV file. Used to parse
 * curve points for performance curves.
 *
 * @param fp FILE pointer from which to read lines.
 * @param curvep Pointer that will be filled with the parsed curve.
 * @param numpoints Number of points to fill in the curve (and input lines).
 * @param line_num Current file line number. Used to pass to
 *	parse_get_next_line to track file parsing progress.
 *
 * @return B_TRUE on successful parse, B_FALSE on failure.
 */
static bool_t
parse_curves(FILE *fp, bezier_t **curvep, size_t numpoints, size_t *line_num)
{
	bezier_t	*curve;
	char		*line = NULL;
	size_t		line_cap = 0;
	ssize_t		line_len = 0;

	ASSERT(*curvep == NULL);
	curve = bezier_alloc(numpoints);

	for (size_t i = 0; i < numpoints; i++) {
		char *comps[2];

		line_len = parser_get_next_line(fp, &line, &line_cap, line_num);
		if (line_len <= 0)
			goto errout;
		if (explode_line(line, ',', comps, 2) != 2)
			goto errout;
		curve->pts[i] = VECT2(atof(comps[0]), atof(comps[1]));
		if (i > 0 && curve->pts[i - 1].x >= curve->pts[i].x)
			goto errout;
	}

	*curvep = curve;
	free(line);

	return (B_TRUE);
errout:
	free(line);
	bezier_free(curve);
	return (B_FALSE);
}

#define	PARSE_SCALAR(name, var) \
	if (strcmp(comps[0], name) == 0) { \
		if (ncomps != 2 || (var) != 0.0) { \
			openfmc_log(OPENFMC_LOG_ERR, "Error parsing " \
			    "acft perf file %s:%lu: malformed or " \
			    "duplicate " name " line.", filename, \
			    line_num); \
			goto errout; \
		} \
		(var) = atof(comps[1]); \
		if ((var) <= 0.0) { \
			openfmc_log(OPENFMC_LOG_ERR, "Error parsing acft " \
			    "perf file %s:%lu: invalid value for " name, \
			    filename, line_num); \
			goto errout; \
		} \
	}

/*
 * Checks that comps[0] contains `name' and if it does, parses comps[1] number
 * of bezier curve points into `var'.
 */
#define	PARSE_CURVE(name, var) \
	if (strcmp(comps[0], name) == 0) { \
		if (ncomps != 2 || atoi(comps[1]) < 2 || (var) != NULL) { \
			openfmc_log(OPENFMC_LOG_ERR, "Error parsing acft " \
			    "perf file %s:%lu: malformed or duplicate " \
			    name " line.", filename, line_num); \
			goto errout; \
		} \
		if (!parse_curves(fp, &(var), atoi(comps[1]), &line_num)) { \
			openfmc_log(OPENFMC_LOG_ERR, "Error parsing acft " \
			    "perf file %s:%lu: malformed or missing lines.", \
			    filename, line_num); \
			goto errout; \
		} \
	}

acft_perf_t *
acft_perf_parse(const char *filename)
{
	acft_perf_t	*acft = calloc(sizeof (*acft), 1);
	FILE		*fp = fopen(filename, "r");
	char		*line = NULL;
	size_t		line_cap = 0, line_num = 0;
	ssize_t		line_len = 0;
	char		*comps[MAX_LINE_COMPS];
	bool_t		version_check_completed = B_FALSE;

	if (fp == NULL)
		goto errout;
	while ((line_len = parser_get_next_line(fp, &line, &line_cap,
	    &line_num)) != -1) {
		ssize_t ncomps;

		if (line_len == 0)
			continue;
		ncomps = explode_line(line, ',', comps, MAX_LINE_COMPS);
		if (ncomps < 0) {
			openfmc_log(OPENFMC_LOG_ERR, "Error parsing acft "
			    "perf file %s:%lu: malformed line, too many "
			    "line components.", filename, line_num);
			goto errout;
		}
		ASSERT(ncomps > 0);
		if (strcmp(comps[0], "VERSION") == 0) {
			int vers;

			if (version_check_completed) {
				openfmc_log(OPENFMC_LOG_ERR, "Error parsing "
				    "acft perf file %s:%lu: duplicate VERSION "
				    "line.", filename, line_num);
				goto errout;
			}
			if (ncomps != 2) {
				openfmc_log(OPENFMC_LOG_ERR, "Error parsing "
				    "acft perf file %s:%lu: malformed VERSION "
				    "line.", filename, line_num);
				goto errout;
			}
			vers = atoi(comps[1]);
			if (vers < ACFT_PERF_MIN_VERSION ||
			    vers > ACFT_PERF_MAX_VERSION) {
				openfmc_log(OPENFMC_LOG_ERR, "Error parsing "
				    "acft perf file %s:%lu: unsupported file "
				    "version %d.", filename, line_num, vers);
				goto errout;
			}
			version_check_completed = B_TRUE;
			continue;
		}
		if (!version_check_completed) {
			openfmc_log(OPENFMC_LOG_ERR, "Error parsing acft "
			    "perf file %s:%lu: first line was not VERSION.",
			    filename, line_num);
			goto errout;
		}
		if (strcmp(comps[0], "ACFTTYPE") == 0) {
			if (ncomps != 2 || acft->acft_type != NULL) {
				openfmc_log(OPENFMC_LOG_ERR, "Error parsing "
				    "acft perf file %s:%lu: malformed or "
				    "duplicate ACFTTYPE line.", filename,
				    line_num);
				goto errout;
			}
			acft->acft_type = strdup(comps[1]);
		} else if (strcmp(comps[0], "ENGTYPE") == 0) {
			if (ncomps != 2 || acft->eng_type != NULL) {
				openfmc_log(OPENFMC_LOG_ERR, "Error parsing "
				    "acft perf file %s:%lu: malformed or "
				    "duplicate ENGTYPE line.", filename,
				    line_num);
				goto errout;
			}
			acft->eng_type = strdup(comps[1]);
		}
		else PARSE_SCALAR("MAXTHR", acft->eng_max_thr)
		else PARSE_SCALAR("REFZFW", acft->ref_zfw)
		else PARSE_SCALAR("MAXFUEL", acft->max_fuel)
		else PARSE_SCALAR("MAXGW", acft->max_gw)
		else PARSE_CURVE("THRDENS", acft->thr_dens_curve)
		else PARSE_CURVE("THRISA", acft->thr_isa_curve)
		else PARSE_CURVE("SFCTHR", acft->sfc_thr_curve)
		else PARSE_CURVE("SFCDENS", acft->sfc_dens_curve)
		else PARSE_CURVE("SFCISA", acft->sfc_isa_curve)
		else {
			openfmc_log(OPENFMC_LOG_ERR, "Error parsing acft perf "
			    "file %s:%lu: unknown line", filename, line_num);
			goto errout;
		}
	}

	if (acft->acft_type == NULL || acft->ref_zfw <= 0 ||
	    acft->max_fuel <= 0 || acft->max_gw <= 0 ||
	    acft->eng_type == NULL || acft->eng_max_thr <= 0 ||
	    acft->thr_dens_curve == NULL || acft->thr_isa_curve == NULL ||
	    acft->sfc_thr_curve == NULL || acft->sfc_dens_curve == NULL ||
	    acft->sfc_isa_curve == NULL) {
		openfmc_log(OPENFMC_LOG_ERR, "Error parsing acft perf "
		    "file %s: missing or corrupt data fields.", filename);
		goto errout;
	}

	fclose(fp);
	free(line);

	return (acft);
errout:
	if (fp)
		fclose(fp);
	if (acft)
		acft_perf_destroy(acft);
	free(line);
	return (NULL);
}

void
acft_perf_destroy(acft_perf_t *acft)
{
	if (acft->acft_type)
		free(acft->acft_type);
	if (acft->eng_type)
		free(acft->eng_type);
	if (acft->thr_dens_curve)
		bezier_free(acft->thr_dens_curve);
	if (acft->thr_isa_curve)
		bezier_free(acft->thr_isa_curve);
	free(acft);
}

/*
 * Returns the maximum average thrust that the engines can attain between
 * two altitudes during a climb.
 *
 * @param flt Flight performance limits.
 * @param acft Aircraft performance limit curves.
 * @param alt1 First (lower) altitude in feet.
 * @param temp1 Estimated TAT in C at altitude alt1.
 * @param alt1 Second (higher) altitude in feet.
 * @param temp1 Estimated TAT in C at altitude alt2.
 * @param tp_alt Altitude of the tropopause in feet.
 *
 * @return The maximum average engine thrust (in Kilonewtons) attainable
 *	between alt1 and alt2 while keeping the flight and aircraft limits.
 */
double
eng_max_thr_avg(const flt_perf_t *flt, acft_perf_t *acft, double alt1,
    double alt2, double ktas, double qnh, double isadev, double tp_alt)
{
	double P, Ps, Pd, D, avg_temp, thr;
	double avg_alt = AVG(alt1, alt2);
	/* convert altitudes to flight levels to calculate avg temp */
	double alt1_fl = alt2fl(alt1, qnh);
	double alt2_fl = alt2fl(alt2, qnh);
	double tp_fl = alt2fl(tp_alt, qnh);

	/*
	 * FIXME: correctly weight the temp average when tp_alt < alt2.
	 */
	avg_temp = AVG(isadev2sat(alt1_fl, isadev),
	    isadev2sat(alt2_fl < tp_fl ? alt2_fl : tp_fl, isadev));
	/*
	 * Ps is the average static air pressure between alt1 and alt2. Next
	 * calculate dynamic pressure rise to get total effective air pressure.
	 */
	Ps = alt2press(avg_alt, qnh);
	Pd = dyn_press(ktas, Ps, avg_temp);
	P = Ps + Pd;
	/*
	 * Finally grab effective air density.
	 */
	isadev = isadev2sat(alt2fl(avg_alt, qnh), avg_temp);
	D = air_density(P + Pd, isadev);
	/*
	 * Derive engine performance.
	 */
	thr = quad_bezier_func_get(D, acft->thr_dens_curve) *
	    quad_bezier_func_get(isadev, acft->thr_isa_curve) *
	    flt->thr_derate;

	return (thr);
}

/*
 * Converts a true airspeed to Mach number.
 *
 * @param ktas True airspeed in knots.
 * @param oat Static outside air temperature in degrees C.
 *
 * @return Mach number.
 */
double
ktas2mach(double ktas, double oat)
{
	return (KTS2MPS(ktas) / speed_sound(oat));
}

/*
 * Converts Mach number to true airspeed.
 *
 * @param ktas Mach number.
 * @param oat Static outside air temperature in degrees C.
 *
 * @return True airspeed in knots.
 */
double
mach2ktas(double mach, double oat)
{
	return (MPS2KTS(mach * speed_sound(oat)));
}

/*
 * Converts true airspeed to calibrated airspeed.
 *
 * @param ktas True airspeed in knots.
 * @param pressure Static air pressure in hPa.
 * @param oat Static outside air temperature in degrees C.
 *
 * @return Calibrated airspeed in knots.
 */
double
ktas2kcas(double ktas, double pressure, double oat)
{
	double qc = impact_press(ktas2mach(ktas, oat), pressure);
	return (MPS2KTS(SPEED_SOUND_ISA *
	    sqrt(5 * (pow(qc / ISA_SL_PRESS + 1, 0.2857142857) - 1))));
}

/*
 * Converts calibrated airspeed to true airspeed.
 *
 * @param ktas Calibrated airspeed in knots.
 * @param pressure Static air pressure in hPa.
 * @param oat Static outside air temperature in degrees C.
 *
 * @return True airspeed in knots.
 */
double
kcas2ktas(double kcas, double pressure, double oat)
{
	double	qc, mach;

	/*
	 * Take the CAS equation and solve for qc (impact pressure):
	 *
	 * qc = P0(((cas^2 / 5* a0^2) + 1)^3.5 - 1)
	 *
	 * Where P0 is pressure at sea level, cas is calibrated airspeed
	 * in m.s^-1 and a0 speed of sound at ISA temperature.
	 */
	qc = ISA_SL_PRESS * (pow((POW2(KTS2MPS(kcas)) / (5 *
	    POW2(SPEED_SOUND_ISA))) + 1, 3.5) - 1);

	/*
	 * Next take the impact pressure equation and solve for Mach number:
	 *
	 * M = sqrt(5 * (((qc / P) + 1)^(2/7) - 1))
	 *
	 * Where qc is impact pressure and P is local static pressure.
	 */
	mach = sqrt(5 * (pow((qc / pressure) + 1, 0.2857142857142) - 1));

	/*
	 * Finally convert Mach number to true airspeed at local temperature.
	 */
	return (mach2ktas(mach, oat));
}

/*
 * Converts Mach number to equivalent airspeed (calibrated airspeed corrected
 * for compressibility).
 *
 * @param mach Mach number.
 * @param oat Static outside static air pressure in hPa.
 *
 * @return Equivalent airspeed in knots.
 */
double
mach2keas(double mach, double press)
{
	return (MPS2KTS(SPEED_SOUND_ISA * mach * sqrt(press / ISA_SL_PRESS)));
}

/*
 * Converts equivalent airspeed (calibrated airspeed corrected for
 * compressibility) to Mach number.
 *
 * @param mach Equivalent airspeed in knots.
 * @param oat Static outside static air pressure in hPa.
 *
 * @return Mach number.
 */
double
keas2mach(double keas, double press)
{
	/*
	 * Take the mach-to-EAS equation and solve for Mach number:
	 *
	 * M = Ve / (a0 * sqrt(P / P0))
	 *
	 * Where Ve is equivalent airspeed in m.s^-1, P is local static
	 * pressure and P0 is ISA sea level pressure (in hPa).
	 */
	return (KTS2MPS(keas) / (SPEED_SOUND_ISA * sqrt(press / ISA_SL_PRESS)));
}

/*
 * Calculates static air pressure from pressure altitude.
 *
 * @param alt Pressure altitude in feet.
 * @param qnh Local QNH in hPa.
 *
 * @return Air pressure in hPa.
 */
double
alt2press(double alt, double qnh)
{
	return (qnh * pow(1 - (ISA_TLR_PER_M * FEET2MET(alt)) / ISA_SL_TEMP_K,
	    (EARTH_GRAVITY * DRY_AIR_MOL) / (R_univ * ISA_TLR_PER_M)));
}

/*
 * Calculates pressure altitude from static air pressure.
 *
 * @param alt Static air pressure in hPa.
 * @param qnh Local QNH in hPa.
 *
 * @return Pressure altitude in feet.
 */
double
press2alt(double press, double qnh)
{
	return (MET2FEET((ISA_SL_TEMP_K * (1 - pow(press / qnh,
	    (R_univ * ISA_TLR_PER_M) / (EARTH_GRAVITY * DRY_AIR_MOL)))) /
	    ISA_TLR_PER_M));
}

/*
 * Converts pressure altitude to flight level.
 *
 * @param alt Pressure altitude in feet.
 * @param qnh Local QNH in hPa.
 *
 * @return Flight level number.
 */
double
alt2fl(double alt, double qnh)
{
	return (press2alt(alt2press(alt, qnh), ISA_SL_PRESS) / 100);
}

/*
 * Converts flight level to pressure altitude.
 *
 * @param fl Flight level number.
 * @param qnh Local QNH in hPa.
 *
 * @return Pressure altitude in feet.
 */
double
fl2alt(double fl, double qnh)
{
	return (press2alt(alt2press(fl * 100, ISA_SL_PRESS), qnh));
}

/*
 * Converts static air temperature to total air temperature.
 *
 * @param sat Static air temperature in degrees C.
 * @param mach Flight mach number.
 *
 * @return Total air temperature in degrees C.
 */
double
sat2tat(double sat, double mach)
{
	return (KELVIN2C(C2KELVIN(sat) * (1 + ((GAMMA - 1) / 2) * POW2(mach))));
}

/*
 * Converts total air temperature to static air temperature.
 *
 * @param tat Total air temperature in degrees C.
 * @param mach Flight mach number.
 *
 * @return Static air temperature in degrees C.
 */
double
tat2sat(double tat, double mach)
{
	return (KELVIN2C(C2KELVIN(tat) / (1 + ((GAMMA - 1) / 2) * POW2(mach))));
}

/*
 * Converts static air temperature to ISA deviation.
 *
 * @param fl Flight level (barometric altitude at QNE in 100s of ft).
 * @param sat Static air temperature in degrees C.
 *
 * @return ISA deviation in degress C.
 */
double
sat2isadev(double fl, double sat)
{
	return (sat - (ISA_SL_TEMP - ((fl / 10) * ISA_ELR_PER_1000)));
}

/*
 * Converts ISA deviation to static air temperature.
 *
 * @param fl Flight level (barometric altitude at QNE in 100s of ft).
 * @param isadev ISA deviation in degrees C.
 *
 * @return Local static air temperature.
 */
double
isadev2sat(double fl, double isadev)
{
	return (isadev + ISA_SL_TEMP - ((fl / 10) * ISA_ELR_PER_1000));
}

/*
 * Returns the speed of sound in m/s in dry air at `oat' degrees C (static).
 */
double
speed_sound(double oat)
{
	/*
	 * This is an approximation that for common flight temperatures
	 * (-65 to +65) is less than 0.1% off.
	 */
	return (20.05 * sqrt(C2KELVIN(oat)));
}

/*
 * Calculates air density.
 *
 * @param pressure Static air pressure in hPa.
 * @param oat Static outside air temperature in degrees C.
 *
 * @return Local air density in kg.m^-3.
 */
double
air_density(double pressure, double oat)
{
	/*
	 * Density of dry air is:
	 *
	 * rho = p / (R_spec * T)
	 *
	 * Where p is local static air pressure, R_spec is the specific gas
	 * constant for dry air and T is absolute temperature.
	 */
	return ((pressure * 100) / (R_spec * C2KELVIN(oat)));
}

/*
 * Calculates impact pressure. This is dynamic pressure with air
 * compressibility considered.
 *
 * @param pressure Static air pressure in hPa.
 * @param mach Flight mach number.
 *
 * @return Impact pressure in hPa.
 */
double
impact_press(double mach, double pressure)
{
	/*
	 * In isentropic flow, impact pressure for air (gamma = 1.4) is:
	 *
	 * qc = P((1 + 0.2 * M^2)^(7/2) - 1)
	 *
	 * Where P is local static air pressure and M is flight mach number.
	 */
	return (pressure * (pow(1 + 0.2 * POW2(mach), 3.5) - 1));
}

/*
 * Calculates dynamic pressure.
 *
 * @param pressure True airspeed in knots.
 * @param press Static air pressure in hPa.
 * @param oat Static outside air temperature in degrees C.
 *
 * @return Dynamic pressure in hPa.
 */
double
dyn_press(double ktas, double press, double oat)
{
	return (0.5 * air_density(press, oat) * POW2(KTS2MPS(ktas)) / 100);
}
