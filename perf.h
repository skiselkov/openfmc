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

#ifndef	_OPENFMC_PERF_H_
#define	_OPENFMC_PERF_H_

#include "geom.h"

#ifdef	__cplusplus
extern "C" {
#endif

/* Temperature scale conversions */
#define	KELVIN2C(k)	((k) - 273.15)
#define	C2KELVIN(c)	((c) + 273.15)
#define	FAH2C(f)	(((f) - 32) * 0.555555)
#define	C2FAH(c)	(((c) * 1.8) + 32)
#define	FAH2KELVIN(f)	(((f) + 459.67) * 0.5555555555)
#define	KELVIN2FAH(k)	(((k) * 1.8) - 459.67)

#define	KTS2MPS(k)	(NM2MET(k) / 3600)	/* knots to m.s^-1 */
#define	MPS2KTS(k)	(MET2NM(k) * 3600)	/* m.s^-1 to knots */

typedef struct {
	int	spd;
	double	Cd;
} drag_coeff_t;

typedef struct {
	char		*acft_type;

	double		ref_zfw;
	double		max_fuel;
	double		max_gw;

	char		*eng_type;

	/* Base max thrust in Kilonewtons @ ISA conditions */
	double		eng_max_thr;
	/*
	 * eng_max_thr fraction as a function of air density (in kg/m^3).
	 */
	bezier_t	*thr_dens_curve;
	/*
	 * eng_max_thr fraction as a function of ISA temperature deviation
	 * in degrees C.
	 */
	bezier_t	*thr_isa_curve;
	/*
	 * Engine specific fuel consumption in kg/hr as a function of
	 * thrust in Kilonewtons.
	 */
	bezier_t	*sfc_thr_curve;
	/*
	 * Engine specific fuel consumption modifier (0 - 1) as a function
	 * of air density (in kg/m^3).
	 */
	bezier_t	*sfc_dens_curve;
	/*
	 * Engine specific fuel consumption modifier (0 - 1) as a function
	 * of ISA temperature deviation in degrees C.
	 */
	bezier_t	*sfc_isa_curve;
} acft_perf_t;

typedef struct {
	double	crz_lvl;
	double	crz_tas;
	double	thr_derate;	/* fraction of eng_max_thr */
} flt_perf_t;

acft_perf_t *acft_perf_parse(const char *filename);
void acft_perf_destroy(acft_perf_t *perf);

double eng_max_thr_avg(const flt_perf_t *flt, acft_perf_t *acft, double alt1,
    double alt2, double ktas, double qnh, double isadev, double tp_alt);

double alt2press(double alt, double qnh);
double press2alt(double press, double qnh);

double alt2fl(double alt, double qnh);
double fl2alt(double alt, double qnh);

double ktas2mach(double ktas, double oat);
double mach2ktas(double mach, double oat);

double ktas2kcas(double ktas, double pressure, double oat);
double kcas2ktas(double kcas, double pressure, double oat);

double mach2keas(double mach, double press);
double keas2mach(double keas, double press);

double sat2tat(double sat, double mach);
double tat2sat(double tat, double mach);

double sat2isadev(double fl, double sat);
double isadev2sat(double fl, double isadev);

double speed_sound(double oat);
double air_density(double pressure, double oat);
double impact_press(double mach, double pressure);
double dyn_press(double ktas, double press, double oat);

#ifdef	__cplusplus
}
#endif

#endif	/* _OPENFMC_PERF_H_ */
