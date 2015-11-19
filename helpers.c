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

#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include <math.h>
#include <ctype.h>

#include "helpers.h"

bool_t
is_valid_vor_freq(double freq_mhz)
{
	unsigned freq_khz = freq_mhz * 1000;

	/* Check correct frequency band */
	if (freq_khz < 108000 || freq_khz > 117950)
		return (0);
	/*
	 * Check the LOC band - freq must be multiple of 200 kHz or
	 * remainder must be 50 kHz.
	 */
	if (freq_khz >= 108000 && freq_khz <= 112000 &&
	    freq_khz % 200 != 0 && freq_khz % 200 != 50)
		return (0);
	/* Above 112 MHz, frequency must be multiple of 50 kHz */
	if (freq_khz % 50 != 0)
		return (0);

	return (1);
}

bool_t
is_valid_loc_freq(double freq_mhz)
{
	unsigned freq_khz = freq_mhz * 1000;

	/* Check correct frequency band */
	if (freq_khz < 108100 || freq_khz > 111950)
		return (0);
	/* Check 200 kHz spacing with 100 kHz or 150 kHz remainder. */
	if (freq_khz % 200 != 100 && freq_khz % 200 != 150)
		return (0);

	return (1);
}

bool_t
is_valid_tacan_freq(double freq_mhz)
{
	unsigned freq_khz = freq_mhz * 1000;

	/* this is quite a guess! */
	if (freq_khz < 133000 || freq_khz > 136000 ||
	    freq_khz % 100 != 0)
		return (0);
	return (1);
}

bool_t
is_valid_ndb_freq(double freq_khz)
{
	unsigned freq_hz = freq_khz * 1000;
	/* 177 kHz for an NDB is the lowest I've ever seen */
	return (freq_hz >= 177000 && freq_hz <= 1750000);
}

/*
 * Checks a runway ID for correct formatting. Runway IDs are
 * 2-, 3- or 4-character strings conforming to the following format:
 *	*) Two-digit runway heading between 01 and 36. Headings less
 *	   than 10 are always prefixed by a '0'. "00" is NOT valid.
 *	*) An optional parallel runway discriminator, one of 'L', 'R' or 'C'.
 *	*) An optional true magnetic heading indicator 'T'. Runways
 *	   which use true headings must belong to an airport_t with
 *	   true_hdg set (indicated by passing the appropriate argument).
 */
bool_t
is_valid_rwy_ID(const char *rwy_ID)
{
	int hdg;
	int len = strlen(rwy_ID);

	if (len < 2 || len > 4 || !isdigit(rwy_ID[0]) || !isdigit(rwy_ID[1]))
		return (B_FALSE);
	hdg = atoi(rwy_ID);
	if (hdg == 0 || hdg > 36)
		return (B_FALSE);
	if (len == 3) {
		if (rwy_ID[2] != 'R' && rwy_ID[2] != 'L' &&
		    rwy_ID[2] != 'C' &&  rwy_ID[2] != 'T')
			return (B_FALSE);
	} else if (len == 4) {
		if ((rwy_ID[2] != 'R' && rwy_ID[2] != 'L' &&
		    rwy_ID[2] != 'C') || rwy_ID[3] != 'T')
			return (B_FALSE);
	}

	return (B_TRUE);
}

size_t
explode_line(char *line, char delim, char **comps, size_t capacity)
{
	size_t i = 1;

	ASSERT(capacity != 0);
	comps[0] = line;
	for (char *p = line; *p != 0; p++) {
		if (*p == delim) {
			if (i >= capacity)
				return ((size_t)-1);
			*p = 0;
			comps[i++] = p + 1;
		}
	}

	return (i);
}

void
strip_newline(char *line)
{
	size_t n = strlen(line);

	if (n >= 2 && line[n - 2] == '\r')
		/* cut off trailing CRLF */
		line[n - 2] = 0;
	else if (n >= 1 && line[n - 1] == '\n')
		/* cut off trailing LF */
		line[n - 1] = 0;
}

void
append_format(char **str, size_t *sz, const char *format, ...)
{
	va_list ap;
	int needed;

	va_start(ap, format);
	needed = vsnprintf(NULL, 0, format, ap);
	va_end(ap);

	va_start(ap, format);
	*str = realloc(*str, *sz + needed + 1);
	(void) vsnprintf(*str + *sz, *sz + needed + 1, format, ap);
	*sz += needed;
	va_end(ap);
}
