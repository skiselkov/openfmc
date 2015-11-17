#include <stdlib.h>
#include <string.h>
#include <assert.h>
#include <stdio.h>
#include <math.h>
#include <ctype.h>

#include "helpers.h"

bool_t
is_valid_vor_freq(double freq_mhz)
{
	int freq_khz = freq_mhz * 1000;

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
	/* 112 MHz is not allowed */
	if (freq_khz == 112000)
		return (0);
	/* Above 112 MHz, frequency must be multiple of 50 kHz */
	if (freq_khz % 50 != 0)
		return (0);

	return (1);
}

bool_t
is_valid_loc_freq(double freq_mhz)
{
	int freq_khz = freq_mhz * 1000;

	/* Check correct frequency band */
	if (freq_khz < 108100 || freq_khz > 111950)
		return (0);
	/* Check 200 kHz spacing with 100 kHz or 150 kHz remainder. */
	if (freq_khz % 200 != 100 && freq_khz % 200 != 150)
		return (0);

	return (1);
}

bool_t
is_valid_rwy_ID(const char *rwy_ID)
{
	char hdg_str[3];
	int hdg;
	int len = strlen(rwy_ID);

	if (len < 2 || len > 3 || !isdigit(rwy_ID[0]) || !isdigit(rwy_ID[1]))
		return (0);
	memcpy(hdg_str, rwy_ID, 2);
	hdg_str[2] = 0;
	hdg = atoi(hdg_str);
	if (hdg == 0 || hdg > 36)
		return (0);
	if (len == 3 && rwy_ID[2] != 'R' && rwy_ID[2] != 'L' &&
	    rwy_ID[2] != 'C') {
		return (0);
	}

	return (1);
}

char **
explode_line(char *line, char *delim, size_t *num_comps)
{
	size_t line_len = strlen(line);
	char **comps;
	size_t i, n;
	char *end = line + line_len;
	char *p;
	size_t delim_n = strlen(delim);

	for (n = 1, p = line; p < end; n++) {
		p = strstr(p, delim);
		if (p == NULL)
			break;
		else
			p += delim_n;
	}

	comps = malloc(sizeof (char *) * n);

	for (i = 0, p = line; p < end; i++) {
		assert(i < n);
		char *d = strstr(p, delim);
		comps[i] = p;
		if (d == NULL)
			break;
		d[0] = 0;
		p = d + delim_n;
	}

	*num_comps = n;
	return (comps);
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
