#include <stdio.h>
#include <string.h>
#include <stdarg.h>

#include "log.h"

void
openfmc_log(openfmc_log_lvl_t lvl, const char *fmt, ...)
{
	va_list ap;
	va_start(ap, fmt);
	openfmc_log_v(lvl, fmt, ap);
	va_end(ap);
}

void
openfmc_log_v(openfmc_log_lvl_t lvl, const char *fmt, va_list ap)
{
	vfprintf(lvl <= OPENFMC_LOG_WARN ? stderr : stdout, fmt, ap);
	if (fmt[strlen(fmt) - 1] != '\n')
		fprintf(lvl <= OPENFMC_LOG_WARN ? stderr : stdout, "\n");
}
