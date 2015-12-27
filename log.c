/*
 * CDDL HEADER START
 *
 * The contents of this file are subject to the terms of the
 * Common Development and Distribution License, Version 1.0 only
 * (the "License").  You may not use this file except in compliance
 * with the License.
 *
 * You can obtain a copy of the license in the file COPYING
 * or http://www.opensource.org/licenses/CDDL-1.0.
 * See the License for the specific language governing permissions
 * and limitations under the License.
 *
 * When distributing Covered Code, include this CDDL HEADER in each
 * file and include the License file COPYING.
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
